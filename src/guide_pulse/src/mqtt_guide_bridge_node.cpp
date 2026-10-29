// ============================================================================
// MQTT <-> 展厅讲解 桥接节点实现。
// 框架（连接线程/遗嘱/心跳/队列/幂等）沿用 tts_demo，删 condition 状态机，
// 新增 scene（迎宾）/navigate（导航+汽车讲解）分发与到达事件消费。
// ============================================================================
#include "guide_pulse/mqtt_guide_bridge_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <map>
#include <random>
#include <set>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include "guide_pulse/agent_monitor.hpp"  // normalize_phrase_text/phrase_matches（迎宾三场景路由匹配）
#include "guide_pulse/service_call.hpp"

namespace guide_pulse
{

using nlohmann::json;

namespace
{

// 模板话术占位替换：把 text 里全部 {point} 换成 point（拒绝提示含原始
// 点位名，现场能听到缺哪个点；话术无占位符则原样返回）
std::string replace_point_placeholder(
  const std::string & text, const std::string & point)
{
  static const std::string kToken = "{point}";
  std::string out = text;
  for (auto pos = out.find(kToken); pos != std::string::npos;
    pos = out.find(kToken, pos + point.size()))
  {
    out.replace(pos, kToken.size(), point);
  }
  return out;
}

// MQTT 报文日志打码（与 tts_demo / robot_com 网关一致）：
// 凭证类字段替换为 <redacted>，避免 token 泄漏进日志；非 JSON 原样返回
std::string log_safe_payload(const std::string & payload)
{
  try {
    auto value = json::parse(payload);
    if (value.is_object() && value.contains("payload") && value.at("payload").is_object()) {
      auto & inner = value.at("payload");
      for (const char * field : {"upload_token", "download_token", "participant_token"}) {
        if (inner.contains(field)) {
          inner[field] = "<redacted>";
        }
      }
    }
    return value.dump();
  } catch (const json::exception &) {
    return payload;
  }
}

}  // namespace

MqttGuideBridgeNode::MqttGuideBridgeNode(const rclcpp::NodeOptions & options)
: Node("guide_bridge", options)
{
  declare_and_load_params();
}

MqttGuideBridgeNode::~MqttGuideBridgeNode()
{
  shutdown();
}

void MqttGuideBridgeNode::declare_and_load_params()
{
  // ---- MQTT 连接与 enroll 注册面参数（协议 v0.9.0/v0.10.0：凭预登记 SN
  // 换取当次运行的 robot_id 与 MQTT 账号密码，静态共享账号已废除）----
  declare_parameter<std::string>("broker_uri", "tcp://localhost:11883");
  declare_parameter<std::string>("enroll_origin", "");
  declare_parameter<std::string>("enroll_origin_env", "GUIDE_PULSE_ENROLL_ORIGIN");
  declare_parameter<std::string>("serial_number", "");
  declare_parameter<std::string>("serial_number_env", "GUIDE_PULSE_SERIAL_NUMBER");
  // 板端 SN 文件（批量部署自动获取）：参数与环境变量都未提供时读取
  declare_parameter<std::string>("serial_number_file", "/robot/info/sn");
  declare_parameter<int>("enroll_timeout_seconds", 5);
  // RobotCar Server 固定 TopicPrefix=robotCar（写死在 Server 代码常量，
  // 非部署配置；见 RobotCar/Server/internal/mqttprotocol/topic.go）
  declare_parameter<std::string>("topic_prefix", "robotCar/v1/robots");
  declare_parameter<std::string>("client_version", "0.1.0");
  declare_parameter<int>("keep_alive_seconds", 30);
  declare_parameter<int>("connect_timeout_seconds", 10);
  declare_parameter<int>("heartbeat_interval_seconds", 30);

  // ---- TTS 相关参数（沿用 tts_demo）----
  declare_parameter<std::string>(
    "tts_service_name", "/aimdk_5Fmsgs/srv/PlayTts");  // SDK 服务，勿改
  declare_parameter<std::string>("tts_domain", "guide_pulse");
  // SSML 静音段（300ms 无声、任务正常完结）：抢占停止文本。
  // 勿用纯标点"。"——任务无法完结会把 SDK 仲裁通道卡住 30-60 秒
  declare_parameter<std::string>(
    "stop_text", "<speak><break time=\"300ms\"/></speak>");
  // SDK 仲裁：新权重(level*100+weight) < 当前任务时请求被丢弃
  declare_parameter<int>("tts_priority_level", 7);
  declare_parameter<int>("tts_priority_weight", 0);
  // 列表播报被 SDK 仲裁拒绝后的快速重试间隔（抢占空闲窗口）
  declare_parameter<int>("tts_retry_seconds", 3);
  declare_parameter<int>("tts_request_timeout_seconds", 5);
  declare_parameter<int>("tts_wait_timeout_seconds", 0);
  declare_parameter<int>("dedupe_capacity", 1024);

  // ---- 唤醒抑制（迎宾/讲解单轮播报防强打断，v0.23.0）----
  declare_parameter<std::string>(
    "suppress_service_name", "/aimdk_5Fmsgs/srv/SetAgentSuppression");
  declare_parameter<bool>("suppress_wakeup", true);
  declare_parameter<int>("suppress_ttl_ms", 8000);
  declare_parameter<int>("suppress_renew_interval_ms", 5000);
  // 尾部宽限估算模型：SDK 无播完事件（estimated_duration 恒 0 且无播完
  // 话题，见 接口说明.md PlayTts 节），只能字数估算 + 参数化真机校准。
  // 2026-10-16 校准：受理≠出声（云端合成 1~3s）需 lead 计入；scale=1.0
  // 不缩水（过短导致播报中唤醒恢复撞音，宁可多禁一会）
  declare_parameter<int>("tts_speech_ms_per_char", 220);
  declare_parameter<int>("suppress_tail_lead_ms", 2500);
  declare_parameter<double>("suppress_tail_scale", 1.0);
  declare_parameter<int>("suppress_tail_max_ms", 0);
  // 停止续期时刷的收尾抑制 TTL（注意 0=云端默认 3000ms；1=立即解除）
  declare_parameter<int>("suppress_tail_flush_ms", 1);
  // 播报期间 SSH 静音运控板麦克风（2026-10-16：唤醒抑制拦不住旧对话轮
  // 直送云端的强打断 request_stop，无音频进入即无打断之源；SSH 目标/
  // 密码复用 display_remote_*）。作用范围（2026-10-17 现场需求）：仅
  // 迎宾讲解与导航到达讲解词，其余播报（App tts/重定位/各类提示）不静音
  declare_parameter<bool>("mic_mute_enabled", true);

  // ---- 屏幕二维码显示（迎宾场景，v0.22.0）----
  declare_parameter<std::string>(
    "display_emotion_service_name", "/aimdk_5Fmsgs/srv/PlayEmotion");
  declare_parameter<std::string>("display_remote_host", "10.1.1.100");
  declare_parameter<std::string>("display_remote_user", "run");
  declare_parameter<std::string>(
    "display_remote_pass", "");  // 空禁用显示；亦可环境变量注入后写入参数
  declare_parameter<std::string>("display_remote_dir", "/home/run/screen_display");
  declare_parameter<int>("display_request_timeout_seconds", 5);
  // 二维码常驻显示统一超时（2026-10-22 现场需求：不分流程——语料触发
  // show_qr / 迎宾 / 送客二维码一律 2 分钟到点撤屏恢复默认表情；切到
  // 下一场景（导览/迎宾/送客等）不等超时，各业务入口先 cancel 让位）
  declare_parameter<int>("display_show_timeout_seconds", 120);  // 2 分钟
  // 二维码业务（show_qr/cancel_qr）互斥窗口秒数（2026-10-17 现场需求：
  // 显示与取消二维码显示是两个业务，各占此窗口与其他业务互斥；默认
  // 20s。只管互斥占用，二维码常驻时长仍由 display_show_timeout_seconds
  // 独立计时互不影响）
  declare_parameter<int>("qr_business_timeout_seconds", 20);
  declare_parameter<int>("display_restore_emotion_id", 90);     // T1 默认表情
  // 取消二维码恢复表情的 PlayEmotion 优先级（2026-10-23 真机故障修复：
  // 打断语义 = 高优先级打断低优先级，二维码以 priority=0 上屏，取消
  // 同为 0 时运控板受理 ok 但不打断循环视频，屏幕滞留二维码——触发
  // 车型导览后二维码不消失即此因）。须大于 0；失败自动重试不受此影响
  declare_parameter<int>("display_restore_priority", 1);

  // ---- 迎宾二维码缓存/下载（v0.22.0 P-036：scene 带 qr_sha256 哈希快照，
  // 未命中本地缓存时以 MQTT 凭据从 Server 文件面拉取图片）----
  declare_parameter<std::string>("qr_cache_dir", "/tmp/guide_pulse/qr_cache");
  declare_parameter<int>("qr_download_timeout_seconds", 5);
  // 未携带 qr_sha256 的迎宾（含语料触发）显示的默认二维码图片。
  // 默认空 = 自动使用随包安装的 config/QRCode.png；配成不存在的路径可禁用
  declare_parameter<std::string>("display_default_qr_path", "");

  // ---- 展厅讲解增量参数（docs/design.md §5）----
  declare_parameter<std::string>("nav_executor", "mock");
  declare_parameter<int>("mock_nav_seconds", 0);
  // Mock 重定位验收等待窗口：受理后前 N 次 poll 返回「尚无结果」（每次
  // poll 间隔 ~250ms），模拟真机 ~1.5s 验收延迟，供集成测试打开「重定位
  // 进行中」的拒绝窗口；0 = 受理即验收通过
  declare_parameter<int>("mock_relocate_hold_polls", 0);
  // 重定位等待导航模块验收的超时（2026-10-17 需求：业务终态 = 导航栈
  // set_pose goal 的 result（验收完成才回，正常 ~1.5s，最多 ~3s），
  // 30s 兜底 result 断流/服务卡死；超时按 failed/localization_failed 收尾）
  declare_parameter<int>("relocate_timeout_seconds", 30);
  // 变形后重定位发送前等待秒数（2026-10-21 现场需求）：变形动作落位后
  // 机器人还有余晃，立即发 set_pose 重设初值会被余晃期抖动干扰，先等
  // 这么多秒再发；0 = 原行为（落位即发）
  declare_parameter<int>("relocate_after_transform_delay_seconds", 2);
  // 导航在途兜底超时：navigate goal 受理后等待 result 终态
  //（SUCCEEDED/FAILED/CANCELLED）的最长时间。正常展厅导航几十秒内到；
  // 超时按 navigation_failed 收尾并释放导览业务——导航栈断流/卡死时
  // 等待期所有新业务被 failed/busy 拒绝，600s 过长会让现场感受为
  //「一直拒绝」，可按场地规模调小（下限 30s，防误配 0 关掉兜底）
  declare_parameter<int>("nav_timeout_seconds", 600);
  // 业务互斥拒绝时的语音提示（2026-10-17 需求）：仅非语音业务进行中播
  //（重定位/导航在途——机器人没在说话）；迎宾/播报/讲解中正在出声不播。
  // 空串 = 禁用对应提示
  declare_parameter<std::string>("busy_hint_relocate_text", "我正在重定位，请稍后");
  declare_parameter<std::string>("busy_hint_navigate_text", "我正在导航，请稍后");
  // 导航/重定位失败时的语音提示（2026-10-17 现场需求：导航栈异常
  //（服务不可用/超时/验收失败）现场要能听到，提示排查 navigation
  // service）；回执照发，播报属本地增强。空串 = 禁用对应提示
  declare_parameter<std::string>(
    "navigate_failed_text", "导航失败，请检查navigation service是否正常");
  // 导航 goal 被拒（航点表无该点位）时的语音提示（2026-10-19 现场需求：
  // t1_slam_nav_ws 拒绝「路线里没有叫 X 的航点」属点位未配置而非导航栈
  // 异常，播报要指引配置点位而不是排查服务；回执照发，播报属本地增强。
  // 空串 = 禁用播报
  declare_parameter<std::string>(
    "navigate_waypoint_missing_text", "还未设置该点位信息哦，请设置相应点位后再次尝试");
  // 变形未确认双足即拒绝下发导航/重定位时的语音提示（2026-10-18 现场
  // 需求：导航指令必须等双足形态确认后才发给 t1_slam_nav_ws；运控
  // 不可用/变形被拒/动作未到位等场景现场要能听到，提示排查运控与
  // 机器人姿态）；回执照发（failed/transform_failed），播报属本地
  // 增强。空串 = 禁用播报
  declare_parameter<std::string>(
    "navigate_transform_failed_text", "变形未完成，暂时无法导航，请检查机器人姿态");
  declare_parameter<std::string>(
    "relocate_failed_text", "重定位失败，请检查navigation service是否正常");
  // 导览受理成功后的引导语（2026-10-17 现场需求）：确认 [导航] 收到
  // 应答 accepted=true 后 TTS 播报「请跟我来…」带客人跟走；{point}
  // 替换点位显示名（映射表 name，如「比亚迪 M9 DM-i」；查不到用原始
  // 点位名）。空串 = 禁用。MQTT 下行与语料触发共用
  declare_parameter<std::string>(
    "navigate_lead_text", "请跟我来，让我带您去看看{point}这款车");
  // 导航目标为迎宾点时的引导语（2026-10-19 现场需求）：App 下发
  // navigate 去迎宾点（如 point_name=Y），普通引导语「请跟我来…去看看
  // 迎宾点这款车」语义不通（迎宾点不是车型），改播这句。判定：code
  // 命中 welcome_point_code 或点位显示名含
  // 「迎宾」。空串 = 回退普通引导语
  declare_parameter<std::string>(
    "navigate_lead_welcome_text", "好嘞，马上前去迎宾点");
  // 导航出发播报语（2026-10-18 现场需求）：收到导航指令（无论语音还是
  // MQTT）受理后先播这句再起身变形人形（先开口、立刻动）；迎宾准备去
  // Y 点的导航不播（agent 已说「整装待发，即刻就位」，再说就重复，
  // handle_welcome_prep 独立链路天然跳过）。空串 = 禁用
  declare_parameter<std::string>("navigate_depart_text", "稍等，马上出发");
  // 重定位出发播报语（2026-10-18 现场需求）：仅 MQTT 下行的 relocate 先
  // 播这句再起身变形人形；语料触发不播（agent 已说「好的，这就在A点
  // 开始重定位」）。空串 = 禁用
  declare_parameter<std::string>(
    "relocate_depart_text", "马上开始重定位，请稍后");
  // ---- 导航途中随机播报（2026-10-21 现场需求）----
  // 安抚语料池：受理引导语播完后，每隔 min~max 秒（随机）挑一句播报
  //（随机取、不与上一条连续重复），安抚顾客「不要着急、跟我走」，
  // 防导航途中长时间沉默太空；到达/取消/失败即停。空列表 = 禁用
  // 2026-10-22 现场需求：清道话术「小T正在赶路，麻烦大家给我让点
  // 空间吧」入池——固定首句播完后途中也要轮播到这句
  declare_parameter<std::vector<std::string>>(
    "navigate_chatter_texts",
    std::vector<std::string>{
      "请不用着急，跟着我慢慢走哦",
      "马上就到了，请跟紧我",
      "请注意脚下，跟我来",
      "别着急，我们很快就到啦",
      "小T正在赶路，麻烦大家给我让点空间吧",
    });
  // 去迎宾点专用池（2026-10-21 现场需求：去迎宾点途中没有顾客跟走，
  // 「请跟我走」话术不对，改播赶路/就位类的话；迎宾准备（语料触发）
  // 与 App 下发 navigate 去迎宾点共用）。空列表 = 去迎宾点途中不播
  declare_parameter<std::vector<std::string>>(
    "navigate_chatter_welcome_texts",
    std::vector<std::string>{
      "我正赶往迎宾点，马上就到啦",
      "请大家稍稍让一让，我赶时间去迎宾点哦",
      "小T正在赶路，马上就到迎宾点啦",
    });
  // 途中播报固定首句（2026-10-21 现场需求）：受理引导语播完后第一次
  // 到播报点先播这句固定清道话术（机器人起步时请围观路人让行，
  // 「要出发啦」的移动预警比单纯「留点空间」更让路人明白为什么让），
  // 播完回到随机池。空串 = 不固定，从第一句起就随机
  declare_parameter<std::string>(
    "navigate_chatter_first_text", "小T要出发啦，麻烦大家给我让点空间吧");
  // 去迎宾点的固定首句（同上，赶路清道口径——迎宾准备场景 agent 已说
  // 「整装待发，即刻就位」，这句是对路人的清道补充，不冲突）。
  // 空串 = 不固定
  declare_parameter<std::string>(
    "navigate_chatter_welcome_first_text", "大家让一让，小T赶时间去迎宾点啦");
  // 一句播完后到下一句的静默秒数（2026-10-21 现场需求：间隔指两句之间
  // 的静默，不含播报本身的时间；下一句时刻 = 本句播报时长估算
  //（受理出声延迟 + 220ms/字，同尾部宽限公式）+ 本静默值）
  declare_parameter<int>("navigate_chatter_silence_seconds", 5);
  declare_parameter<int>("tour_speak_interval_seconds", 2);
  // 点位映射表落盘文件（v0.25.0 P-039 point_map 全量替换 + 跨重启保留；
  // v0.28.0 P-042 起含播放词一起落盘）。默认 /robot/persist/var/guide_pulse/
  //（平台持久化数据分区 ext4，断电不丢）；PointMapStore 会自动建目录，
  // 文件不存在 = 空表合法（等 App 推表），重启不丢
  declare_parameter<std::string>(
    "point_map_file", "/robot/persist/var/guide_pulse/point_map.json");
  // navigate 到达而映射表为空（App 未推表）时的本地提示话术：播报后
  // 回执 navigate.result/failed:point_not_found，不启动导航（2026-10-17
  // 现场需求：配置缺失要能被现场听到、看到）
  declare_parameter<std::string>(
    "point_map_missing_text", "无法执行导览工作，请配置点位映射表哦");
  // navigate/relocate 点位名未命中（表空或名称不在表中）时的本地提示
  // 话术：{point} 占位替换原始 point_name，播报后回执
  // failed:point_not_found，不启动导航/重定位
  //（2026-10-17 现场需求：映射表是唯一权威，未命中直接拒绝）。
  // 空串 = 禁用播报（回执照发）
  declare_parameter<std::string>(
    "point_map_unmatched_text", "未找到点位{point}，请重新配置映射表");
  // 迎宾点 code（2026-10-19 现场需求：迎宾词强制以映射表下发为准）：
  // scene 未带 texts 时取映射表该 code 条目的 text 作迎宾词（如 Y=
  // 迎宾点 条目的播放词）；表未配 text 则本次迎宾不播词（引导语/
  // 变形/二维码/回执照常）。空串 = 禁用映射表迎宾词
  declare_parameter<std::string>("welcome_point_code", "Y");
  // 迎宾引导语（2026-10-17 现场需求：先出声招呼、再起身变形的迎宾节奏）：
  // scene=welcome 到达后先播引导语，播完（TtsStatus 终态回流/字数估算
  // 兜底）再变形，变形完成播迎宾词。空串 = 禁用（直接进变形，回退原
  // 时序）。MQTT 下行与语料触发共用
  declare_parameter<std::string>(
    "welcome_lead_text", "哈喽哈喽！电池满格，热情也满格");
  // 迎宾准备就位播报语（2026-10-18 现场需求）：语料触发导航去迎宾点，
  // 到达变四足（摄像头收平）后最后播报。空串 = 禁用
  declare_parameter<std::string>(
    "welcome_prep_done_text", "元气满满，做好准备迎接顾客");
  // 送客到达播报语（2026-10-21 现场需求；2026-10-22 换长文案，语料
  // 提及「扫码收选车小资料」——与送客二维码上屏配套）：语料触发送顾客
  // 回迎宾点，到达变四足（摄像头收平）后最后播报。空串 = 禁用（到达即
  // 释放业务）
  declare_parameter<std::string>(
    "send_off_done_text",
    "欢乐的时光过的特别快，又到时间讲拜拜！祝您一路绿灯、续航永不掉线，"
    "希望有我的陪伴，能给你带去满满的小惊喜，您可以扫码收下我们的选车"
    "小资料，方便您回去再慢慢对比，如果有需要可以随时和我们联系。");
  // 原地送客虚拟点位 code（2026-10-28 现场需求）：App 下发 navigate 去
  // 该点位（默认 Z）不导航——Z 是虚拟点位（原地送客仪式，无需存在于
  // 点位映射表/航点表），原地完成 变形双足 + 播送客语 send_off_done_text
  //（伴随右手挥手）+ 送客二维码上屏，见 handle_inplace_send_off。
  // 空串 = 关闭分流（Z 按普通点位走映射表解析链路）
  declare_parameter<std::string>("send_off_point_code", "Z");
  // 原地送客播报伴随的挥手动作 id（McPresetMotion 枚举：1001=右手
  // 挥手）。0 = 禁用（只播语音不挥手）
  declare_parameter<int>("send_off_wave_motion_id", 1001);
  // 原地播报虚拟点位 code（2026-10-28 现场需求，与 Z 点原地送客同款
  // 分流机制）：App 下发 navigate 去该点位（默认 W）不导航——W 是
  // 虚拟航点（无需存在于航点表），但播报语料须配置在点位映射表该
  // code 条目的 text 播放词（App 推表改词即生效；MQTT 消息只作触发，
  // 携带的 texts 不作播报内容），原地完成 变形双足 + 播报该语料
  //（伴随说话动作轮换池 motion_preset_ids，由 play_current 的 source
  // 前缀分流自动触发，含 motion_repeat_seconds 续播），见
  // handle_inplace_broadcast。空串 = 关闭分流（W 按普通点位走映射表
  // 解析链路）
  declare_parameter<std::string>("broadcast_point_code", "W");
  // 迎宾准备到达后延迟变形秒数（2026-10-18 测试需求）：到达 Y 点收到
  // 导航 result 后不立即变形，等这么多秒再变四足。0 = 原行为（到达即变形）
  declare_parameter<int>("welcome_prep_transform_delay_seconds", 0);
  // 迎宾流程兜底超时：正常路径播报终态回流（TtsStatus）即释放业务占用
  //（worker 的 tts_early 分支），此参数仅在终态事件丢失（BEST_EFFORT）
  // 时兜底——到期退出迎宾占用，后续 navigate/tts 不再被 business_busy
  // 拒绝；到期不停播报（单轮自然播完）、不影响二维码常驻
  //（display_show_timeout_seconds 独立计时）。须覆盖最坏流程：
  // 变形 60s + 默认词播报 ~26s，120s 留余量
  declare_parameter<int>("welcome_flow_timeout_seconds", 120);

  // ---- AgentMonitor 语料触发（原厂 agent ASR 识别命中 -> 本地注入等价
  // 指令；配置由 main.cpp 读取注入 AgentMonitor，见 agent_monitor.hpp）----
  declare_parameter<std::string>(
    "agent_monitor.asr_topic", "/aima/agent/asr_result");
  declare_parameter<int>("agent_monitor.cooldown_seconds", 10);
  // 命中任一语料 -> 迎宾流程（等价 scene=welcome）。空列表 = 关闭语料触发
  declare_parameter<std::vector<std::string>>("agent_monitor.welcome_phrases",
    std::vector<std::string>{});
  // 命中任一语料 -> 屏幕显示默认二维码 display_show_timeout_seconds
  //（默认 120s=2 分钟，2026-10-22 现场需求统一时长）。空列表 = 关闭
  declare_parameter<std::vector<std::string>>("agent_monitor.qr_phrases",
    std::vector<std::string>{});
  // 命中任一语料 -> 取消二维码显示（2026-10-17 现场需求：agent 说
  //「好的，这就取消二维码显示」-> 恢复默认表情撤屏，未显示 no-op）。
  // 空列表 = 关闭
  declare_parameter<std::vector<std::string>>("agent_monitor.qr_cancel_phrases",
    std::vector<std::string>{});
  // 命中语料 -> 在对应航点上重定位（NavExecutor::set_pose）。每条格式
  // 「航点名|语料」（航点名 = t1_slam_nav_ws 航点文件点位名，如 A）。
  // 空列表 = 关闭。main.cpp 解析注入 AgentMonitor 规则
  declare_parameter<std::vector<std::string>>("agent_monitor.relocate_phrases",
    std::vector<std::string>{});
  // 命中语料 -> 导航到对应点位（2026-10-17 现场需求：agent 复述车型
  // 名「好的，您是想了解下腾势 B5这款车吗，请稍等」-> 查点位映射表
  // 下发导航）。每条格式「点位名|语料」，点位名 = 点位映射表 name/
  // code（如「腾势 B5」，handle_navigate 经映射表两级解析取 code）。
  // 空列表 = 关闭。main.cpp 解析注入 AgentMonitor 规则。
  // 2026-10-19 现场需求：导航语料默认由点位映射表动态生成（见下方
  // navigate_phrase_template，点位设置下发即更新，无需在此静态罗列），
  // 本参数仅作表未下发时的静态兜底/联调手写触发用
  declare_parameter<std::vector<std::string>>("agent_monitor.navigate_phrases",
    std::vector<std::string>{});
  // 导航语料动态生成模板（2026-10-19 现场需求）：每次点位设置
  //（point_map）下发后，除迎宾点（welcome_point_code）外，每个点位按
  // 本模板生成匹配语料，{point} 替换映射表 name（显示名，如「比亚迪
  // ætəʊ 1」-> 语料「要去看比亚迪ætəʊ 1，没问题」）。语料随表走：
  // App 改点位即生效，无需改 yaml 重启；匹配经归一化忽略空格/标点/
  // 大小写差异。空串 = 关闭动态生成（回退 navigate_phrases 静态配置）
  declare_parameter<std::string>(
    "agent_monitor.navigate_phrase_template", "要去看{point}，没问题");
  // 命中任一语料 -> 停止导航（等价本地注入 navigate.stop，2026-10-17
  // 现场需求：agent 说「这就停止展厅讲解」-> 取消在途导航，无导航
  // no-op，不发协议回执）。空列表 = 关闭
  declare_parameter<std::vector<std::string>>(
    "agent_monitor.navigate_stop_phrases",
    std::vector<std::string>{});
  // 迎宾准备语料（2026-10-18 现场需求）：agent 说「整装待发，即刻就位
  // 需要先导航去Y点」类话术 -> 等 agent 说完 -> 先确保人形（导航前置）
  // -> 导航去 point 迎宾点 -> 到达变四足收摄像头 -> 播就位语。每条格式
  // 「点位名|语料」，点位名 = 点位映射表 name/code（如 Y）。空列表 = 关闭
  declare_parameter<std::vector<std::string>>(
    "agent_monitor.welcome_prep_phrases", std::vector<std::string>{});
  // 送客语料（2026-10-21 现场需求）：agent 说「热情相送，宾至如归」
  // 类话术 -> 等 agent 说完 -> 先确保人形（导航前置）-> 导航去 point
  // 迎宾点送顾客 -> 到达变四足收摄像头 -> 播感谢语 send_off_done_text。
  // 每条格式「点位名|语料」，点位名写法与 welcome_prep_phrases 一致
  //（名称关键词如「迎宾」，或直接写 code 如 Y，精确解析优先）。
  // 空列表 = 关闭
  declare_parameter<std::vector<std::string>>(
    "agent_monitor.send_off_phrases", std::vector<std::string>{});
  // 按 trace_id 回查 TTS 话术原文（运控板日志，复用 display_remote_* SSH 配置）
  declare_parameter<bool>("agent_monitor.fetch_tts_text", true);

  // ---- 迎宾变形（scene=welcome 时确保人形形态，直调厂家运控）----
  // 参数由 McTransform 模块消费（构造见 start()），此处仅声明默认值
  declare_parameter<bool>("welcome_transform", true);
  // 趴姿（被动/趴下/坐下/阻尼/躺下）是否一路扶起再变形。迎宾由后端在
  // 有人到场时触发，与「远程下发航点」场景不同，默认扶起；false 则趴着
  // 只播报不变形（与导航栈 ensure_biped 的保守策略一致）
  declare_parameter<bool>("welcome_stand_from_down", true);
  declare_parameter<std::string>(
    "get_action_service", "/aimdk_5Fmsgs/srv/GetMcAction");  // SDK 固定服务名
  declare_parameter<std::string>(
    "set_action_service", "/aimdk_5Fmsgs/srv/SetMcAction");  // SDK 固定服务名
  // SetMcAction.request.source：非空即可（无需 SetMcInputSource 注册）。
  // 用独立名字便于在运控日志里分辨「这条变形是谁发的」
  declare_parameter<std::string>("mc_source_name", "guide_pulse");
  // 变形全程时限：最长路径五步（阻尼/躺下 -> 被动 -> 狗站立 -> 行走态 ->
  // 变形），狗站立三步实测十余秒，60s 留足余量
  declare_parameter<int>("transform_wait_timeout_seconds", 60);
  // 摄像头舵机：头部相机随形态翻转（0 度=四足收平，90 度=双足竖起），
  // 迎宾流程到达人形后竖起（幂等，已是人形也调）
  declare_parameter<bool>("welcome_camera_servo", true);  // false 关闭
  declare_parameter<int>("welcome_camera_position", 90);  // 角度 0-90
  declare_parameter<int>("welcome_camera_speed", 500);    // 速度 100-1000
  // 四足（迎宾准备模式收摄像头）时舵机角度：SDK 定义 0 度 = 四足形态位置
  declare_parameter<int>("welcome_prep_camera_position", 0);
  declare_parameter<std::string>(
    "set_servo_service", "/aimdk_5Fmsgs/srv/SetServo");  // SDK 固定服务名
  // 变形音：要发变形动作前后台推厂家官方 WAV 的 PCM 到 HAL 播放话题
  //（对齐导航栈速度桥「先开口、立刻发动作」；2026-10-16 补齐迎宾直调
  // 路径缺的这声）。失败只记日志，绝不阻断变形
  declare_parameter<bool>("welcome_transform_sound", true);
  // 变形音 wav 在**运控板上**的绝对路径（SSH paplay 播放，走运控板
  // PulseAudio；固件 v2d0 起 /hal_audio 没了，本机推 PCM 无人消费）。
  // 空 = 厂家官方 transform4to2.wav（App 变形同款，约 1.8s）
  declare_parameter<std::string>("transform_sound_path", "");
  // 人 -> 狗方向变形音 wav 路径（2026-10-18 迎宾准备四足变形）。
  // 空 = 同目录 transform2to4.wav（按 4to2 命名推定，真机校准点；
  // 文件不存在仅一条 WARN 不影响变形）
  declare_parameter<std::string>("transform_down_sound_path", "");

  // ---- 播报伴随动作（TTS 受理成功时触发预设动作）----
  declare_parameter<bool>("motion_enabled", true);
  // 预设动作轮换池（McPresetMotion 枚举值：1001=右手挥手，2003=双手
  // 说话动作（幅度小，已弃用），2004~2009=T1 说话动作 1~6（幅度更大，
  // 2026-10-18 现场需求：讲解动作加大改用此族）；多条时每次播报轮换
  // 一个，现场看效果可删减或固定某一个
  declare_parameter<std::vector<int64_t>>(
    "motion_preset_ids",
    std::vector<int64_t>{2004, 2005, 2006, 2007, 2008, 2009});
  declare_parameter<std::string>(
    "motion_service_name", "/aimdk_5Fmsgs/srv/SetMcPresetMotion");
  declare_parameter<int>("motion_request_timeout_seconds", 2);
  // 动作续播间隔秒（2026-10-28 现场需求：讲解句长动作短，手部动作
  // 太短）：播报期间每隔此秒数续触轮换池下一个动作直到本句播完；
  // 0 = 禁用（每句受理时只触发一次，原行为）
  declare_parameter<int>("motion_repeat_seconds", 6);

  broker_uri_ = get_parameter("broker_uri").as_string();
  enroll_origin_ = get_parameter("enroll_origin").as_string();
  serial_number_ = get_parameter("serial_number").as_string();

  // enroll 面地址 / 序列号支持环境变量注入（参数优先，空值回退环境变量）
  const auto enroll_origin_env = get_parameter("enroll_origin_env").as_string();
  if (enroll_origin_.empty() && !enroll_origin_env.empty()) {
    if (const char * value = std::getenv(enroll_origin_env.c_str())) {
      enroll_origin_ = value;
    }
  }
  const auto serial_number_env = get_parameter("serial_number_env").as_string();
  if (serial_number_.empty() && !serial_number_env.empty()) {
    if (const char * value = std::getenv(serial_number_env.c_str())) {
      serial_number_ = value;
    }
  }

  // 第三级回退：板端 SN 文件（批量部署不写配置的关键来源）。
  // 优先级：serial_number 参数 > 环境变量 > serial_number_file 文件
  const auto serial_number_file = get_parameter("serial_number_file").as_string();
  if (serial_number_.empty() && !serial_number_file.empty()) {
    if (const auto file_sn = read_serial_number_file(serial_number_file)) {
      serial_number_ = *file_sn;
      RCLCPP_INFO(
        get_logger(), "从文件 %s 读取 serial_number=%s",
        serial_number_file.c_str(), serial_number_.c_str());
    }
  }

  topic_prefix_ = get_parameter("topic_prefix").as_string();
  client_version_ = get_parameter("client_version").as_string();
  tts_service_name_ = get_parameter("tts_service_name").as_string();
  tts_domain_ = get_parameter("tts_domain").as_string();
  stop_text_ = get_parameter("stop_text").as_string();
  tts_priority_level_ = get_parameter("tts_priority_level").as_int();
  tts_priority_weight_ = get_parameter("tts_priority_weight").as_int();
  // 参数合法性校验：非法档位/加权回退默认（L7+0），避免静默传给 SDK 被仲裁丢弃
  if (!valid_priority_level(tts_priority_level_)) {
    RCLCPP_WARN(
      get_logger(), "tts_priority_level=%d 不是 SDK 合法档位（1/2/4/6/7/8/10），"
      "回退默认 7 (SYSTEM_L7)", tts_priority_level_);
    tts_priority_level_ = 7;
  }
  if (!valid_priority_weight(tts_priority_weight_)) {
    RCLCPP_WARN(
      get_logger(), "tts_priority_weight=%d 越界（0-99），回退默认 0",
      tts_priority_weight_);
    tts_priority_weight_ = 0;
  }
  tts_retry_seconds_ = std::max(1, static_cast<int>(
    get_parameter("tts_retry_seconds").as_int()));
  enroll_timeout_seconds_ = std::max(1, static_cast<int>(
    get_parameter("enroll_timeout_seconds").as_int()));
  keep_alive_seconds_ = get_parameter("keep_alive_seconds").as_int();
  connect_timeout_seconds_ = get_parameter("connect_timeout_seconds").as_int();
  heartbeat_interval_seconds_ = get_parameter("heartbeat_interval_seconds").as_int();
  tts_request_timeout_seconds_ = get_parameter("tts_request_timeout_seconds").as_int();
  tts_wait_timeout_seconds_ = get_parameter("tts_wait_timeout_seconds").as_int();
  dedupe_capacity_ = get_parameter("dedupe_capacity").as_int();

  nav_executor_name_ = get_parameter("nav_executor").as_string();
  mock_nav_seconds_ = std::max(0, static_cast<int>(
    get_parameter("mock_nav_seconds").as_int()));
  mock_relocate_hold_polls_ = std::max(0, static_cast<int>(
    get_parameter("mock_relocate_hold_polls").as_int()));
  relocate_timeout_seconds_ = std::max(1, static_cast<int>(
    get_parameter("relocate_timeout_seconds").as_int()));
  // 负值按 0 处理（原行为：落位即发）
  relocate_after_transform_delay_seconds_ = std::max(0, static_cast<int>(
    get_parameter("relocate_after_transform_delay_seconds").as_int()));
  nav_timeout_seconds_ = std::max(30, static_cast<int>(
    get_parameter("nav_timeout_seconds").as_int()));
  busy_hint_relocate_text_ = get_parameter("busy_hint_relocate_text").as_string();
  busy_hint_navigate_text_ = get_parameter("busy_hint_navigate_text").as_string();
  navigate_failed_text_ = get_parameter("navigate_failed_text").as_string();
  navigate_waypoint_missing_text_ =
    get_parameter("navigate_waypoint_missing_text").as_string();
  navigate_transform_failed_text_ =
    get_parameter("navigate_transform_failed_text").as_string();
  relocate_failed_text_ = get_parameter("relocate_failed_text").as_string();
  navigate_lead_text_ = get_parameter("navigate_lead_text").as_string();
  navigate_lead_welcome_text_ =
    get_parameter("navigate_lead_welcome_text").as_string();
  navigate_depart_text_ = get_parameter("navigate_depart_text").as_string();
  relocate_depart_text_ = get_parameter("relocate_depart_text").as_string();
  {
    // 先拷贝再迭代：as_string_array() 返回 Parameter 内部引用，直接
    // range-for 临时 Parameter 会悬空（与 motion_preset_ids 同款真机
    // 踩坑注释）；空串条目滤掉。两池同法加载
    const auto load_pool = [this](const char * name, std::vector<std::string> & pool) {
        const auto texts = get_parameter(name).as_string_array();
        for (const auto & text : texts) {
          if (!text.empty()) {
            pool.push_back(text);
          }
        }
      };
    load_pool("navigate_chatter_texts", navigate_chatter_texts_);
    load_pool("navigate_chatter_welcome_texts", navigate_chatter_welcome_texts_);
    navigate_chatter_first_text_ =
      get_parameter("navigate_chatter_first_text").as_string();
    navigate_chatter_welcome_first_text_ =
      get_parameter("navigate_chatter_welcome_first_text").as_string();
  }
  // 静默钳 1s（仅防 0/负值：chatter 走 speak_hint 同档排队不强抢，
  // 估算偏短时新句排队接播、不叠音，间隔完全由配置定）
  navigate_chatter_silence_seconds_ = std::max(
    1, static_cast<int>(
    get_parameter("navigate_chatter_silence_seconds").as_int()));
  tour_speak_interval_seconds_ = std::max(1, static_cast<int>(
    get_parameter("tour_speak_interval_seconds").as_int()));
  point_map_file_ = get_parameter("point_map_file").as_string();
  point_map_missing_text_ = get_parameter("point_map_missing_text").as_string();
  point_map_unmatched_text_ =
    get_parameter("point_map_unmatched_text").as_string();
  welcome_point_code_ = get_parameter("welcome_point_code").as_string();
  welcome_lead_text_ = get_parameter("welcome_lead_text").as_string();
  welcome_prep_done_text_ = get_parameter("welcome_prep_done_text").as_string();
  send_off_done_text_ = get_parameter("send_off_done_text").as_string();
  send_off_point_code_ = get_parameter("send_off_point_code").as_string();
  send_off_wave_motion_id_ = static_cast<int32_t>(
    get_parameter("send_off_wave_motion_id").as_int());
  broadcast_point_code_ = get_parameter("broadcast_point_code").as_string();
  // 防御：W 与 Z 配成同一 code 时分流歧义（Z 分支在前先命中，W 永远
  // 走不到）——响亮告警提醒配置错误，不静默吞掉
  if (!broadcast_point_code_.empty() && broadcast_point_code_ == send_off_point_code_) {
    RCLCPP_WARN(
      get_logger(),
      "broadcast_point_code 与 send_off_point_code 相同（%s）：原地播报分流"
      "不可达（Z 送客分流在前先命中），请修改配置", broadcast_point_code_.c_str());
  }
  // 负值按 0 处理（原行为：到达即变形）
  welcome_prep_transform_delay_seconds_ = std::max(0, static_cast<int>(
    get_parameter("welcome_prep_transform_delay_seconds").as_int()));
  welcome_flow_timeout_seconds_ = std::max(1, static_cast<int>(
    get_parameter("welcome_flow_timeout_seconds").as_int()));

  // ---- MQTT 迎宾三场景路由表（2026-10-22 现场需求）----
  // 复用语料触发同款配置（语音/MQTT 双触发面一套指令词，改 yaml 一处
  // 即双面生效）：welcome_prep/send_off 条目格式「点位名|指令」，welcome
  // 条目即指令本身。解析校验与 main.cpp 注入 AgentMonitor 同款（非法
  // 条目告警跳过，不影响另一触发面）。先拷贝再迭代（as_string_array
  // 返回 Parameter 内部引用，range-for 临时 Parameter 会悬空，同款
  // 真机踩坑注释）
  {
    const auto load_routes =
      [this](const char * name, std::vector<WelcomeSceneRoute> & routes) {
        const auto items = get_parameter(name).as_string_array();
        for (const auto & item : items) {
          const auto sep = item.find('|');
          if (sep == std::string::npos || sep == 0 || sep + 1 >= item.size()) {
            RCLCPP_WARN(
              get_logger(),
              "%s 格式非法（应为 点位名|指令），MQTT 迎宾路由跳过该条: 「%s」",
              name, item.c_str());
            continue;
          }
          WelcomeSceneRoute route;
          route.point = item.substr(0, sep);
          route.phrase = item.substr(sep + 1);
          if (normalize_phrase_text(route.phrase).empty()) {
            RCLCPP_WARN(
              get_logger(),
              "%s 指令部分归一化后为空（纯标点/空白），MQTT 迎宾路由跳过该条: 「%s」",
              name, item.c_str());
            continue;
          }
          routes.push_back(std::move(route));
        }
      };
    load_routes("agent_monitor.welcome_prep_phrases", welcome_prep_routes_);
    load_routes("agent_monitor.send_off_phrases", send_off_routes_);
    // ---- 内置短标记词（2026-10-22 联调实测：后端 scene=welcome 下发的
    // texts 是三选一标记词「准备迎宾」「开始迎宾」「欢送顾客」，不携带
    // agent 完整话术——日志实测 texts=["开始迎宾"]）——追加进三张路由
    // 表，route_welcome_scene 命中后 texts 只用于路由不作播报（与 yaml
    // 长指令双形态兼容，归一化子串匹配互不冲突：标记词间互不为子串，
    // agent 话术「开始迎接我的顾客」也不含「开始迎宾」整词）。准备
    // 迎宾/欢送顾客的点位取迎宾点：welcome_point_code（精确 code）优先，
    // 空则名称关键词「迎宾」（handle_welcome_prep 内两级解析 + 关键词
    // 兜底，与语料触发同口径，App 换迎宾点改映射表即生效） ----
    const std::string marker_point =
      welcome_point_code_.empty() ? "迎宾" : welcome_point_code_;
    welcome_prep_routes_.push_back({marker_point, "准备迎宾"});
    send_off_routes_.push_back({marker_point, "欢送顾客"});
    welcome_scene_phrases_.push_back("开始迎宾");
    const auto welcome_phrases =
      get_parameter("agent_monitor.welcome_phrases").as_string_array();
    for (const auto & phrase : welcome_phrases) {
      if (normalize_phrase_text(phrase).empty()) {
        continue;  // 空指令（纯标点）不作为有效路由词（同语料触发口径）
      }
      welcome_scene_phrases_.push_back(phrase);
    }
    navigate_phrase_template_ =
      get_parameter("agent_monitor.navigate_phrase_template").as_string();
    const auto navigate_phrases =
      get_parameter("agent_monitor.navigate_phrases").as_string_array();
    for (const auto & item : navigate_phrases) {
      const auto sep = item.find('|');
      const auto phrase =
        (sep == std::string::npos) ? item : item.substr(sep + 1);
      if (normalize_phrase_text(phrase).empty()) {
        continue;
      }
      navigate_route_phrases_.push_back(phrase);
    }
  }

  // robot_id/凭据来自 enroll 响应，构造时未知；主题在 apply_credential 后生成
  boot_id_ = generate_uuid_v7();
  session_id_ = generate_uuid_v7();
  started_at_ = std::chrono::steady_clock::now();
}

void MqttGuideBridgeNode::build_topics()
{
  // ---- 主题规划（契约：下行 down/commands，上行回执 up/commands）----
  down_topic_ = topic_for(topic_prefix_, robot_id_, "down/commands");
  up_commands_topic_ = topic_for(topic_prefix_, robot_id_, "up/commands");
  heartbeat_topic_ = topic_for(topic_prefix_, robot_id_, "heartbeat");
  presence_topic_ = topic_for(topic_prefix_, robot_id_, "presence");
}

bool MqttGuideBridgeNode::apply_credential(const RobotCredential & credential)
{
  // 协议：同 SN 恒返回同一 robot_id，只轮换密码——正常路径仅更新密码。
  // 防御性处理：若 robot_id 变化（Server 侧重建注册表等），重建客户端与主题。
  if (credential.robot_id != robot_id_) {
    if (!robot_id_.empty()) {
      RCLCPP_WARN(
        get_logger(), "enroll 返回的 robot_id 发生变化: %s -> %s（重建 MQTT 客户端与主题）",
        robot_id_.c_str(), credential.robot_id.c_str());
      if (client_) {
        try {
          client_->disconnect()->wait_for(std::chrono::seconds(2));
        } catch (const mqtt::exception &) {
          // 未连接或已断开：忽略
        }
        client_.reset();
      }
      connected_.store(false);
      subscribed_.store(false);
    } else {
      RCLCPP_INFO(
        get_logger(), "enroll 分配 robot_id=%s（SN=%s，同 SN 稳定不变）",
        credential.robot_id.c_str(), serial_number_.c_str());
    }
    robot_id_ = credential.robot_id;
    build_topics();
  }

  username_ = credential.username;
  password_ = credential.password;

  if (!client_) {
    const auto client_id = "guide-pulse-" + robot_id_;
    client_ = std::make_unique<mqtt::async_client>(broker_uri_, client_id);
    client_->set_callback(*this);
    RCLCPP_INFO(
      get_logger(), "MQTT 客户端就绪: %s (client_id=%s)", broker_uri_.c_str(), client_id.c_str());
  }
  return true;
}

mqtt::connect_options MqttGuideBridgeNode::build_connect_options()
{
  // 遗嘱：异常掉线时由 broker 代发 offline presence（retained，契约格式）。
  // 每次连接前重建：遗嘱与 online presence 须携带同一个（连接前新生成的）
  // session_id，契约要求 session_id 每次网络连接前重新生成。
  const json will_payload = {
    {"spec_version", "1.0"},
    {"robot_id", robot_id_},
    {"session_id", session_id_},
    {"boot_id", boot_id_},
    {"state", "offline"},
    {"reason", "connection_lost"},
    {"occurred_at", nullptr},  // 契约：connection_lost 时为 null（broker 代发无时钟）
  };
  mqtt::properties json_props{
    mqtt::property(mqtt::property::PAYLOAD_FORMAT_INDICATOR, 1),
    mqtt::property(mqtt::property::CONTENT_TYPE, "application/json"),
  };
  auto will = mqtt::make_message(presence_topic_, will_payload.dump(), 1, true, json_props);

  auto builder = mqtt::connect_options_builder()
    .mqtt_version(MQTTVERSION_5)
    .clean_start(true)
    .keep_alive_interval(std::chrono::seconds(keep_alive_seconds_))
    .connect_timeout(std::chrono::seconds(connect_timeout_seconds_))
    .will(*will);
  // 凭据来自当次 enroll（username 由 Server 发放，恒非空）
  builder.user_name(username_);
  builder.password(password_);
  return builder.finalize();
}

bool MqttGuideBridgeNode::start()
{
  if (running_.load()) {
    return true;
  }

  // ---- enroll 注册面配置校验（协议要求：凭 SN 换凭据，别无旁路）----
  if (enroll_origin_.empty() || serial_number_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "缺少 enroll 配置：enroll_origin='%s' serial_number='%s'（请配置 Server "
      "HTTP 注册面地址与本机序列号，SN 需运维预先登记）",
      enroll_origin_.c_str(), serial_number_.c_str());
    return false;
  }
  if (!valid_serial_number(serial_number_)) {
    RCLCPP_ERROR(
      get_logger(),
      "serial_number='%s' 格式非法（需 6..64 位，首字符字母数字，其后仅限 "
      "[A-Za-z0-9._:-]）。获取顺序：serial_number 参数 > 环境变量 %s > "
      "serial_number_file 文件", serial_number_.c_str(),
      get_parameter("serial_number_env").as_string().c_str());
    return false;
  }
  enroller_ = std::make_unique<RobotEnroller>(enroll_origin_, enroll_timeout_seconds_);

  // SDK TTS 客户端（PlayTts 服务来自机器人本体固件）
  tts_ = std::make_unique<TtsClient>(
    shared_from_this(), tts_service_name_, tts_domain_,
    tts_request_timeout_seconds_, stop_text_);

  // ---- 屏幕二维码显示（迎宾场景，v0.22.0）----
  // PlayEmotion 服务由运控板执行：MP4 须先 scp 推送（remote_* 配置）。
  // 空密码 = 禁用显示（远程推送不可用，迎宾 TTS 照常）
  ScreenDisplay::Config screen_cfg;
  screen_cfg.emotion_service_name =
    get_parameter("display_emotion_service_name").as_string();
  screen_cfg.remote_host = get_parameter("display_remote_host").as_string();
  screen_cfg.remote_user = get_parameter("display_remote_user").as_string();
  screen_cfg.remote_pass = get_parameter("display_remote_pass").as_string();
  screen_cfg.remote_dir = get_parameter("display_remote_dir").as_string();
  screen_cfg.request_timeout_seconds =
    get_parameter("display_request_timeout_seconds").as_int();
  screen_cfg.show_timeout_seconds =
    get_parameter("display_show_timeout_seconds").as_int();
  screen_cfg.restore_emotion_id =
    get_parameter("display_restore_emotion_id").as_int();
  screen_cfg.restore_priority =
    get_parameter("display_restore_priority").as_int();
  if (screen_cfg.remote_pass.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "display_remote_pass 为空：迎宾二维码显示禁用（TTS 播报不受影响）");
  }
  screen_ = std::make_unique<ScreenDisplay>(shared_from_this(), screen_cfg);

  // 二维码业务互斥窗口（show_qr/cancel_qr 各占此窗口，与其他业务互斥）
  qr_business_timeout_seconds_ =
    static_cast<int>(get_parameter("qr_business_timeout_seconds").as_int());

  // ---- 播报期间麦克风硬抑制（2026-10-16，SSH pactl，目标复用屏幕配置）----
  mic_mute_enabled_ = get_parameter("mic_mute_enabled").as_bool();
  mic_mute_target_ = screen_cfg.remote_user + "@" + screen_cfg.remote_host;
  mic_mute_pass_ = screen_cfg.remote_pass;
  if (mic_mute_enabled_ &&
    (mic_mute_pass_.empty() || mic_mute_pass_.find('\'') != std::string::npos))
  {
    mic_mute_enabled_ = false;
    RCLCPP_WARN(
      get_logger(), "mic_mute 关闭：display_remote_pass 为空或含单引号（无法 SSH）");
  }
  if (mic_mute_enabled_) {
    RCLCPP_INFO(
      get_logger(), "播报麦克风硬抑制已启用: ssh %s pactl set-source-mute",
      mic_mute_target_.c_str());
  }

  // ---- 迎宾二维码缓存/下载（v0.22.0 P-036）----
  // 文件面与 enroll 注册面同源（RobotCar Server 单服务多路由）；
  // 下载凭据复用当次 enroll 的 MQTT 密码（HTTP Basic，username=robot_id）
  QrCodeCache::Config qr_cfg;
  qr_cfg.origin = enroll_origin_;
  qr_cfg.cache_dir = get_parameter("qr_cache_dir").as_string();
  qr_cfg.timeout_seconds =
    static_cast<int>(get_parameter("qr_download_timeout_seconds").as_int());
  qr_cache_ = std::make_unique<QrCodeCache>(qr_cfg);

  // 默认二维码（未携带 qr_sha256 的迎宾用）：未配置时回落随包安装的
  // config/QRCode.png；文件缺失时禁用并告警
  default_qr_path_ = get_parameter("display_default_qr_path").as_string();
  if (default_qr_path_.empty()) {
    try {
      default_qr_path_ =
        ament_index_cpp::get_package_share_directory("guide_pulse") + "/config/QRCode.png";
    } catch (const std::exception &) {
      default_qr_path_.clear();  // ament 索引不可用：默认禁用
    }
  }
  if (!default_qr_path_.empty() && !std::filesystem::exists(default_qr_path_)) {
    RCLCPP_WARN(
      get_logger(), "默认二维码图片不存在(不显示): %s", default_qr_path_.c_str());
    default_qr_path_.clear();
  }

  // 唤醒抑制客户端（独立回调组，worker 线程 wait_for 等回执）
  suppress_service_name_ = get_parameter("suppress_service_name").as_string();
  suppress_wakeup_ = get_parameter("suppress_wakeup").as_bool();
  suppress_ttl_ms_ = static_cast<int>(get_parameter("suppress_ttl_ms").as_int());
  suppress_renew_interval_ms_ =
    static_cast<int>(get_parameter("suppress_renew_interval_ms").as_int());
  tts_speech_ms_per_char_ =
    static_cast<int>(get_parameter("tts_speech_ms_per_char").as_int());
  suppress_tail_lead_ms_ =
    static_cast<int>(get_parameter("suppress_tail_lead_ms").as_int());
  suppress_tail_scale_ = get_parameter("suppress_tail_scale").as_double();
  suppress_tail_max_ms_ =
    static_cast<int>(get_parameter("suppress_tail_max_ms").as_int());
  suppress_tail_flush_ms_ =
    static_cast<int>(get_parameter("suppress_tail_flush_ms").as_int());
  // 防御：TTL 必须大于续期间隔（否则两次续期间存在恢复窗口）
  if (suppress_renew_interval_ms_ >= suppress_ttl_ms_) {
    suppress_renew_interval_ms_ = suppress_ttl_ms_ - 1000;
  }
  if (suppress_wakeup_) {
    suppress_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    suppress_client_ = create_client<aimdk_msgs::srv::SetAgentSuppression>(
      suppress_service_name_, rmw_qos_profile_services_default, suppress_callback_group_);
  }

  // ---- 迎宾变形（scene=welcome 时确保人形形态）----
  // 直调厂家运控服务（本体固件提供），与 t1_slam_nav_ws 是否在跑无关；
  // 服务不在也允许启动——联调环境没有运控时迎宾只多一条 WARN，播报不受影响
  McTransform::Config transform_cfg;
  transform_cfg.enabled = get_parameter("welcome_transform").as_bool();
  transform_cfg.stand_from_down = get_parameter("welcome_stand_from_down").as_bool();
  transform_cfg.get_action_service = get_parameter("get_action_service").as_string();
  transform_cfg.set_action_service = get_parameter("set_action_service").as_string();
  mc_source_name_ = get_parameter("mc_source_name").as_string();
  transform_cfg.source_name = mc_source_name_;
  transform_cfg.wait_timeout_seconds = std::max(5, static_cast<int>(
    get_parameter("transform_wait_timeout_seconds").as_int()));
  // 摄像头舵机（变形完成后竖起摄像头，人脸检测可用；四足方向收平，
  // 2026-10-18 迎宾准备模式：摄像头收进去）
  transform_cfg.camera_servo_enabled = get_parameter("welcome_camera_servo").as_bool();
  transform_cfg.camera_position = static_cast<int>(
    get_parameter("welcome_camera_position").as_int());
  transform_cfg.camera_retract_position = static_cast<int>(
    get_parameter("welcome_prep_camera_position").as_int());
  transform_cfg.camera_speed = static_cast<int>(
    get_parameter("welcome_camera_speed").as_int());
  transform_cfg.servo_service = get_parameter("set_servo_service").as_string();
  // 变形音（SSH 运控板 paplay，失败只记日志；凭据复用 display_remote_*，
  // 与麦克风静音/屏幕推送同源——密码为空时 McTransform 构造中自行禁用）
  transform_cfg.sound_enabled = get_parameter("welcome_transform_sound").as_bool();
  transform_cfg.sound_path = get_parameter("transform_sound_path").as_string();
  transform_cfg.down_sound_path =
    get_parameter("transform_down_sound_path").as_string();
  transform_cfg.ssh_target = get_parameter("display_remote_user").as_string() + "@" +
    get_parameter("display_remote_host").as_string();
  transform_cfg.ssh_pass = get_parameter("display_remote_pass").as_string();
  if (mc_source_name_.empty()) {
    // 空 source 会被运控以 Invalid source 当场拒掉（state=-10002），而表象
    // 只是「等不到位超时」——这儿响亮拦住，不带注定被拒的请求跑下去
    RCLCPP_WARN(
      get_logger(), "mc_source_name 为空：迎宾变形禁用（运控会拒空 source 请求）");
    transform_cfg.enabled = false;
  }
  transform_ = std::make_unique<McTransform>(shared_from_this(), transform_cfg);
  if (transform_cfg.enabled) {
    RCLCPP_INFO(
      get_logger(),
      "迎宾变形已启用: 直调运控 %s / %s（已是人形则不动；趴姿%s扶起）",
      transform_cfg.set_action_service.c_str(),
      transform_cfg.get_action_service.c_str(),
      transform_cfg.stand_from_down ? "默认" : "不");
  } else {
    RCLCPP_INFO(get_logger(), "迎宾变形已关闭（welcome_transform=false）");
  }

  // ---- 播报伴随动作 ----
  // 独立于迎宾变形开关（动作在当前形态下直接执行；变形流程本身也会
  // 保证人形）。服务不在也允许启动——动作失败仅 WARN，播报不受影响
  motion_enabled_ = get_parameter("motion_enabled").as_bool();
  {
    // 先拷贝再迭代：as_integer_array() 返回 Parameter 内部引用，
    // 直接 range-for 临时 Parameter 会悬空（真机踩坑：读到垃圾值）
    const auto motion_ids = get_parameter("motion_preset_ids").as_integer_array();
    for (const auto id : motion_ids) {
      if (id > 0) {
        motion_preset_ids_.push_back(static_cast<int32_t>(id));
      }
    }
  }
  motion_request_timeout_seconds_ = std::max(1, static_cast<int>(
    get_parameter("motion_request_timeout_seconds").as_int()));
  motion_repeat_seconds_ = std::max(0, static_cast<int>(
    get_parameter("motion_repeat_seconds").as_int()));
  if (motion_enabled_ && !motion_preset_ids_.empty()) {
    motion_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    motion_client_ = create_client<aimdk_msgs::srv::SetMcPresetMotion>(
      get_parameter("motion_service_name").as_string(),
      rmw_qos_profile_services_default, motion_callback_group_);
    RCLCPP_INFO(
      get_logger(), "播报伴随动作已启用: %zu 个动作轮换（首个=%d）",
      motion_preset_ids_.size(), motion_preset_ids_.front());
  } else {
    RCLCPP_INFO(
      get_logger(), "播报伴随动作已关闭（motion_enabled=false 或动作池为空）");
  }

  // 导航执行器：mock（联调/测试）或 waypoint（真机，对接 t1_slam_nav_ws
  // 的 /liwei_navigation/navigate action，docs/design.md §4）
  if (nav_executor_name_ == "waypoint") {
    // poll_timeout 传入 nav_timeout_seconds（可配置兜底，默认 600s）：
    // goal 受理后 result 断流时按超时收尾，导览业务不无限占用
    nav_ = std::make_unique<WaypointNavExecutor>(
      shared_from_this(), "/liwei_navigation/navigate",
      "/liwei_navigation/query", std::chrono::seconds(nav_timeout_seconds_));
    RCLCPP_INFO(
      get_logger(), "导航执行器: waypoint（/liwei_navigation/navigate action，"
      "point_name 直接映射航点名，兜底超时 %ds）", nav_timeout_seconds_);
  } else {
    if (nav_executor_name_ != "mock") {
      RCLCPP_WARN(
        get_logger(), "未知 nav_executor='%s'，回退 mock", nav_executor_name_.c_str());
      nav_executor_name_ = "mock";
    }
    nav_ = std::make_unique<MockNavExecutor>(mock_nav_seconds_, mock_relocate_hold_polls_);
  }
  tour_ = std::make_unique<TourRunner>(*nav_);

  // ---- 点位映射表（v0.25.0 P-039）----
  // 启动即加载持久化表（App 是唯一权威来源，上次下发的表跨重启仍有效，
  // 无需等 App 重推）；文件缺失=空表正常等推，损坏=告警等 App 重推覆盖
  point_map_ = std::make_unique<PointMapStore>(point_map_file_);
  if (point_map_->load()) {
    RCLCPP_INFO(
      get_logger(), "点位映射表已加载: %zu 条（文件=%s）",
      point_map_->size(),
      point_map_file_.empty() ? "纯内存模式" : point_map_file_.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(), "点位映射表文件损坏，按空表启动（等 App 重推覆盖）: %s",
      point_map_file_.c_str());
  }
  // 表生效通知（加载成功含空表、损坏按空表均通知）：main.cpp 依此为
  // AgentMonitor 重建动态导航语料规则——持久化表跨重启保留，启动即
  // 生成语料，无需等 App 重推（回调未设置 no-op，联调用）
  if (on_point_map_updated) {
    on_point_map_updated(point_map_->points());
  }

  running_.store(true);

  // ---- 连接管理线程：enroll 换凭据 -> MQTT 连接 -> 断线/被拒重试 ----
  // 不启用 paho automatic_reconnect：凭据轮换语义（CONNACK 0x86/0x87 后
  // 必须先重新 enroll）需要应用层统一驱动
  RCLCPP_INFO(
    get_logger(), "enroll 注册面: %s serial_number=%s broker=%s",
    enroll_origin_.c_str(), serial_number_.c_str(), broker_uri_.c_str());
  connect_thread_ = std::thread([this]() {connect_loop();});

  worker_thread_ = std::thread([this]() {worker_loop();});

  // 连接维护：回调线程不能碰 paho API，订阅/上线通告统一在此定时器完成
  maintenance_timer_ = create_wall_timer(
    std::chrono::seconds(1), [this]() {maintain_connection();});

  if (heartbeat_interval_seconds_ > 0) {
    heartbeat_timer_ = create_wall_timer(
      std::chrono::seconds(heartbeat_interval_seconds_), [this]() {publish_heartbeat();});
  }

  // 按需阻塞等待 TTS 服务（真机上固件通常已就绪；联调时可设 0 跳过）
  if (tts_wait_timeout_seconds_ > 0) {
    RCLCPP_INFO(
      get_logger(), "等待 SDK TTS 服务: %s (最长 %ds)", tts_service_name_.c_str(),
      tts_wait_timeout_seconds_);
    if (!tts_->wait_for_service(std::chrono::seconds(tts_wait_timeout_seconds_))) {
      RCLCPP_WARN(
        get_logger(), "TTS 服务暂不可用，将在收到指令时按需重试");
    }
  }

  RCLCPP_INFO(get_logger(), "guide_bridge 已启动，等待 enroll 分配 robot_id 后接入 MQTT ...");
  return true;
}

// ---------------------------------------------------------------------------
// 连接管理线程：enroll（按需）-> MQTT 连接 -> 失败按原因分派重试。
// 所有阻塞调用（HTTP enroll、connect 等待）都发生在本线程，不拖累 executor。
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::connect_loop()
{
  const auto auth_rejected = [](int reason_code) {
      // 0x86 Bad user name or password（密码被轮换/过期）
      // 0x87 Not authorized（ACL/账号被删）
      return reason_code == 0x86 || reason_code == 0x87;
    };
  // 可中断睡眠：shutdown 置 running_=false 后最多 250ms 内醒来退出
  const auto interruptible_sleep = [this](int seconds) {
      for (int slept = 0; slept < seconds * 4 && running_.load(); ++slept) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    };
  auto backoff_seconds = 1;

  while (running_.load()) {
    if (connected_.load()) {
      backoff_seconds = 1;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    // ---- 阶段 1：需要时重新 enroll（首次启动 / 凭据被拒 / 密码到期）----
    if (!credentials_ready_ || need_enroll_) {
      if (enroll_halted_) {
        // 协议：robot_revoked 必须停止重试并上报运维；节点保持存活等处理
        interruptible_sleep(5);
        continue;
      }
      const auto result = enroller_->enroll(serial_number_);
      if (result.error == EnrollError::NONE) {
        apply_credential(result.credential);
        credentials_ready_ = true;
        need_enroll_ = false;
        // 注意：这里不重置退避——退避只在连接成功后复位。否则凭据持续被拒
        //（如 broker 侧配置错误）时会以 ~1Hz 空转打注册面
        // 密码绝不进日志（协议：凭据只在响应中出现一次、不落盘）
        RCLCPP_INFO(
          get_logger(), "enroll 成功: robot_id=%s username=%s expires_at=%s",
          result.credential.robot_id.c_str(), result.credential.username.c_str(),
          result.credential.expires_at.c_str());
      } else {
        switch (result.error) {
          case EnrollError::REVOKED:
            RCLCPP_ERROR(
              get_logger(),
              "enroll 被拒: 本机 SN 已被吊销（robot_revoked）。停止重试，"
              "请运维处理（RobotCar: scripts/robot-admin.sh show/reactivate %s）: %s",
              serial_number_.c_str(), result.detail.c_str());
            enroll_halted_ = true;
            break;
          case EnrollError::RATE_LIMITED:
            RCLCPP_WARN(
              get_logger(), "enroll 触发限速（rate_limited），%ds 后退避重试: %s",
              backoff_seconds, result.detail.c_str());
            break;
          case EnrollError::REJECTED:
            RCLCPP_WARN(
              get_logger(),
              "enroll 被拒: SN 未预登记（enroll_rejected），%ds 后重试"
              "（可让运维执行 scripts/robot-admin.sh register %s guide_pulse）: %s",
              backoff_seconds, serial_number_.c_str(), result.detail.c_str());
            break;
          case EnrollError::INVALID_REQUEST:
            RCLCPP_ERROR(
              get_logger(), "enroll 请求参数非法（检查 serial_number 配置）: %s",
              result.detail.c_str());
            break;
          default:
            RCLCPP_WARN(
              get_logger(), "enroll 失败（%ds 后重试）: %s",
              backoff_seconds, result.detail.c_str());
            break;
        }
        interruptible_sleep(backoff_seconds);
        backoff_seconds = std::min(backoff_seconds * 2, 60);
        continue;
      }
    }

    if (!client_) {
      // 理论不可达（凭据就绪即建客户端）；防御性等待
      interruptible_sleep(1);
      continue;
    }

    // ---- 阶段 2：连接 MQTT（每次连接前重生成 session_id，契约要求）----
    session_id_ = generate_uuid_v7();
    heartbeat_sequence_ = 0;
    try {
      mqtt::token_ptr connect_token = client_->connect(build_connect_options());
      if (!connect_token->wait_for(std::chrono::seconds(2 * connect_timeout_seconds_))) {
        // 连接仍可能在途完成（connected() 回调置位）；复位并下轮重试
        RCLCPP_WARN(
          get_logger(), "MQTT 连接超时（%ds），%ds 后重试", connect_timeout_seconds_,
          backoff_seconds);
        try {
          client_->disconnect()->wait_for(std::chrono::seconds(1));
        } catch (const mqtt::exception &) {
          // 尚未连上：disconnect 抛错属预期，忽略
        }
      } else {
        connected_.store(true);  // 与 connected() 回调双保险
        backoff_seconds = 1;
      }
    } catch (const mqtt::exception & e) {
      const int reason = e.get_reason_code();
      if (auth_rejected(reason)) {
        // 协议：凭据被拒（轮换/过期/吊销）时重新 enroll 再连
        need_enroll_ = true;
        credentials_ready_ = false;
        RCLCPP_WARN(
          get_logger(), "MQTT CONNECT 被拒（reason=0x%02X %s），重新 enroll 获取凭据",
          reason, e.get_reason_code_str().c_str());
      } else {
        RCLCPP_WARN(
          get_logger(), "MQTT 连接失败（%ds 后重试）: %s",
          backoff_seconds, e.what());
      }
    }

    interruptible_sleep(backoff_seconds);
    backoff_seconds = std::min(backoff_seconds * 2, 30);
  }
}

void MqttGuideBridgeNode::shutdown() noexcept
{
  if (!running_.exchange(false)) {
    return;
  }
  RCLCPP_INFO(get_logger(), "正在停止 guide_bridge ...");

  heartbeat_timer_.reset();
  maintenance_timer_.reset();
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  // 麦克风硬抑制关停兜底：worker 已 join 无并发；静音未恢复则恢复
  //（防进程退出后机器人失聪）
  set_mic_muted(false);
  // 变形线程在 worker 之后回收：worker（handle_scene 发起变形）已收尾，
  // 变形线程的等待循环每 500ms 检查关停（rclcpp::ok()，Ctrl-C 已触发），
  // 约半秒内退出
  if (transform_) {
    transform_->shutdown();
  }
  // 连接线程可能阻塞在 enroll HTTP / connect 等待（均有超时上限），可安全 join
  if (connect_thread_.joinable()) {
    connect_thread_.join();
  }
  if (tour_) {
    tour_->cancel();  // 丢弃在途导航回调
  }

  if (client_ && client_->is_connected()) {
    publish_presence("offline", "graceful_shutdown");  // 契约枚举值
    try {
      client_->disconnect()->wait_for(std::chrono::seconds(3));
    } catch (const mqtt::exception & e) {
      RCLCPP_WARN(get_logger(), "MQTT 断开异常: %s", e.what());
    }
  }
  client_.reset();
  enroller_.reset();
  tts_.reset();
  nav_.reset();
  tour_.reset();
}

// ---------------------------------------------------------------------------
// MQTT 回调
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::connected(const std::string & cause)
{
  // 注意：paho 在持有全局互斥锁的线程上回调本函数，此处严禁调用任何
  // paho API（publish/subscribe/is_connected），否则非递归锁重入直接死锁。
  // 订阅与 online 通告交给 maintain_connection() 定时器处理。
  connected_.store(true);
  RCLCPP_INFO(get_logger(), "MQTT 已连接: %s", cause.c_str());
}

void MqttGuideBridgeNode::connection_lost(const std::string & cause)
{
  connected_.store(false);
  subscribed_.store(false);  // 重连后由维护定时器补订阅
  // 若断开源于密码到期/轮换（Server 侧作废旧密码），连接线程重连被拒
  //（0x86）后会自动重新 enroll；这里无需额外处理
  RCLCPP_WARN(get_logger(), "MQTT 连接断开: %s（连接线程将自动重连）", cause.c_str());
}

// 维护定时器（executor 线程，paho 回调之外）：完成（重）连接后的
// 订阅恢复与 online presence 通告，失败下个周期自动重试。
void MqttGuideBridgeNode::maintain_connection()
{
  if (!running_.load() || !connected_.load() || subscribed_.load()) {
    return;
  }
  try {
    client_->subscribe(down_topic_, 1)->wait_for(std::chrono::seconds(3));
    subscribed_.store(true);
    RCLCPP_INFO(get_logger(), "已订阅下行主题: %s", down_topic_.c_str());
    // session_id 已由连接线程在本次连接前重生成（遗嘱与通告同值）。
    // 契约 reason：首连 connected，重连 reconnected。
    publish_presence("online", ever_online_.load() ? "reconnected" : "connected");
    ever_online_.store(true);
  } catch (const mqtt::exception & e) {
    RCLCPP_WARN(get_logger(), "订阅下行主题失败（1s 后重试）: %s", e.what());
  }
}

void MqttGuideBridgeNode::message_arrived(mqtt::const_message_ptr message)
{
  const auto & topic = message->get_topic();
  if (topic != down_topic_) {
    return;  // 只处理本机下行主题
  }
  const auto payload = message->to_string();
  // 日志风格与 tts_demo / robot_com 网关一致（token 类字段打码）
  RCLCPP_INFO(
    get_logger(), "MQTT 收到 topic=%s payload=%s",
    topic.c_str(), log_safe_payload(payload).c_str());

  // paho 回调线程只入队，避免阻塞其 IO 线程
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(std::move(payload));
    if (queue_.size() > 256) {  // 有界队列保护内存
      queue_.pop_front();
      RCLCPP_WARN(get_logger(), "下行队列积压超限，丢弃最旧报文");
    }
  }
  queue_cv_.notify_one();
}

// ---------------------------------------------------------------------------
// 本地指令注入（语料触发等）：入队后由 worker 线程与 MQTT 下行同样
// 串行处理。AgentMonitor 工作线程调用，线程安全
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::inject_command(CommandEvent event)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    injected_commands_.push_back(std::move(event));
  }
  queue_cv_.notify_one();
}

// ---------------------------------------------------------------------------
// 工作线程：解析 command 信封 -> 回执/过期/幂等过滤 -> tts/tts.stop/scene/
// navigate/navigate.stop/relocate 分发；音频会话按 next_at 周期调度；导航到达事件经
// TourRunner 单槽消费（在途导航期间以 250ms 上限等待，保证及时开播讲解）；
// 重定位等待导航模块验收（在途以 250ms 上限轮询 set_pose goal 的 result，
// relocate_timeout_seconds 超时兜底）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::worker_loop()
{
  while (running_.load()) {
    std::string item;
    std::optional<CommandEvent> injected;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      // 计算定时唤醒时刻：取会话下一跳；二维码常驻超时；导航在途/到达
      // 待消费/重定位等待验收时再钳制 250ms 上限（NavExecutor 回调与本
      // 线程轮询无唤醒通道，周期消费）
      auto wake_at = std::chrono::steady_clock::time_point::max();
      if (session_) {
        wake_at = std::min(wake_at, session_next_at_);
      }
      if (motion_renew_until_.has_value()) {
        // 动作续播：下次续触与窗口到期两个时刻都要唤醒（先到先行动）
        wake_at = std::min(wake_at, motion_renew_next_at_);
        wake_at = std::min(wake_at, *motion_renew_until_);
      }
      if (screen_ && screen_->active()) {
        wake_at = std::min(wake_at, screen_->deadline());
      }
      if (suppress_renew_active_) {
        wake_at = std::min(wake_at, suppress_next_at_);
      }
      if ((tour_ && tour_->navigating()) || relocate_pending_) {
        wake_at = std::min(
          wake_at, std::chrono::steady_clock::now() + std::chrono::milliseconds(250));
      }
      if (wake_at != std::chrono::steady_clock::time_point::max()) {
        queue_cv_.wait_until(
          lock, wake_at, [this]() {
            return !running_.load() || !queue_.empty() || !injected_commands_.empty() ||
                   tts_early_stop_;
          });
      } else {
        queue_cv_.wait(
          lock, [this]() {
            return !running_.load() || !queue_.empty() || !injected_commands_.empty() ||
                   tts_early_stop_;
          });
      }
      if (!running_.load()) {
        break;
      }
      if (!injected_commands_.empty()) {
        // 本地注入优先消费：语料触发即刻进流程，不排在 MQTT 积压之后
        injected = std::move(injected_commands_.front());
        injected_commands_.pop_front();
      } else if (!queue_.empty()) {
        item = std::move(queue_.front());
        queue_.pop_front();
      }
    }

    if (injected.has_value()) {
      // 本地注入指令（语料触发）：operation_id 本地生成唯一，无幂等/
      // 过期面，跳过信封解析直接分发——与 MQTT 下行共用 handle_command
      //（迎宾互斥窗口等策略在分发内统一生效）。不打印注入行（语料
      // 命中行已含 action/point/话术，后续业务行衔接，2026-10-18 现场
      // 日志整理）
      handle_command(*injected);
      continue;
    }

    if (!item.empty()) {
      std::string error;
      // 契约 v0.6.0/v0.24.0：payload 非法但能解析出 operation_id 的本机
      // tts/navigate/scene 指令，按类型回 *.result(rejected, invalid_request)；
      // 解析不出则直接丢弃。tts.stop/navigate.stop/未知 scene 枚举值无回执
      std::string invalid_operation_id;
      std::string invalid_command_type;
      const auto event = parse_command_event(
        item, robot_id_, error, &invalid_operation_id, &invalid_command_type);
      if (!event) {
        if (error == "ignore") {
          // ota 等由其他组件处理，非本组件职责
          RCLCPP_DEBUG(
            get_logger(), "忽略非本组件指令 type=unknown（由其他组件处理）");
        } else if (!invalid_operation_id.empty()) {
          RCLCPP_ERROR(
            get_logger(),
            "下行指令校验失败: type=%s %s operation_id=%s，回执 rejected/invalid_request",
            invalid_command_type.c_str(), error.c_str(), invalid_operation_id.c_str());
          if (invalid_command_type == "tts") {
            send_tts_result(
              invalid_operation_id, TTS_RESULT_REJECTED, TTS_REASON_INVALID_REQUEST);
          } else {
            send_action_result(
              invalid_command_type + ".result", invalid_operation_id,
              ACTION_RESULT_REJECTED, TTS_REASON_INVALID_REQUEST);
          }
        } else {
          // 连 operation_id 都解析不出/信封非法：丢弃记日志
          RCLCPP_ERROR(
            get_logger(), "下行指令校验失败: %s payload=%.120s",
            error.c_str(), item.c_str());
        }
        continue;
      }
      if (event->expired) {
        // 契约：过期 tts 回 rejected(command_expired)；navigate/relocate/
        // scene/point_map 过期回各自 result 的 rejected(command_expired)
        //（v0.24.0 P-038 / v0.25.0 P-039 result 章节与状态机；到达/执行
        // 不再发生）；tts.stop/navigate.stop 无回执，过期直接丢弃
        if (event->type == "tts") {
          RCLCPP_WARN(
            get_logger(), "指令已过期: type=tts operation_id=%s，回执 rejected/command_expired",
            event->operation_id.c_str());
          send_tts_result(
            event->operation_id, TTS_RESULT_REJECTED, TTS_REASON_COMMAND_EXPIRED);
        } else if (event->type == "navigate" || event->type == "relocate" ||
          event->type == "scene" || event->type == "point_map")
        {
          RCLCPP_WARN(
            get_logger(), "指令已过期: type=%s operation_id=%s，回执 rejected/command_expired",
            event->type.c_str(), event->operation_id.c_str());
          send_action_result(
            event->type + ".result", event->operation_id,
            ACTION_RESULT_REJECTED, TTS_REASON_COMMAND_EXPIRED);
        } else {
          RCLCPP_WARN(
            get_logger(), "指令已过期丢弃: type=%s operation_id=%s",
            event->type.c_str(), event->operation_id.c_str());
        }
        continue;
      }
      {
        // 契约 v0.6.0：同一 operation_id 的指令重发必须重发同一回执
        //（同一 event_id），不得重新执行副作用。仅 worker 线程访问，免锁
        const auto it = receipt_cache_.find(event->operation_id);
        if (it != receipt_cache_.end()) {
          RCLCPP_WARN(
            get_logger(), "重复 operation_id=%s，重发同一回执（同 event_id）",
            event->operation_id.c_str());
          publish_json(up_commands_topic_, it->second, 1, false);
          continue;
        }
      }
      if (seen_recently(event->operation_id)) {
        RCLCPP_WARN(
          get_logger(),
          "重复 operation_id=%s，忽略（回执缓存已淘汰，不重播不重发）",
          event->operation_id.c_str());
        continue;
      }

      handle_command(*event);
    } else {
      // 无新报文：到点的定时任务（会话下一跳/二维码超时/唤醒抑制续期）
      // 与导航到达事件
      bool tts_early = false;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        tts_early = tts_early_stop_;
        tts_early_stop_ = false;
      }
      const auto now = std::chrono::steady_clock::now();
      if (tts_early && suppress_renew_active_ && !session_ && suppress_stop_at_.has_value()) {
        // 本会话 TtsStatus 终态（END/STOP/ERROR）已回流：语音真实结束，
        // 提前解除麦克风静音与唤醒抑制（估算时序仅兜底；多条会话中段的
        // 终态因 session_ 仍在不满足条件，正常走续期）
        suppress_renew_active_ = false;
        suppress_stop_at_.reset();
        // 语音真实结束：动作续播窗口一并关闭（手立刻停，不等估算到期）
        motion_renew_until_.reset();
        set_mic_muted(false);
        if (suppress_armed_) {
          suppress_wakeup(static_cast<std::uint32_t>(suppress_tail_flush_ms_));
        }
        suppress_armed_ = false;
        RCLCPP_INFO(
          get_logger(), "播报终态已回流（TtsStatus），提前解除麦克风静音与唤醒抑制");
        // 播报终态回流 = 播报类业务真实结束：业务占用即刻释放
        //（2026-10-16 真机教训：迎宾 ~30s 播完且终态已回流，但固定 120s
        // 互斥窗口仍占用，连续两次迎宾第二次被拒 failed/busy）。迎宾固定
        // 120s 与 tts/讲解字数估算窗口自即刻起降级为兜底超时，仅在
        // BEST_EFFORT 丢终态事件时兜底。守卫只放行播报类业务（白名单）：
        // 导览在途期无播报会话不经此分支（防「窗口到期后残留播报仍在 +
        // 新导航已受理」交叉误释放）；重定位等待期同样排除（kRelocate
        // 的占用由 relocate_pending_ 表达、终态由验收/超时给出——不占
        // 业务的残留播报回流不许清它，否则 business_ 标签错乱、busy
        // 语音提示静默）；二维码显示/取消业务（kShowQr/kCancelQr，2026-
        // 10-17）无播报会话、无终态回流面，只由 20s 窗口到期自然释放
        if (business_ == Business::kTts || business_ == Business::kWelcome ||
          (business_ == Business::kTour && !(tour_ && tour_->navigating())))
        {
          RCLCPP_INFO(
            get_logger(), "播报终态已回流，%s业务占用即刻释放（互斥窗口降级为兜底超时）",
            business_name());
          business_ = Business::kNone;
          business_busy_until_.reset();
        }
      } else if (session_ && now >= session_next_at_) {
        play_current();
      } else if (motion_renew_until_.has_value() &&
        (now >= motion_renew_next_at_ || now >= *motion_renew_until_))
      {
        // 动作续播（2026-10-28 现场需求：讲解句长动作短）：窗口到期
        //（本句播完）停手；否则续触轮换池下一个动作（interrupt=true
        // 打断旧动作无缝接续，语音才是主时间轴）并排下一次
        if (now >= *motion_renew_until_) {
          motion_renew_until_.reset();
        } else {
          trigger_speech_motion();
          motion_renew_next_at_ =
            now + std::chrono::seconds(motion_repeat_seconds_);
        }
      } else if (suppress_renew_active_ && now >= suppress_next_at_) {
        if (suppress_stop_at_.has_value() && now >= *suppress_stop_at_) {
          // 尾部宽限到期（最后一条语音已播完）：停续期，刷短 TTL 立即解除
          suppress_renew_active_ = false;
          suppress_stop_at_.reset();
          set_mic_muted(false);
          if (suppress_armed_) {
            suppress_wakeup(static_cast<std::uint32_t>(suppress_tail_flush_ms_));
          }
          suppress_armed_ = false;
        } else {
          // 播报进行中：续期 tick 照常（驱动尾部到期检查），但抑制服务
          // 调用仅静音失败的降级场景发（正常路径麦克风已静音，无对象）
          suppress_next_at_ = now +
            std::chrono::milliseconds(suppress_renew_interval_ms_);
          if (suppress_armed_) {
            suppress_wakeup(static_cast<std::uint32_t>(suppress_ttl_ms_));
          }
        }
      } else if (screen_ && screen_->active() && now >= screen_->deadline()) {
        // 二维码常驻超时（display_show_timeout_seconds，全流程统一 2 分钟，
        // 2026-10-22 现场需求）：以 display_restore_priority（高于上屏的 0）
        // 播默认表情打断循环视频（2026-10-23 真机故障：同优先级打不断，
        // 受理 ok 但屏幕滞留二维码）；取消失败时 deadline 已被顺延 5s，
        // 本分支到点自动重试撤屏（ScreenDisplay::cancel 失败保持常驻态）
        screen_->cancel("超时2min");
      } else if (relocate_pending_) {
        // 重定位验收轮询（250ms 周期）：终态只认导航模块的验收结果
        //（set_pose goal 的 result）或超时（relocate_timeout_seconds，默认 30s）
        const auto verified = nav_->poll_set_pose_verified(relocate_waypoint_);
        if (verified.has_value()) {
          finish_relocate(*verified);
        } else if (now >= relocate_deadline_) {
          RCLCPP_WARN(
            get_logger(),
            "重定位等待验收超时（%ds）: 航点「%s」operation_id=%s，按 failed 收尾",
            relocate_timeout_seconds_, relocate_waypoint_.c_str(),
            relocate_operation_id_.c_str());
          finish_relocate(false);
        }
      } else if (tour_) {
        // 导览受理成功 -> 播引导语（2026-10-17 现场需求：确认 [导航]
        // 收到应答 accepted=true 后 TTS「请跟我来…」；被拒不播——
        // 终态会走 navigate_failed 提示。取消/被新任务替换的旧受理
        // 事件由 TourRunner seq 作废，不会到这）
        if (const auto accepted = tour_->poll_accepted()) {
          if (accepted->first && business_ == Business::kTour &&
            tour_->navigating())
          {
            // 目标是迎宾点（2026-10-19 现场需求：App 下发 navigate 去
            // 迎宾点，普通引导语「请跟我来…去看看迎宾点这款车」语义不通
            //——迎宾点不是车型）：改播迎宾引导语 navigate_lead_welcome_text。
            // 判定 = 迎宾准备（该链路
            // 本身就是去迎宾点）或 code 命中 welcome_point_code 或显示名
            // 含「迎宾」关键词（与点位解析的关键词机制同思路，不绑定
            // 单一 code）；未配置迎宾引导语回退普通引导语。送客除外
            //（2026-10-21 现场需求：送客虽也去迎宾点，但顾客在跟随，
            // 途中播报用普通安抚池「请跟紧我」更贴切，赶路池「请大家
            // 让一让」不对）
            const bool to_welcome = !nav_send_off_ &&
              (nav_welcome_prep_ ||
              (!welcome_point_code_.empty() &&
              tour_->current_point() == welcome_point_code_) ||
              nav_point_display_.find("迎宾") != std::string::npos);
            // 引导语（2026-10-21 现场需求：仅 App MQTT 下发的导航播，
            // 语料触发不播——agent 已说过带顾客过去的话术，再说就重复；
            // 迎宾准备/送客同不播，agent 已说过触发语料）
            const bool lead_play = !nav_local_ && !nav_welcome_prep_ &&
              !nav_send_off_ && !navigate_lead_text_.empty();
            std::string lead_text;
            if (lead_play) {
              lead_text =
                (to_welcome && !navigate_lead_welcome_text_.empty())
                ? navigate_lead_welcome_text_
                : replace_point_placeholder(navigate_lead_text_, nav_point_display_);
            }
            // 话术不在本行打（免同句双打）：播报原文见 speak_hint
            // 「语音提示已受理」行
            RCLCPP_INFO(
              get_logger(), "导航受理成功，%s%s",
              lead_play ? "播引导语" : "跳过引导语",
              lead_play && to_welcome && !navigate_lead_welcome_text_.empty()
              ? "（迎宾点）" : "");
            if (lead_play) {
              speak_hint(lead_text, "navigate_lead");
            }
            const auto & chatter_pool = to_welcome
              ? navigate_chatter_welcome_texts_
              : navigate_chatter_texts_;
            const bool chatter_first = !(to_welcome
              ? navigate_chatter_welcome_first_text_
              : navigate_chatter_first_text_).empty();
            if (!chatter_pool.empty() || chatter_first) {
              nav_chatter_welcome_ = to_welcome;
              nav_chatter_first_pending_ = chatter_first;
              nav_chatter_next_at_ = now + nav_chatter_next_delay(lead_text);
              RCLCPP_INFO(
                get_logger(), "导航途中播报已安排: %s %zu 条%s，每句播完静默 %ds 后下一句",
                to_welcome ? "迎宾池" : "普通池", chatter_pool.size(),
                chatter_first ? "，首句固定" : "",
                navigate_chatter_silence_seconds_);
            }
          }
        }
        // 到达槽消费（导航中 250ms 轮询；空闲时到达事件也会在唤醒后被拾取）
        if (auto arrival = tour_->poll_arrival()) {  // 非 const：默认讲解词兜底要写 texts
          // v0.24.0（P-038）：导航终态回执——到达即 success（讲解词播报与否
          // 属本地细节，不影响结果）；失败回 navigation_failed
          // ponytail: NavOutcome.detail 是自由文本（服务被拒/超时/FAILED 归并），
          // 无结构化分类，point_not_found/localization_lost 不猜——细分时给
          // NavOutcome 加 reason 枚举再映射
          // v0.24.0（P-038）+ v0.30.0（P-044）：MQTT 下行走 navigate.result
          // 回执；语料触发（nav_local_）已发过 navigate.triggered
          //（nav_triggered_）则改发 navigate.completed 终态上报（payload
          // 无 operation_id，与 triggered 一一对应），未发 triggered 的
          //（前置重定位失败仍出发的导航）不发——App 不感知
          if (!nav_local_) {
            send_action_result(
              NAVIGATE_RESULT_TYPE, nav_operation_id_,
              arrival->ok ? ACTION_RESULT_SUCCESS : ACTION_RESULT_FAILED,
              arrival->ok ? "" : RESULT_REASON_NAVIGATION_FAILED);
          } else if (nav_triggered_) {
            send_nav_completed(
              NAVIGATE_COMPLETED_TYPE, nav_point_display_,
              arrival->ok ? ACTION_RESULT_SUCCESS : ACTION_RESULT_FAILED,
              arrival->ok ? "" : RESULT_REASON_NAVIGATION_FAILED);
          }
          nav_operation_id_.clear();
          nav_triggered_ = false;
          if (arrival->ok) {
            // ---- 迎宾准备/送客到达（2026-10-18 现场需求；送客 2026-10-21）：
            // 变形四足作为待命形态（摄像头舵机收平 0 度），变形完成后最后
            // 播报——迎宾准备播就位语「元气满满，做好准备迎接顾客」、
            // 送客播感谢语 send_off_done_text「欢乐的时光过的特别快…」。
            // 不走默认车型讲解词兜底（迎宾点不是车型点）；变形阻塞 worker
            // 等待（与迎宾变形同模式，上限 transform_wait_timeout_seconds），
            // 期间新指令排队 ----
            if (nav_welcome_prep_ || nav_send_off_) {
              const bool send_off = nav_send_off_;
              const char * const label = send_off ? "送客" : "迎宾准备";
              nav_welcome_prep_ = false;
              nav_send_off_ = false;
              const std::string prep_source =
                std::string(label) + ":" + tour_->current_point();
              // ---- 到达后延迟变形（2026-10-18 测试需求）：收到导航 result
              // 不立即变形，等 welcome_prep_transform_delay_seconds 秒再变
              // 四足（送客共用：顾客可能还在旁边道别，立即变形不安全）。
              // 分片等待：rclcpp 关停时数百毫秒内退出不卡 worker 收尾；
              // 等待期间新指令排队（与下方变形阻塞 worker 同模式）----
              if (welcome_prep_transform_delay_seconds_ > 0) {
                RCLCPP_INFO(
                  get_logger(), "%s到达: %s，等 %ds 后再变形四足: %s",
                  label, tour_->current_point().c_str(),
                  welcome_prep_transform_delay_seconds_, prep_source.c_str());
                const auto delay_deadline = std::chrono::steady_clock::now() +
                  std::chrono::seconds(welcome_prep_transform_delay_seconds_);
                while (rclcpp::ok() &&
                  std::chrono::steady_clock::now() < delay_deadline)
                {
                  std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
              }
              RCLCPP_INFO(
                get_logger(), "%s到达: %s，开始变形四足（摄像头收起）: %s",
                label, tour_->current_point().c_str(), prep_source.c_str());
              transform_->ensure_quadruped(prep_source);
              if (transform_->in_flight()) {
                RCLCPP_INFO(
                  get_logger(), "四足变形进行中，变形完成后播%s: %s",
                  send_off ? "感谢语" : "就位语", prep_source.c_str());
              }
              transform_->wait_done();
              // ---- 送客二维码上屏（阶段 2，2026-10-22 现场需求）：变形
              // 完成、感谢语开播前显示（先屏后声，与迎宾同款时序；常驻
              // 时长 display_show_timeout_seconds 独立计时，新业务到达
              // 由各业务入口 screen_->cancel 让位）。future 此刻通常早已
              // 完成（准备 ~3.5s << 导航时长），get() 零等待 ----
              if (send_off) {
                finish_send_off_qr(true, prep_source);
              }
              // 变形结束（成败均已在变形线程记日志）-> 最后播报：送客播
              // 感谢语、迎宾准备播就位语（短内容不静音：结束后要能被唤醒
              // 迎接顾客；空串 = 禁用，到达即释放业务）
              const std::string done_text =
                send_off ? send_off_done_text_ : welcome_prep_done_text_;
              if (!done_text.empty()) {
                const auto busy_until = estimate_busy_until(
                  {done_text}, tour_speak_interval_seconds_);
                start_session(
                  {done_text}, TTS_MODE_SEQUENCE,
                  tour_speak_interval_seconds_, /*loop_forever=*/false,
                  prep_source,
                  /*mute_mic=*/false);
                // 保持导览业务，进入播报窗口（终态回流即刻释放）
                business_busy_until_ = busy_until;
              } else {
                business_ = Business::kNone;
                business_busy_until_.reset();
              }
            } else {
              // 指令未携带讲解词（MQTT 下行不带 texts / 语料触发）->
              // 到达播报强制映射表权威（2026-10-19 现场需求）：
              //   1) 指令自带 texts（上方分支，最具体，逐次内容优先）
              //   2) 点位映射表条目 text（v0.28.0 P-042：App 推表配置的
              //      播放词——现场在 App 改表即生效）
              //   3) 未配置 = 到达不播报（下方既有分支，释放占用）
              if (arrival->texts.empty()) {
                if (const auto point_text =
                  point_map_->text_for_code(tour_->current_point()))
                {
                  arrival->texts = {*point_text};
                  RCLCPP_INFO(get_logger(), "讲解词: %s", point_text->c_str());
                  RCLCPP_INFO(
                    get_logger(),
                    "导航到达: %s，未携带讲解词，播点位映射表播放词（%zu 字节）",
                    tour_->current_point().c_str(), point_text->size());
                }
              }
              if (arrival->texts.empty()) {
                RCLCPP_INFO(
                  get_logger(), "导航到达: %s（未携带讲解词且无默认词，不播报）",
                  tour_->current_point().c_str());
                // 到达且无需讲解：导览业务到此结束，释放占用
                business_ = Business::kNone;
                business_busy_until_.reset();
              } else {
                RCLCPP_INFO(
                  get_logger(), "导航到达: %s，开始汽车讲解（%zu 条）",
                  tour_->current_point().c_str(), arrival->texts.size());
                // 讲解窗口先算（start_session 会 move 走 texts）
                const auto busy_until =
                  estimate_busy_until(arrival->texts, tour_speak_interval_seconds_);
                const std::string source = "讲解:" + tour_->current_point();
                start_session(
                  std::move(arrival->texts), TTS_MODE_SEQUENCE,
                  tour_speak_interval_seconds_, /*loop_forever=*/false, source,
                  /*mute_mic=*/true);  // 导航讲解词：长内容，静音防打断
                // 保持导览业务，进入讲解窗口期（窗口到期即释放）
                business_busy_until_ = busy_until;
              }
            }
          } else {
            RCLCPP_ERROR(
              get_logger(), "导航失败: %s (%s)，%s",
              tour_->current_point().c_str(), arrival->detail.c_str(),
              nav_local_
                ? (nav_triggered_
                  ? "已发 navigate.completed/failed:navigation_failed 终态上报（P-044）"
                  : "不发协议回执与终态上报（语料触发但未发 triggered，前置重定位失败）")
                : "已回执 navigate.result/failed:navigation_failed");
            // 导航失败：导览业务结束，释放占用（迎宾准备/送客标志一并清理）
            if (nav_send_off_) {
              finish_send_off_qr(false, "");  // 送客导航失败：丢弃二维码显示（收割后台防泄漏）
            }
            nav_welcome_prep_ = false;
            nav_send_off_ = false;
            business_ = Business::kNone;
            business_busy_until_.reset();
            // 失败语音提示（2026-10-17 现场需求：导航栈异常现场要能听到，
            // 提示排查 navigation service；业务已释放不强抢，空串=禁用）。
            // 2026-10-19 现场需求：goal 被拒（detail 以「导航任务被拒」开头，
            // 典型=语料配的点位不在航点表里，如 F 而路线只有 Y/J/A/E/H/K）
            // 属点位未配置而非导航栈故障，播「还未设置该点位信息」指引
            // 配置，与栈故障提示区分开
            const bool waypoint_missing =
              arrival->detail.rfind("导航任务被拒", 0) == 0;
            speak_hint(
              waypoint_missing ? navigate_waypoint_missing_text_ : navigate_failed_text_,
              waypoint_missing ? "navigate_waypoint_missing" : "navigate_failed");
          }
        }
        // ---- 导航途中随机播报（2026-10-21 现场需求）：在途且到点 -> 按受理
        // 时定格的池（nav_chatter_welcome_：迎宾点专用/普通安抚）随机挑一
        // 句播报（speak_hint 不强抢、不触发伴随动作——机器人正在走路，
        // 不做说话动作）；下一句时刻 = 本句播报时长估算 + 静默秒数
        //（间隔语义 = 两句之间的静默，不含播报时间，2026-10-21 现场
        // 需求）。到达/取消/失败后 navigating()=false 或未安排（池空，
        // nullopt）即自然停 ----
        if (business_ == Business::kTour && tour_->navigating() &&
          nav_chatter_next_at_.has_value() && now >= *nav_chatter_next_at_)
        {
          const std::string chatter = pick_nav_chatter_text();
          if (!chatter.empty()) {
            speak_hint(chatter, "navigate_chatter");
            nav_chatter_next_at_ = now + nav_chatter_next_delay(chatter);
          }
        } else if (!(tour_ && tour_->navigating())) {
          nav_chatter_next_at_.reset();  // 导航已结束（到达/取消/失败）：清定时
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 业务互斥（仅 worker 线程调用，成员免锁）：同一时刻至多一个业务。
// 播报类（tts/迎宾/讲解）以 TtsStatus 终态回流为准即刻释放占用；字数
// 估算窗口（tts/讲解）与固定总超时（迎宾 welcome_flow_timeout_seconds）
// 仅作终态丢失时的兜底；导览在途期以 TourRunner 导航状态为准
//（到达/失败事件即终态，窗口另设）
// ---------------------------------------------------------------------------
bool MqttGuideBridgeNode::business_busy() const
{
  if (relocate_pending_) {
    return true;  // 重定位等待导航模块验收：不受估算窗口限制（终态=验收/超时）
  }
  if (business_ == Business::kTour && tour_ && tour_->navigating()) {
    return true;  // 导航在途或到达待消费：不受估算窗口限制
  }
  return business_busy_until_.has_value() &&
         std::chrono::steady_clock::now() < *business_busy_until_;
}

const char * MqttGuideBridgeNode::business_name() const
{
  switch (business_) {
    case Business::kTts: return "播报";
    case Business::kWelcome: return "迎宾";
    case Business::kTour: return "导览";
    case Business::kRelocate: return "重定位";
    case Business::kShowQr: return "二维码显示";
    case Business::kCancelQr: return "取消二维码";
    default: return "无";
  }
}

std::chrono::steady_clock::time_point MqttGuideBridgeNode::estimate_busy_until(
  const std::vector<std::string> & texts, int interval_seconds) const
{
  // 迎宾窗口原有公式抽出（三处共用）：lead + Σ(字数估算×scale) + 条间间隔
  int64_t busy_ms = 0;
  for (const auto & t : texts) {
    busy_ms += suppress_tail_lead_ms_ +
      static_cast<int64_t>(
        estimate_speech_seconds(t, tts_speech_ms_per_char_) * 1000.0 *
        suppress_tail_scale_);
  }
  if (!texts.empty()) {
    busy_ms += static_cast<int64_t>(interval_seconds) *
      static_cast<int64_t>(texts.size() - 1) * 1000;
  }
  return std::chrono::steady_clock::now() + std::chrono::milliseconds(busy_ms);
}

void MqttGuideBridgeNode::handle_command(const CommandEvent & event)
{
  // 业务互斥：迎宾/导览/播报/重定位/二维码显示/取消二维码显示任一业务
  // 进行中，新业务指令一律拒绝（单业务占用模型，2026-10-16 需求：一次
  // 只保证一个业务执行；2026-10-17 起 relocate 亦为不可打断业务——终态
  // 要等导航模块验收，等待期占互斥；同日起 show_qr/cancel_qr 亦为互斥
  // 业务（现场需求）——显示与取消二维码显示是两个业务，各占
  // qr_business_timeout_seconds（默认 20s）窗口，与其他业务互斥）。
  // v0.24.0（P-038）起四类业务指令均有回执面：tts 回
  // rejected(business_busy)，navigate/relocate/scene 回各自 result 的
  // failed(busy)——Server 落库推 App，用户可感知「忙碌」稍后再试。
  // 本地注入指令（event.local，语料触发）不发协议回执（Server 不认识其
  // operation_id），拒绝提示语音也无需播（show_qr/cancel_qr 被拒时 agent
  // 正在说命中语料，自产提示只会相撞；speak_busy_hint 对二维码业务
  // 天然不播）。tts.stop 永远放行——它是解除占用的唯一指令（重定位
  // 等待期除外：见 tts.stop 分支，初值已发的等待不归它停）；
  // navigate.stop 永远放行——它是取消类指令，只取消在途导航（无导航
  // no-op，不影响播报类业务）。cancel_qr 可抢占 kShowQr 窗口——取消类
  // 动作与 tts.stop/navigate.stop 同语义：显示保护期（20s）内「收到后
  // 取消二维码显示」必须生效，被自己的显示业务挡住即功能失效；抢占后
  // 业务切换为 kCancelQr 重计窗口。show_qr 无抢占豁免（其他业务进行中
  // 老老实实被拒）。point_map 永远放行（v0.25.0 P-039）——纯数据同步
  //（映射表全量替换），无任何业务副作用，不占互斥也不因业务在跑而被拒
  //（2026-10-17 需求：映射表消息处理不受业务处理影响）。语料触发的
  // 迎宾走同一链路（inject_command），同样受限
  const bool qr_cancel_preempts_show =
    event.type == "cancel_qr" && business_ == Business::kShowQr;
  if (event.type != "tts.stop" && event.type != "navigate.stop" &&
    event.type != "point_map" && !qr_cancel_preempts_show && business_busy())
  {
    if (event.type == "tts") {
      RCLCPP_WARN(
        get_logger(), "业务互斥: %s进行中，拒绝 tts operation_id=%s（回执 business_busy）",
        business_name(), event.operation_id.c_str());
      send_tts_result(
        event.operation_id, TTS_RESULT_REJECTED, TTS_REASON_BUSINESS_BUSY);
    } else {
      RCLCPP_WARN(
        get_logger(), "业务互斥: %s进行中，拒绝 type=%s operation_id=%s（%s）",
        business_name(), event.type.c_str(), event.operation_id.c_str(),
        event.local ? "不发协议回执（语料触发）" : "回执 failed/busy");
      if (!event.local) {
        send_action_result(
          event.type + ".result", event.operation_id,
          ACTION_RESULT_FAILED, RESULT_REASON_BUSY);
      }
    }
    // 语音提示（现场可感知，2026-10-17 需求）：非语音业务进行中时播
    //「我正在{业务}，请稍后」；迎宾/播报/讲解中正在出声，不播
    speak_busy_hint();
    return;
  }

  if (event.type == "tts") {
    // 单业务占用：先按原文估算占用窗口（start_session 会拷贝 texts）
    const auto busy_until = estimate_busy_until(event.texts, event.interval_seconds);
    // 新播报列表（App 下发）：从头开始播；单轮播报：列表播完一轮自行结束
    // 新业务到达 -> 迎宾二维码让位（恢复默认表情）
    screen_->cancel("tts");
    start_session(
      event.texts, event.mode, event.interval_seconds, /*loop_forever=*/false,
      "tts:" + event.operation_id,
      /*mute_mic=*/false);  // App 下发 tts：不静音（唤醒抑制照常）
    // 契约 v0.6.0 回执：服务不可达/超时 = 播放子系统不可用 ->
    // rejected(playback_unavailable)；SDK 仲裁/业务拒绝属本地忙碌策略
    //（按 tts_retry_seconds 持续重试），照常回 accepted
    const auto result = play_current();
    if (result.service_failed) {
      send_tts_result(
        event.operation_id, TTS_RESULT_REJECTED, TTS_REASON_PLAYBACK_UNAVAILABLE);
      // 受理失败：业务未建立，不占用（与回执语义一致）。会话与唤醒抑制
      // 一并终止——回执已拒，保留会话只会空转重试（服务恢复后播出去
      // 是 App 不知情的幽灵播报），且抑制续期无人收尾（service_failed
      // 路径不会设置尾部宽限 suppress_stop_at_，续期将永续）
      cancel_session();
      business_ = Business::kNone;
      business_busy_until_.reset();
    } else {
      send_tts_result(event.operation_id, TTS_RESULT_ACCEPTED);
      business_ = Business::kTts;
      business_busy_until_ = busy_until;
    }
    return;
  }

  if (event.type == "scene") {
    // MQTT 迎宾三场景路由（2026-10-22 现场需求）：texts 带固定指令时
    // 分流准备迎宾/欢送顾客（开始迎宾指令仅清除 texts 后仍走原流程）。
    // 拷贝一份再路由：命中开始迎宾指令时改写 texts（agent 话术不作
    // 迎宾词，迎宾词走映射表），不改原始事件
    CommandEvent scene_event = event;
    if (route_welcome_scene(scene_event)) {
      return;  // 已分流处理（含 scene.result 回执）
    }
    handle_scene(scene_event);
    return;
  }

  if (event.type == "show_qr") {
    handle_show_qr(event);
    return;
  }

  if (event.type == "cancel_qr") {
    handle_cancel_qr(event);
    return;
  }

  if (event.type == "point_map") {
    handle_point_map(event);
    return;
  }

  if (event.type == "relocate") {
    handle_relocate(event);
    return;
  }

  if (event.type == "welcome_prep") {
    handle_welcome_prep(event);
    return;
  }

  // 送客（2026-10-21 现场需求，仅本地注入：AgentMonitor 命中
  // agent_monitor.send_off_phrases 语料）：与迎宾准备同链路，仅到达后
  // 播感谢语（send_off=true 区分）
  if (event.type == "send_off") {
    handle_welcome_prep(event, /*send_off=*/true);
    return;
  }

  if (event.type == "navigate") {
    handle_navigate(event);
    return;
  }

  if (event.type == "tts.stop") {
    // tts.stop 无回执（发后即忘）。本地策略（docs/design.md §3.4）：
    // 掐断当前语音 + 终止播报会话 + 取消进行中的导航（stop = 放弃本次
    // 讲解流程）。SDK 无 StopTts：以理论最高权重抢占掐断。
    // 新业务到达 -> 迎宾二维码一并取消
    screen_->cancel("tts.stop");
    // 停止一切 = 业务占用立即释放（后续指令即刻可受理）。唯一例外是
    // 重定位等待期（2026-10-17）：初值已发给定位、验收 ~1.5s 自然终态，
    // 「停止播报/导航」的语义管不到它；等待期无会话无导航，下面各步对
    // 它本就是 no-op，占用保持到 finish_relocate
    if (business_ == Business::kRelocate) {
      RCLCPP_INFO(
        get_logger(), "tts.stop 到达：重定位等待验收不受影响（终态由导航模块给出）");
    } else {
      business_ = Business::kNone;
      business_busy_until_.reset();
    }
    // 语音已被 1099 掐断：唤醒抑制立即结束（刷短 TTL 立即解除）
    if (suppress_renew_active_) {
      suppress_renew_active_ = false;
      suppress_stop_at_.reset();
      set_mic_muted(false);
      if (suppress_armed_) {
        suppress_wakeup(static_cast<std::uint32_t>(suppress_tail_flush_ms_));
      }
      suppress_armed_ = false;
    }
    const bool had_session = session_.has_value();
    const bool had_nav = tour_->navigating();
    if (had_nav && !nav_operation_id_.empty()) {
      // 被中止的导航对原指令就是失败确定：回 failed/aborted（v0.24.0）。
      // 语料触发（nav_local_）不发回执，已发过 triggered（P-040）的改发
      // navigate.completed/failed:aborted 终态上报（P-044——tts.stop 停止
      // 一切，在途语音导航一并被打断，App 侧 triggered 有终态可配对）
      if (!nav_local_) {
        send_action_result(
          NAVIGATE_RESULT_TYPE, nav_operation_id_, ACTION_RESULT_FAILED,
          RESULT_REASON_ABORTED);
      } else if (nav_triggered_) {
        send_nav_completed(
          NAVIGATE_COMPLETED_TYPE, nav_point_display_, ACTION_RESULT_FAILED,
          RESULT_REASON_ABORTED);
      }
      nav_operation_id_.clear();
      nav_triggered_ = false;
    }
    if (nav_send_off_) {
      finish_send_off_qr(false, "");  // 送客被停止：丢弃二维码显示（收割后台防泄漏）
    }
    nav_welcome_prep_ = false;  // 迎宾准备/送客在途一并取消（不变形不播就位语/感谢语）
    nav_send_off_ = false;
    cancel_session();
    tour_->cancel();
    const auto result = tts_->stop();
    if (result.ok) {
      RCLCPP_INFO(
        get_logger(), "停止播报: 会话=%s 导航=%s",
        had_session ? "有" : "无", had_nav ? "有" : "无");
    } else {
      RCLCPP_WARN(get_logger(), "停止播报失败: %s", result.detail.c_str());
    }
    return;
  }

  if (event.type == "navigate.stop") {
    // navigate.stop 无回执（发后即忘，v0.23.0 与 tts.stop 同构）。契约：
    // 只停止当前正在进行的导航，无导航时 no-op；不指定导航目标；到达后
    // 的讲解/其他播报不归它管（tts.stop 才停一切）。停止方式归本地：
    // Mock 下即时取消；真机导航栈对接后可换减速停靠。
    // 被停止的 navigate 回 failed/aborted（v0.24.0 P-038）
    // ponytail: 无独立"导航在途但业务非导览"场景（handle_navigate 前置
    // cancel_session），无需处理语音/二维码
    if (tour_->navigating()) {
      if (!nav_operation_id_.empty()) {
        // 被停止的 navigate 一律有终态（v0.24.0 P-038 / v0.30.0 P-044）：
        // MQTT 下行回 failed/aborted 回执；语料触发的导航曾以本地铸造的
        // operation_id 发孤儿 navigate.result（Server 对应不到任何已下发
        // 指令只记日志），P-044 起改发 navigate.completed/failed:aborted
        // 终态上报（payload {point_name, trigger, outcome, reason}，无
        // operation_id，与 navigate.triggered 一一配对，Server 去重后推
        // App robot.navigation.completed）——后端对语音触发的导览既有
        //「开始」也有「被停止」终态可感知（2026-10-18 现场需求的正确
        // 协议化表达）。未发 triggered 的（前置重定位失败仍出发）不发
        if (!nav_local_) {
          send_action_result(
            NAVIGATE_RESULT_TYPE, nav_operation_id_, ACTION_RESULT_FAILED,
            RESULT_REASON_ABORTED);
        } else if (nav_triggered_) {
          send_nav_completed(
            NAVIGATE_COMPLETED_TYPE, nav_point_display_, ACTION_RESULT_FAILED,
            RESULT_REASON_ABORTED);
        }
        nav_operation_id_.clear();
        nav_triggered_ = false;
      }
      tour_->cancel();  // 取消导航并清空到达槽（不触发回调）
      if (nav_send_off_) {
        finish_send_off_qr(false, "");  // 送客被停止：丢弃二维码显示（收割后台防泄漏）
      }
      nav_welcome_prep_ = false;  // 迎宾准备/送客在途一并取消（不变形不播就位语/感谢语）
      nav_send_off_ = false;
      business_ = Business::kNone;  // 导览业务到此终止，释放占用
      business_busy_until_.reset();
      RCLCPP_INFO(
        get_logger(), "停止导航: point=%s operation_id=%s（已回执 failed/aborted%s）",
        tour_->current_point().c_str(), event.operation_id.c_str(),
        nav_local_ ? "，语料触发的导航" : "");
    } else {
      RCLCPP_INFO(
        get_logger(), "停止导航: 无导航在途 no-op operation_id=%s",
        event.operation_id.c_str());
    }
  }
}

// ---------------------------------------------------------------------------
// 语料触发二维码显示（type=show_qr，仅本地注入：AgentMonitor 命中
// agent_monitor.qr_phrases 语料，如「收到，这就给您显示二维码，您可以
// 扫描二维码留下您的个人信息哦。」）。纯屏幕动作：显示默认二维码图片
//（同迎宾未携带 qr_sha256 的兜底 config/QRCode.png），常驻
// display_show_timeout_seconds（默认 120s=2 分钟，2026-10-22 现场需求
// 全流程统一时长，worker 按 ScreenDisplay deadline 超时恢复默认表情）。已在显示时重复触发：互斥窗口（20s）内
// 复读被拒（business_busy），窗口外复读重新上屏并重置常驻计时。业务语义（2026-10-17 现场需求）：显示二维码是一个
// 互斥业务——上屏成功即占 kShowQr 业务 qr_business_timeout_seconds
//（默认 20s），期间其他业务（tts/scene/navigate/relocate/cancel_qr 除
// 外——cancel_qr 可抢占）被拒，保住二维码不被新业务立即让位；窗口到期
// 自然释放（二维码常驻不受影响，仍由 display_show_timeout_seconds 或
// 新业务到达时的让位规则收尾）。上屏失败（默认图缺失/转码/推送失败）
// 仅告警不占业务（与 tts 受理失败不占用同语义）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_show_qr(const CommandEvent & event)
{
  if (default_qr_path_.empty()) {
    RCLCPP_WARN(
      get_logger(), "语料触发二维码: 默认二维码未配置或缺失，本次不显示");
    return;
  }
  const auto prep = screen_->prepare(default_qr_path_, "default-qr");
  if (!prep.ok) {
    RCLCPP_WARN(
      get_logger(), "语料触发二维码准备失败(本次不显示): %s", prep.detail.c_str());
    return;
  }
  const auto display = screen_->play_prepared("语料:" + event.operation_id);
  if (!display.ok) {
    RCLCPP_WARN(
      get_logger(), "语料触发二维码显示失败: %s", display.detail.c_str());
    return;
  }
  business_ = Business::kShowQr;
  business_busy_until_ = std::chrono::steady_clock::now() +
    std::chrono::seconds(qr_business_timeout_seconds_);
  RCLCPP_INFO(
    get_logger(), "二维码显示业务建立: %ds 互斥窗口（常驻时长由屏幕超时独立计时）",
    qr_business_timeout_seconds_);
}

// ---------------------------------------------------------------------------
// 语料触发取消二维码显示（type=cancel_qr，仅本地注入：AgentMonitor 命中
// agent_monitor.qr_cancel_phrases 语料，如「好的，这就取消二维码显示」，
// 2026-10-17 现场需求：收到后取消二维码显示）。取消动作幂等：未显示时
// no-op（ScreenDisplay::cancel 幂等），正在显示则恢复默认表情撤屏。
// 业务语义（同现场需求）：取消二维码显示是第二个互斥业务——无论是否
// 有图在屏，受理即占 kCancelQr 业务 qr_business_timeout_seconds（默认
// 20s）与其他业务互斥；若此前处于 kShowQr 显示业务窗口内则本指令经
// handle_command 的抢占豁免进入并切换业务重计窗口（取消类动作必须
// 生效）。cancel 失败（PlayEmotion 不可达）仅告警，业务照占（agent 已
// 承诺取消，保护期内不让位新业务）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_cancel_qr(const CommandEvent & event)
{
  const bool was_showing = screen_ && screen_->active();
  const auto result = screen_->cancel("语料取消:" + event.operation_id);
  if (result.ok) {
    RCLCPP_INFO(
      get_logger(), "语料触发取消二维码显示: %s（%s）", event.operation_id.c_str(),
      was_showing ? "已撤屏" : "无二维码在屏 no-op");
  } else {
    RCLCPP_WARN(
      get_logger(), "语料触发取消二维码显示失败(不影响业务占用): %s",
      result.detail.c_str());
  }
  business_ = Business::kCancelQr;
  business_busy_until_ = std::chrono::steady_clock::now() +
    std::chrono::seconds(qr_business_timeout_seconds_);
}

// ---------------------------------------------------------------------------
// 地图重定位（type=relocate，两个来源共用：MQTT 下行（App 触发，回
// relocate.result）与本地注入（AgentMonitor 命中 agent_monitor.
// relocate_phrases 语料，成功后同样发 relocate.result——2026-10-21
// 现场需求，失败仍不发）。前提：人工已把机器人摆到该航点上
// 并对准朝向。业务语义（2026-10-17 需求）：**不可打断的互斥业务**，
// 终态 = 导航模块的验收结果——链路：point_name 先经点位映射表解析
//（2026-10-17 现场需求，与 navigate 对齐：未命中即拒绝——映射表是
// 唯一权威，未命中=表与现场不同步，透传航点名只会把错误下发给
// 导航模块；播报提示 + 回执 failed:point_not_found，不发起 set_pose
// 也不占用业务；语料触发的 relocate.triggered 上报**验收通过后**才发
//（finish_relocate——只有重定位成功才发消息给后端）；成功时还加发
// relocate.result 回执（2026-10-21 需求））-> NavExecutor::set_pose 受理（真机发 /liwei_navigation/
// navigate 的 set_pose goal，**机器人不动**，服务端发一次初始位姿并
// 验收）-> 置 kRelocate 等待态（占业务互斥，新业务被拒 + 语音提示
//「我正在重定位，请稍后」）-> worker 每 250ms 轮询验收
//（poll_set_pose_verified：set_pose goal 的 result（验收完成才回），
// 真机受理后 ~1.5s 出结果）-> 终态收尾见 finish_relocate（回执 + 释占 + 播报），
// relocate_timeout_seconds（默认 30s）超时兜底。受理失败（航点名不在
// 航点表/服务不可用/状态冲突被拒）当场回执 failed 不占业务
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_relocate(const CommandEvent & event)
{
  const char * source = event.local ? "语料触发" : "MQTT 下行";

  // 点位解析（2026-10-17 现场需求，与 navigate 对齐）：point_name 先经
  // 映射表 name->code；未命中（表空/名称不在表中）直接拒绝——播报
  // 提示（含原始点位名，现场能听到缺哪个点）+ 回执
  // relocate.result/failed:point_not_found（语料触发不发回执），不发起
  // set_pose 也不占用业务
  std::string waypoint;
  if (const auto code = point_map_->resolve(event.point_name)) {
    waypoint = *code;
    RCLCPP_INFO(
      get_logger(), "重定位点位解析: 「%s」-> 航点 %s（映射表命中）",
      event.point_name.c_str(), waypoint.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(), "重定位被拒绝: 「%s」未命中点位映射表（%zu 条），%s",
      event.point_name.c_str(), point_map_->size(),
      event.local ? "不发协议回执（语料触发）" : "回执 failed/point_not_found");
    speak_hint(
      replace_point_placeholder(point_map_unmatched_text_, event.point_name),
      "point_map_unmatched");
    if (!event.local) {
      send_action_result(
        RELOCATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_POINT_NOT_FOUND);
    }
    return;
  }

  // 语音触发上报（v0.26.0 P-040）：**不在受理时发**——重定位**验收
  // 通过后**才发 relocate.triggered（见 finish_relocate）——只有
  // 重定位成功才发消息给后端，此后的变形被拒/受理被拒/验收失败/超时
  // 均不上报（App 不感知失败的语音重定位）；2026-10-21 现场需求起
  // 成功时还加发 relocate.result 回执（同样只在成功时）

  // 切换场景先撤屏（2026-10-22 现场需求）：重定位是业务切换（变形+
  // 播报），屏幕常驻的旧二维码先恢复默认表情让位（未显示 no-op）；
  // 放在点位校验之后——被拒（未命中映射表）不切换场景，二维码保留
  screen_->cancel("relocate");

  // ---- 重定位出发播报（2026-10-18 现场需求）：仅 MQTT 下行先播「马上
  // 开始重定位，请稍后」；语料触发不播——agent 已说「好的，这就在A点
  // 开始重定位」，再说就重复。先开口、立刻动（不等播完，与导航出发语
  // 同哲学）；空串 = 禁用 ----
  if (!event.local) {
    speak_hint(relocate_depart_text_, "relocate_depart");
  }
  // ---- 重定位前置变形（2026-10-18 现场需求）：set_pose 有「机器人须
  // 双足站立」的硬约束（t1_slam_nav_ws 文档），四足迎宾准备模式下必须
  // 先站起来才能重定位。两个来源（MQTT/语料触发）都变形；已是人形零
  // 动作立即通过；变形失败不阻断 set_pose（McTransform 内部已记日志）----
  transform_->ensure_biped("重定位:" + event.operation_id);
  if (transform_->in_flight()) {
    RCLCPP_INFO(
      get_logger(), "重定位前置变形进行中，变形完成后发起 set_pose: %s",
      event.operation_id.c_str());
  }
  transform_->wait_done();
  // 双足形态门禁（2026-10-18 现场需求）：set_pose 与 navigate 同走
  // /liwei_navigation/navigate action，且 t1_slam_nav_ws 文档明确
  // set_pose 有「机器人须双足站立」硬约束——未确认双足直接拒绝
  //（回 failed/transform_failed），不带注定被导航模块拒的请求跑下去
  if (!transform_->biped_confirmed()) {
    RCLCPP_WARN(
      get_logger(), "重定位被拒: 双足形态未确认（变形失败或运控不可用），"
      "不下发 set_pose: %s", event.operation_id.c_str());
    speak_hint(navigate_transform_failed_text_, "relocate_transform_failed");
    if (!event.local) {
      send_action_result(
        RELOCATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_TRANSFORM_FAILED);
    }
    return;
  }

  // 同步受理（真机毫秒级，受理超时兜底 5s）。受理被拒当场终态：
  // 不占业务、不等待（导航在途的冲突在 handle_command 互斥分支已被拒，
  // 这里的被拒来自服务端其余前置校验）
  const auto outcome = nav_->set_pose(waypoint);
  if (!outcome.ok) {
    const char * reason = outcome.point_not_found
      ? RESULT_REASON_POINT_NOT_FOUND
      : RESULT_REASON_LOCALIZATION_FAILED;
    // 回执文案按来源区分：语料触发失败不发协议回执（仅成功时发
    // relocate.result，见 finish_relocate 2026-10-21 需求）
    const std::string receipt_note = event.local
      ? std::string("不发协议回执（语料触发）")
      : "回执 failed/" + std::string(reason);
    RCLCPP_WARN(
      get_logger(), "%s重定位被拒: 航点「%s」operation_id=%s，%s（%s）",
      source, waypoint.c_str(), event.operation_id.c_str(), receipt_note.c_str(),
      outcome.detail.c_str());
    if (!event.local) {
      send_action_result(
        RELOCATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED, reason);
    }
    // 失败语音提示（2026-10-17 现场需求：服务不可用/被拒现场要能听到，
    // 提示排查 navigation service；回执照发，播报属本地增强）
    speak_hint(relocate_failed_text_, "relocate_failed");
    return;
  }

  // 受理成功：进入等待验收期（kRelocate 业务，占互斥；终态只认导航模块
  // 的验收结果或超时）。回执与播报都留到终态（finish_relocate）
  RCLCPP_INFO(
    get_logger(), "%s重定位已受理: 航点「%s」operation_id=%s，等待导航模块验收（%s）",
    source, waypoint.c_str(), event.operation_id.c_str(), outcome.detail.c_str());
  relocate_pending_ = true;
  relocate_operation_id_ = event.operation_id;
  // 显示名（映射表 name，反查失败用原始 point_name）：终态验收通过后
  // 语料触发的 relocate.triggered 上报用（2026-10-19 需求），受理时定格
  //（等待期 point_map 可能被 App 重推，终态反查会漂移）
  relocate_display_name_ = point_display_name(waypoint, event.point_name);
  relocate_waypoint_ = std::move(waypoint);
  relocate_point_name_ = event.point_name;
  relocate_texts_ = event.texts;
  relocate_local_ = event.local;
  relocate_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::seconds(relocate_timeout_seconds_);
  business_ = Business::kRelocate;
  business_busy_until_.reset();  // 等待期不受估算窗口约束（deadline 另记）
}

// ---------------------------------------------------------------------------
// 重定位终态收尾（worker 线程，验证结果来自 poll_set_pose_verified 或
// 超时）：MQTT 下行回执 relocate.result（验收通过=success；验收失败/
// 超时=failed:localization_failed）；语料触发验收通过时发
// relocate.triggered 上报 + relocate.completed 终态上报（P-044，替代
// 原 2026-10-21 方案的孤儿 relocate.result；失败仍都不发——App 不感知
// 失败的语音重定位）
// -> 释放业务占用（业务在回执发出后才结束）
// -> 成功时播报（texts 优先，缺省「{point_name}点 重定位 成功」（重定位
// 前后空格分段，TTS 连读易错），单轮；MQTT 下行未携带 texts 不播默认
// 成功语——2026-10-28 现场需求，App 已收 success 回执无须复读；显式
// texts 与语料触发仍播；
// 走播报会话，不静音麦克风（2026-10-17：仅迎宾/导航讲解需要 mute），
// 唤醒抑制照常，替换当前会话）；失败播 relocate_failed_text 提示
//（2026-10-17 现场需求：提示排查 navigation service）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::finish_relocate(bool verified)
{
  const auto source = relocate_local_ ? "语料触发" : "MQTT 下行";
  relocate_pending_ = false;
  business_ = Business::kNone;
  business_busy_until_.reset();
  if (verified) {
    RCLCPP_INFO(
      get_logger(), "%s重定位验收通过: 航点「%s」operation_id=%s，%s",
      source, relocate_waypoint_.c_str(), relocate_operation_id_.c_str(),
      relocate_local_
        ? "语料触发：发 relocate.triggered 上报 + relocate.completed 终态上报"
          "（成功后才发，2026-10-19 需求；终态自 P-044 起改用 completed，"
          "不再发本地 operation_id 的孤儿 relocate.result）"
        : "回执 success");
    if (!relocate_local_) {
      send_action_result(
        RELOCATE_RESULT_TYPE, relocate_operation_id_, ACTION_RESULT_SUCCESS);
    } else {
      // 2026-10-19 现场需求：语料触发的重定位只有验收通过才发
      // relocate.triggered 上报（P-040 事件形状不变，仅发布时机从
      // 受理时挪到终态成功；App 拿到即「语音重定位已成功」的事实）
      send_nav_triggered(RELOCATE_TRIGGERED_TYPE, relocate_display_name_);
      // v0.30.0（P-044，替代 2026-10-21 现场需求的孤儿 relocate.result
      // 方案）：语料触发的重定位成功改发 relocate.completed 终态上报
      //（payload {point_name, trigger:voice, outcome:success}，无
      // operation_id）——与 relocate.triggered 背靠背、按时间序一一对应，
      // Server 去重后推 App robot.navigation.completed；旧方案发本地
      // UUID v7 的 relocate.result，Server 对应不到任何已下发指令只记
      // 日志，App 无事件可收。失败/超时仍不发（App 不感知失败的语音
      // 重定位，2026-10-19 口径不变）
      send_nav_completed(
        RELOCATE_COMPLETED_TYPE, relocate_display_name_, ACTION_RESULT_SUCCESS);
    }
    // 播报在业务释放之后（协议语义：重定位成功即 success，texts 播报
    // 与否属本地细节不影响结果；播报期间新业务可替换会话——音频单会话）
    std::vector<std::string> texts = relocate_texts_;
    if (texts.empty() && !relocate_local_) {
      // MQTT 下行不播默认成功语（2026-10-28 现场需求）：App 已收
      // relocate.result/success 回执知晓结果，现场再播「X点 重定位
      // 成功」属信息重复，去掉不播。指令显式携带 texts 仍照播（App
      // 点名要播的随动话术，非本地默认文案）；语料触发不在本条需求
      // 内，维持播默认成功语——语音链路无协议回执，播报是现场唯一
      // 的成功反馈
      RCLCPP_INFO(
        get_logger(),
        "MQTT 下行重定位成功：不播默认成功语「%s点 重定位 成功」（2026-10-28 需求）",
        relocate_point_name_.c_str());
      return;
    }
    if (texts.empty()) {
      // 「重定位」前后留空格（2026-10-18 现场需求）：连写（如「A点重定位
      // 成功」）TTS 会把「点重定位」连读、发音异常，空格分段保证按
      // 「X点 / 重定位 / 成功」断句；texts 自带语料原样播，不做处理
      texts = std::vector<std::string>{relocate_point_name_ + "点 重定位 成功"};
    }
    start_session(
      std::move(texts), TTS_MODE_SEQUENCE, tour_speak_interval_seconds_,
      /*loop_forever=*/false, "重定位:" + relocate_operation_id_,
      /*mute_mic=*/false);  // 重定位播报：短内容，不静音（唤醒抑制照常）
  } else {
    RCLCPP_WARN(
      get_logger(), "%s重定位未通过: 航点「%s」operation_id=%s，%s",
      source, relocate_waypoint_.c_str(), relocate_operation_id_.c_str(),
      relocate_local_
        ? "语料触发：不发 relocate.triggered 上报、不发 relocate.result 回执（成功才发，2026-10-19/21 需求）"
        : "回执 failed/localization_failed");
    if (!relocate_local_) {
      send_action_result(
        RELOCATE_RESULT_TYPE, relocate_operation_id_, ACTION_RESULT_FAILED,
        RESULT_REASON_LOCALIZATION_FAILED);
    }
    // 失败语音提示（业务已释放；空串=禁用）
    speak_hint(relocate_failed_text_, "relocate_failed");
  }
}

// ---------------------------------------------------------------------------
// 业务互斥拒绝时的语音提示：仅非语音业务进行中播「我正在{业务}，请稍后」
//（重定位等待期/导航在途——机器人没在说话，口头告知忙碌）；迎宾/播报/
// 讲解中正在出声，插提示反而打断，不播（2026-10-17 需求：迎宾不需要说）。
// is_interrupted=false：同档排队不强抢；失败仅记日志（拒绝结果已回执）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::speak_busy_hint()
{
  std::string text;
  if (business_ == Business::kRelocate) {
    text = busy_hint_relocate_text_;
  } else if (business_ == Business::kTour && tour_ && tour_->navigating()) {
    text = busy_hint_navigate_text_;
  } else {
    return;  // 迎宾/播报/讲解（正在出声）或空闲兜底：不播
  }
  if (text.empty()) {
    return;  // 部署配置关闭了对应提示
  }
  TtsCommand command;
  command.text = text;
  command.priority_level = tts_priority_level_;
  command.priority_weight = tts_priority_weight_;
  command.is_interrupted = false;  // 排队不强抢进行中的业务播报
  command.trace_id = "busy_hint";  // 来源标识透传 SDK（AgentMonitor 自产
                                   // 回环过滤天然生效）
  const auto speak = tts_->play(command);
  if (speak.ok) {
    RCLCPP_INFO(get_logger(), "忙碌提示已受理: %s", text.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(), "忙碌提示失败（不影响拒绝回执）: %s: %s",
      text.c_str(), speak.detail.c_str());
  }
}

// ---------------------------------------------------------------------------
// 点位映射表（type=point_map，v0.25.0 P-039；v0.28.0 P-042 条目含可选
// 播放文本 text）：用 points 全量替换本地映射（不做增量合并；App 是
// 唯一权威来源），每次下发都整表覆盖旧表（点位映射 + 播放词一起落盘
// 持久化分区，断电不丢、重启自动加载）。
// 纯数据同步：不受业务互斥拦截（见 handle_command 放行说明），也不占用
// 业务；在 worker 线程串行处理，替换瞬间完成（毫秒级文件写），不与在途
// 播报/导航产生交叉。幂等由 worker 通用通道保证：同一 operation_id 重发
// 重发同一回执（receipt_cache_），不重复替换。
// 回执 point_map.result：success（已全量替换生效）/ failed
// (point_map_failed)（落盘失败，内存保留旧表 = 未生效）/ rejected
// (command_expired / invalid_request)（由 worker 过期分支与解析层
// invalid 通道回，不经本函数）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_point_map(const CommandEvent & event)
{
  // 播放词（P-042）条数单列：现场排查「App 是否已配播放词」一眼可辨
  std::size_t text_count = 0;
  for (const auto & entry : event.points) {
    if (entry.text.has_value()) {
      ++text_count;
    }
  }
  RCLCPP_INFO(
    get_logger(), "点位映射表同步: %zu 条（含播放词 %zu 条）operation_id=%s"
    "（全量替换旧表 %zu 条）",
    event.points.size(), text_count, event.operation_id.c_str(), point_map_->size());
  if (point_map_->replace(event.points)) {
    for (const auto & entry : event.points) {
      RCLCPP_INFO(
        get_logger(), "  映射: %s -> %s%s", entry.code.c_str(), entry.name.c_str(),
        entry.text.has_value() ? "（含播放词）" : "");
    }
    send_action_result(POINT_MAP_RESULT_TYPE, event.operation_id, ACTION_RESULT_SUCCESS);
    // 表生效通知（2026-10-19 现场需求）：main.cpp 依此为 AgentMonitor
    // 重建动态导航语料规则（除迎宾点外每点位按模板生成，语料随表走）。
    // 仅替换成功路径通知（落盘失败旧表保留，规则不动）
    if (on_point_map_updated) {
      on_point_map_updated(event.points);
    }
    // 同步成功后的本地语音提示（现场可感知，2026-10-17 需求）：附加
    // 动作，失败只记日志——回执 success 已发，协议语义不受影响。
    // is_interrupted=false：同优先级排队不强抢进行中的业务播报
    TtsCommand command;
    command.text = "点位映射表同步完成";
    command.priority_level = tts_priority_level_;
    command.priority_weight = tts_priority_weight_;
    command.is_interrupted = false;
    command.trace_id = "point_map";  // 来源标识透传 SDK（domain=guide_pulse，
                                     // AgentMonitor 自产回环过滤天然生效）
    const auto speak = tts_->play(command);
    if (speak.ok) {
      RCLCPP_INFO(get_logger(), "语音提示已受理: 点位映射表同步完成");
    } else {
      RCLCPP_WARN(
        get_logger(), "语音提示失败（不影响同步结果）: 点位映射表同步完成: %s",
        speak.detail.c_str());
    }
  } else {
    // 落盘失败（磁盘满/目录不可写等）：内存保留旧表，本次未生效
    RCLCPP_ERROR(
      get_logger(),
      "点位映射表落盘失败（旧表保留）: %s operation_id=%s，回执 failed/point_map_failed",
      point_map_->file_path().c_str(), event.operation_id.c_str());
    send_action_result(
      POINT_MAP_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
      RESULT_REASON_POINT_MAP_FAILED);
  }
}

// ---------------------------------------------------------------------------
// MQTT 迎宾三场景路由（2026-10-22 现场需求，见头文件声明注释）：
// App 下发 scene=welcome 的 texts 是小T的应答话术，末尾一字不差带
// 固定指令（小T提示词 §10.2 指令对照表），按指令路由三种场景——
//   「整装待发，即刻就位」准备迎宾 -> handle_welcome_prep(false)：
//     导航去迎宾点，到达变四足收摄像头、播就位语
//   「热情相送，宾至如归」欢送顾客 -> handle_welcome_prep(true)：
//     导航去迎宾点送顾客，到达变四足、播感谢语
//   「恭候光临，倒屣相迎」开始迎宾 -> 清除 texts 返回 false 走原
//     handle_scene（texts 是 agent 话术不作迎宾词，迎宾词走映射表）
// 匹配：全部 texts 归一化拼接后子串匹配（phrase_matches 自带双侧
// 归一化，与 AgentMonitor 语音触发同款同源语料——语音/MQTT 双触发
// 面一套指令词）；未命中任何指令不改写原事件（兼容 v0.20.0 App
// 指定迎宾词的旧行为）。回执：MQTT 来源照发 scene.result——
// welcome_prep/send_off 成功发起导航即 success（受理语义，到达后的
// 变形/就位语/感谢语属本地细节），前置校验失败（映射表空/点位未
// 命中/双足未确认）回 failed/scene_failed；语料触发（local）不回执
//（与既有本地注入口径一致）
// ---------------------------------------------------------------------------
bool MqttGuideBridgeNode::route_welcome_scene(CommandEvent & event)
{
  if (event.texts.empty()) {
    return false;  // 未携带话术：无从路由，走原迎宾流程（迎宾词走映射表）
  }
  // 拼接全部 texts 再匹配（agent 回复可能拆多条；归一化去空白/标点，
  // 指令是长特征短语，跨条拼接误命中可忽略）
  std::string joined;
  for (const auto & text : event.texts) {
    joined += text;
  }

  // 准备迎宾 / 欢送顾客：转本地等价指令（保留 MQTT operation_id 供
  // 日志与回执关联；点位名取指令条目「点位名|指令」部分，经映射表
  // 两级解析 + 名称关键词兜底，与语料触发同口径）
  const auto try_route =
    [&](const std::vector<WelcomeSceneRoute> & routes, bool send_off) {
      for (const auto & route : routes) {
        if (!phrase_matches(joined, route.phrase)) {
          continue;
        }
        RCLCPP_INFO(
          get_logger(),
          "迎宾场景路由: texts 命中%s指令，转%s流程（话术只用于路由，"
          "就位语/感谢语照常播）: point=%s operation_id=%s",
          send_off ? "欢送顾客" : "准备迎宾", send_off ? "送客" : "迎宾准备",
          route.point.c_str(), event.operation_id.c_str());
        CommandEvent routed = event;
        routed.type = send_off ? "send_off" : "welcome_prep";
        routed.point_name = route.point;
        routed.texts.clear();  // 话术已消费：流程内不播 agent 应答
        const bool started = handle_welcome_prep(routed, send_off);
        if (!event.local) {
          send_action_result(
            SCENE_RESULT_TYPE, event.operation_id,
            started ? ACTION_RESULT_SUCCESS : ACTION_RESULT_FAILED,
            started ? "" : RESULT_REASON_SCENE_FAILED);
        }
        return true;
      }
      return false;
    };
  if (try_route(welcome_prep_routes_, false) || try_route(send_off_routes_, true)) {
    return true;
  }

  // 开始迎宾指令：话术属 agent 应答不作迎宾词——清除 texts 让
  // handle_scene 回退映射表迎宾点条目（2026-10-19 映射表权威需求）
  for (const auto & phrase : welcome_scene_phrases_) {
    if (phrase_matches(joined, phrase)) {
      RCLCPP_INFO(
        get_logger(),
        "迎宾场景路由: texts 命中开始迎宾指令，话术不作迎宾词"
        "（迎宾词走映射表）: operation_id=%s",
        event.operation_id.c_str());
      event.texts.clear();
      break;
    }
  }
  return false;  // 走原 handle_scene（开始迎宾 / 未带指令的普通迎宾）
}

// ---------------------------------------------------------------------------
// 迎宾模块（scene=welcome）：确定迎宾词（强制映射表权威，2026-10-19
// 现场需求——指令 texts 优先，缺省取映射表迎宾点 [welcome_point_code]
// 条目的 text；两者皆空 = 不播迎宾词，引导语/
// 变形/重定位/二维码/回执照常）-> 开一轮播报。不打断导航；单轮播完
// 自行结束
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_scene(const CommandEvent & event)
{
  // 协议层已保证 scene=welcome（可扩展枚举唯一值），未知值在解析层丢弃

  // 切换场景先撤屏（2026-10-22 现场需求）：迎宾开始时若屏幕仍在常驻
  // 上一流程的二维码（语料触发/上一轮迎宾/送客，2 分钟窗口内），先
  // 恢复默认表情再进迎宾流程（引导语/变形），不等变形后新二维码上屏
  // 才被动替换；未显示时 no-op
  screen_->cancel("迎宾场景切换");

  // 迎宾词两级：1) 指令自带 texts（v0.20.0，App 指定）；2) 映射表
  // 迎宾点（welcome_point_code，默认 Y）条目 text（2026-10-19 需求：
  // 强制以映射表下发为准）。皆空 = 不播迎宾词
  std::vector<std::string> texts = event.texts;
  if (texts.empty() && !welcome_point_code_.empty()) {
    if (const auto map_text = point_map_->text_for_code(welcome_point_code_)) {
      texts = {*map_text};
      RCLCPP_INFO(
        get_logger(), "迎宾词取自映射表迎宾点 %s（%zu 字节）",
        welcome_point_code_.c_str(), map_text->size());
    }
  }
  if (texts.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "迎宾无播放词：指令未带 texts 且映射表迎宾点(%s)未配置 text，"
      "本次迎宾不播报（引导语/变形/二维码/回执不受影响）",
      welcome_point_code_.c_str());
  }
  // ---- 二维码阶段 1「准备」（缓存比对/下载 + 转码 + 推送，耗时大头
  // ~3.5-5s）：std::async 后台线程执行，与引导语播报/变形/重定位并行
  // 重叠，总时长 = max(引导语+变形+重定位, 准备)；worker 在上屏（阶段
  // 2）前 get() 收割，届时通常早已完成。上屏（阶段 2）留到变形完成后、
  // TTS 开播前——变形动画/变形结束恢复的表情会顶掉变形前上屏的内容
  //（PlayEmotion 单通道，新请求即打断），且迎宾词提及「扫描我屏幕上
  // 显示的二维码」，TTS 开播时屏幕必须已是二维码（2026-10-16 需求：
  // 二维码显示在开始迎宾 TTS 播报之前）。缺失/失败只影响显示，不阻断
  // 播报 ----
  // 线程安全：fetch 每次 curl 句柄独立、prepare 全程 subprocess，均无
  // 共享可变状态；screen_ 其余方法（play_prepared/cancel）仍只在 worker
  // 线程调用——worker 阻塞在本函数期间无人并发触碰 screen_，后台线程
  // 写入的 prepared_ 状态经 future.get() 建立 happens-before 后才被
  // worker 消费；robot_id_/password_ 先在 worker 上拷贝再按值捕获，
  // 避免与连接线程的凭据轮换（apply_credential）竞争
  bool qr_ready = false;      // 二维码内容已推送就绪（待变形完成后上屏）
  std::future<bool> qr_task;  // 无效 = 无需准备（未携带哈希且未配置默认图）
  if (event.qr_sha256.empty()) {
    // 未携带二维码哈希（含语料触发的本地注入）：显示默认二维码图片
    //（随包安装 config/QRCode.png）；未配置/文件缺失则不显示
    if (!default_qr_path_.empty()) {
      RCLCPP_INFO(
        get_logger(), "迎宾开始: operation_id=%s 迎宾词=%zu 条 二维码=默认图片(后台准备)",
        event.operation_id.c_str(), texts.size());
      qr_task = std::async(
        std::launch::async, [this] {
          const auto t0 = std::chrono::steady_clock::now();
          const auto prep = screen_->prepare(default_qr_path_, "default-qr");
          if (!prep.ok) {
            RCLCPP_WARN(
              get_logger(), "默认二维码准备失败(本次不显示,不影响播报): %s",
              prep.detail.c_str());
            return false;
          }
          const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
          RCLCPP_INFO(
            get_logger(), "默认二维码准备完成(后台并行): 耗时 %lldms",
            static_cast<long long>(ms));
          return true;
        });
    } else {
      RCLCPP_INFO(
        get_logger(), "迎宾开始: operation_id=%s 迎宾词=%zu 条 二维码=无（未携带 qr_sha256）",
        event.operation_id.c_str(), texts.size());
      // 本次无新二维码上屏，入口已撤屏（上方场景切换 cancel），无需再动
    }
  } else {
    RCLCPP_INFO(
      get_logger(), "迎宾开始: operation_id=%s 迎宾词=%zu 条 二维码=后台下载+准备(hash=%.12s…)",
      event.operation_id.c_str(), texts.size(), event.qr_sha256.c_str());
    // 凭据在 worker 上拷贝后按值捕获（见上「线程安全」）
    const std::string robot_id = robot_id_;
    const std::string password = password_;
    const std::string qr_sha256 = event.qr_sha256;
    qr_task = std::async(
      std::launch::async, [this, robot_id, password, qr_sha256] {
        const auto t0 = std::chrono::steady_clock::now();
        auto image_path = qr_cache_->cached_path(qr_sha256);
        const bool cache_hit = image_path.has_value();
        if (!cache_hit) {
          const auto fetch = qr_cache_->fetch(robot_id, password, qr_sha256);
          if (fetch.ok) {
            image_path = fetch.image_path;
          } else {
            RCLCPP_WARN(
              get_logger(),
              "迎宾二维码下载失败(本次不显示,不影响播报): hash=%.12s… %s",
              qr_sha256.c_str(), fetch.detail.c_str());
            return false;
          }
        }
        const auto prep = screen_->prepare(*image_path, qr_sha256);
        if (!prep.ok) {
          RCLCPP_WARN(
            get_logger(), "迎宾二维码准备失败(本次不显示,不影响播报): hash=%.12s… %s",
            qr_sha256.c_str(), prep.detail.c_str());
          return false;
        }
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - t0).count();
        RCLCPP_INFO(
          get_logger(), "迎宾二维码准备完成(后台并行): %s hash=%.12s… 耗时 %lldms",
          cache_hit ? "缓存命中" : "已下载", qr_sha256.c_str(),
          static_cast<long long>(ms));
        return true;
      });
  }
  // ---- 迎宾引导语（2026-10-17 现场需求）：先播 TTS 引导语（默认
  //「哈喽哈喽！电池满格，热情也满格」），播完再变形——先出声招呼、
  // 再起身变形的迎宾节奏。复用播报会话机制（麦克风静音/唤醒抑制/
  // trace 前缀全套）；worker 正阻塞在本函数，定时分支的 play_current
  // 到不了，受理后手动首播（与 tts 指令受理同款）。播完判定：TtsStatus
  // 终态回流提前解除（AgentMonitor 线程按 trace 前缀置 tts_early_stop_），
  // 字数估算兜底（BEST_EFFORT 丢事件）。受理失败不卡流程：告警后直接
  // 进变形（与变形失败不阻断语音同哲学），迎宾词照常 ----
  if (!welcome_lead_text_.empty()) {
    start_session(
      {welcome_lead_text_}, TTS_MODE_SEQUENCE, /*interval_seconds=*/0,
      /*loop_forever=*/false, "迎宾引导:" + event.operation_id,
      /*mute_mic=*/true);  // 引导语是迎宾业务一环：静音防打断
    const auto lead = play_current();
    if (lead.ok) {
      const bool terminal = wait_speech_done(welcome_lead_text_);
      RCLCPP_INFO(
        get_logger(), "迎宾引导语播完（%s），开始变形: 迎宾:%s",
        terminal ? "终态回流" : "估算兜底", event.operation_id.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "迎宾引导语受理失败(%s)，跳过等待直接变形（迎宾词不受影响）",
        lead.detail.c_str());
      cancel_session();
    }
  }

  // ---- 迎宾变形：引导语播完后发起（2026-10-17 需求：先播引导语、再
  // 变形、变形完播迎宾词）；已是人形零动作立即通过 ----
  transform_->ensure_biped("迎宾:" + event.operation_id);

  // ---- 等待变形完成后再开播（2026-10-15 需求：先变形、后人声）----
  // wait_done 必然在有限时间返回：变形线程内部每层等待都有上限（单次服务
  // 调用 3s / 动作到位总时限 / 等稳定 10s），运控不可达时更是立即放弃并
  // 照常播报。等待期间新指令在下行队列排队（worker 串行，处理完迎宾才
  // 轮到它们）；二维码的下载/转码/推送（实测 ~3.5s）已在后台线程与
  // 引导语播报、变形、重定位并行重叠，上屏只等变形。worker 与本函数
  // 同线程调用 transform_，无并发；shutdown 先 join worker 再回收变形线程
  if (transform_->in_flight()) {
    RCLCPP_INFO(
      get_logger(), "迎宾变形进行中，变形完成后开始播报: 迎宾:%s",
      event.operation_id.c_str());
  }
  if (transform_->wait_done()) {
    // 变形结束（成败均已在变形线程记日志），开始迎宾播报
    RCLCPP_INFO(
      get_logger(), "迎宾变形流程结束，开始迎宾播报: 迎宾:%s",
      event.operation_id.c_str());
  }
  // 变形后重定位（2026-10-18 现场需求）：迎宾变形真的变了形（180°
  // 转身把 AMCL 带偏）-> 发 set_pose（最新停留点）把定位拉回，再做
  // 二维码上屏与迎宾播报——机器人虽不走，但定位认知修正后下次导航
  // 才不会跑偏；已是人形零动作路径跳过；失败不阻断迎宾（补偿性）
  relocate_after_transform("迎宾:" + event.operation_id);

  // ---- 收割后台二维码准备（阶段 1）结果：引导语+变形+重定位窗口
  //（实测 >10s）覆盖准备耗时（~3.5s），此处通常零等待；准备各环节
  // 均有超时上限（curl 5s / ffmpeg / scp ConnectTimeout 5s），get()
  // 必然有限返回，不会挂死流程 ----
  if (qr_task.valid()) {
    qr_ready = qr_task.get();
  }
  // ---- 二维码阶段 2「上屏」：变形动画/结束表情已过去不会被顶掉，
  // 在 TTS 开播之前完成显示（2026-10-16 需求）。二维码常驻显示
  //（display_show_timeout_seconds 超时/新业务取消由 ScreenDisplay 与
  // worker 驱动）；上屏失败不阻断迎宾语音（屏幕与音频相互独立）----
  if (qr_ready) {
    const auto display = screen_->play_prepared("迎宾:" + event.operation_id);
    if (!display.ok) {
      RCLCPP_WARN(
        get_logger(), "迎宾二维码显示失败(不影响播报): %s",
        display.detail.c_str());
    }
  }

  // 迎宾业务占用：播报终态回流（TtsStatus）即刻释放（worker 的
  // tts_early 分支，2026-10-16）；welcome_flow_timeout_seconds（默认
  // 120s）仅作终态事件丢失时的兜底超时——到期退出迎宾占用，后续
  // navigate/tts 不再被 business_busy 拒绝；两条路径均不停播报（单轮
  // 自然播完）、不影响二维码常驻（120s 独立计时）。占用未释放前新业务
  // 互斥；释放后若播报仍在，新业务到达自然接管（cancel_session 终止
  // 残留播报）。
  // 无迎宾词（指令未带 texts 且映射表迎宾点未配 text，2026-10-19）：
  // 跳过播报会话与业务占用（PlaylistSession 空表不可播），二维码常驻
  // 与回执不受影响——迎宾流程主体（引导语/变形/重定位）已在上方同步
  // 完成，无需占用窗口。
  // 赋值在 start_session 之后（事务顺序：先立会话再立占用，拒绝窗口即刻生效）
  if (!texts.empty()) {
    start_session(
      std::move(texts), TTS_MODE_SEQUENCE, tour_speak_interval_seconds_,
      /*loop_forever=*/false, "迎宾:" + event.operation_id,
      /*mute_mic=*/true);  // 迎宾讲解：长内容，静音防打断
    business_ = Business::kWelcome;
    business_busy_until_ = std::chrono::steady_clock::now() +
      std::chrono::seconds(welcome_flow_timeout_seconds_);
  }

  // v0.24.0（P-038）：迎宾流程主体完成（变形已结束 + 二维码已上屏 + 播报
  // 会话已立）——回 scene.result(success)。texts 播报与二维码显示属本地
  // 执行细节不影响结果；变形失败/二维码缺失按设计降级继续，仍算完成
  //（scene_failed 场景不存在：本流程无「整体失败」路径）。本地注入
  //（语料触发）不发协议回执
  if (!event.local) {
    send_action_result(
      SCENE_RESULT_TYPE, event.operation_id, ACTION_RESULT_SUCCESS);
  }
}

// ---------------------------------------------------------------------------
// 播报伴随动作：TTS 受理成功时从轮换池取下一个预设动作触发。
// worker 线程同步调用（正常毫秒级返回；异常最多阻塞 motion_request_
// timeout_seconds，不影响 5s 周期的抑制续期）。失败仅告警不重试不阻断。
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::trigger_speech_motion()
{
  if (motion_preset_ids_.empty()) {
    return;  // 轮换池空：只播语音不做动作
  }
  const auto motion_id = motion_preset_ids_[
    motion_next_index_ % motion_preset_ids_.size()];
  ++motion_next_index_;
  trigger_preset_motion(motion_id);
}

// ---------------------------------------------------------------------------
// 触发指定预设动作（播报伴随动作公共段，worker 线程）：说话动作轮换池
//（trigger_speech_motion）与原地送客右手挥手（send_off_wave_motion_id，
// 2026-10-28 现场需求）共用。motion_id<=0 = 禁用；功能关闭/无客户端同
// 轮换池口径直接返回。失败仅告警不重试不阻断（语音才是主时间轴）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::trigger_preset_motion(int32_t motion_id)
{
  if (!motion_enabled_ || !motion_client_ || motion_id <= 0) {
    return;  // 功能关闭/动作禁用：只播语音不做动作
  }
  auto request = std::make_shared<aimdk_msgs::srv::SetMcPresetMotion::Request>();
  request->header.stamp = now();
  request->source = mc_source_name_.empty() ? "guide_pulse" : mc_source_name_;
  request->area.value = 0;  // NONE：预设动作按整体编排执行
  request->motion.value = motion_id;
  request->interrupt = true;  // 打断上一个伴随动作（语音才是主时间轴）

  const auto response = call_with_timeout(
    *motion_client_, request, motion_request_timeout_seconds_);
  if (!response) {
    RCLCPP_WARN(
      get_logger(), "播报动作 %d 无应答（跳过，播报不受影响）", motion_id);
    return;
  }
  const auto & result = response->response;
  if (result.header.code == 0) {
    RCLCPP_INFO(
      get_logger(), "播报动作已触发: id=%d task_id=%lu state=%d",
      motion_id, static_cast<unsigned long>(result.task_id),
      result.state.value);
  } else {
    RCLCPP_WARN(
      get_logger(), "播报动作 %d 被拒: code=%ld state=%d reason=%d（播报不受影响）",
      motion_id, static_cast<long>(result.header.code), result.state.value,
      result.state.reason);
  }
}

// ---------------------------------------------------------------------------
// 业务拒绝提示播报（worker 线程）：点位移交流程的各类「当场拒绝」共用
//（映射表空/未命中/受理被拒等）。不强抢进行中的其他播报（is_interrupted
// =false），失败仅告警不影响回执；text 空跳过（配置禁用）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::speak_hint(const std::string & text, const char * trace_id)
{
  if (text.empty()) {
    return;
  }
  TtsCommand command;
  command.text = text;
  command.priority_level = tts_priority_level_;
  command.priority_weight = tts_priority_weight_;
  command.is_interrupted = false;
  command.trace_id = trace_id;
  const auto speak = tts_->play(command);
  if (!speak.ok) {
    RCLCPP_WARN(
      get_logger(), "语音提示失败（不影响回执）: %s: %s",
      command.text.c_str(), speak.detail.c_str());
  }
}

// ---------------------------------------------------------------------------
// 导航途中随机播报（2026-10-21 现场需求，仅 worker 线程）：
// - pick_nav_chatter_text：按 nav_chatter_welcome_ 选池后随机取一条，
//   n>1 时在「其余条目」上均匀取样（[0,n-2] 取 j 后跳过上一条下标），
//   保证不与上一条连续重复——同一句短间隔连播现场观感差
// - nav_chatter_next_delay(text)：下一句等待 = text 播报时长估算
//   （受理出声延迟 lead + 每字ms×字数×scale，与尾部宽限/引导语等待
//   同款公式，可选封顶）+ 静默秒数——间隔语义 = 两句之间的静默，
//   不含播报本身的时间（2026-10-21 现场需求）
// ---------------------------------------------------------------------------
std::string MqttGuideBridgeNode::pick_nav_chatter_text()
{
  // 固定首句优先（2026-10-21 现场需求）：受理后第一次播报先播固定
  // 清道话术，播完清标志、回到随机池
  if (nav_chatter_first_pending_) {
    nav_chatter_first_pending_ = false;
    return nav_chatter_welcome_
      ? navigate_chatter_welcome_first_text_
      : navigate_chatter_first_text_;
  }
  // 按受理时定格的标志选池：去迎宾点用专用池（赶路/就位话术），
  // 其余导航用普通安抚池
  const auto & pool = nav_chatter_welcome_
    ? navigate_chatter_welcome_texts_
    : navigate_chatter_texts_;
  if (pool.empty()) {
    return {};
  }
  static thread_local std::mt19937 rng{std::random_device{}()};
  const std::size_t n = pool.size();
  std::size_t index = 0;
  if (n > 1) {
    std::uniform_int_distribution<std::size_t> dist(0, n - 2);
    const auto j = dist(rng);
    index = (j >= nav_chatter_last_index_) ? j + 1 : j;
  }
  nav_chatter_last_index_ = index;
  return pool[index];
}

std::chrono::milliseconds MqttGuideBridgeNode::nav_chatter_next_delay(
  const std::string & text) const
{
  int64_t speak_ms = suppress_tail_lead_ms_ +
    static_cast<int64_t>(estimate_speech_seconds(text, tts_speech_ms_per_char_)) *
    1000LL * suppress_tail_scale_;
  if (suppress_tail_max_ms_ > 0) {
    speak_ms = std::min(speak_ms, static_cast<int64_t>(suppress_tail_max_ms_));
  }
  return std::chrono::milliseconds(speak_ms) +
    std::chrono::seconds(navigate_chatter_silence_seconds_);
}

// ---------------------------------------------------------------------------
// 变形后重定位（worker 线程，2026-10-18 现场需求）：导航/迎宾前置变形
// 实际发生且成功（biped_transformed——发了变形动作并落位）时，同步发
// 不带航点的 {command: set_pose} 到 t1_slam_nav_ws——变形动作自带约
// 180° 转身（QUADRUPED_TO_BIPED_ROTATE），AMCL 定位被带偏；服务端用
// 最新停留点（最近一次导航结束实际停的位置）重设初值拉回。本身就是
// 人形（零动作路径）不发送（2026-10-21 现场需求：已是人形不 set_pose
// ——机器人没动，AMCL 持续跟踪未被带偏，强制重定位多此一举；已是
// 人形跳过路径视为成功照常上报，只有变形后的 set_pose 失败不上报）。
// 变形后发送前先等 relocate_after_transform_
// delay_seconds（默认 2s，2026-10-21 现场需求：落位余晃期抖动会干扰
// set_pose 初值，稳一稳再发；0 = 落位即发）。失败/被拒（无停留点/
// 导航栈未起/正在变形等）只记日志不阻断业务——补偿性重定位，尽力而为。
// 返回值：true = 成功（含未变形跳过路径）；false = 变形后尝试了但
// 失败/被拒——语料触发的 navigate.triggered 上报据此把关（只有重定位
// 成功才发消息给后端）
// ---------------------------------------------------------------------------
bool MqttGuideBridgeNode::relocate_after_transform(const std::string & source)
{
  if (!transform_->biped_transformed()) {
    RCLCPP_INFO(
      get_logger(), "变形后重定位跳过: 本次未发生形态切换（已是人形）（%s）",
      source.c_str());
    return true;  // 无形态切换 = 定位未被带偏，无需重定位，视为成功
  }
  // 变形刚落位先稳一稳再发（2026-10-21 现场需求）：落位余晃期间
  // 里程计抖动会干扰 set_pose 初值验收，等 relocate_after_transform_
  // delay_seconds 再发。分片等待：rclcpp 关停时数百毫秒内退出不卡
  // worker 收尾（与 welcome_prep 到达延迟变形同模式）
  if (relocate_after_transform_delay_seconds_ > 0) {
    RCLCPP_INFO(
      get_logger(), "变形后重定位: 变形刚落位，等 %ds 再发 set_pose（%s）",
      relocate_after_transform_delay_seconds_, source.c_str());
    const auto delay_deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(relocate_after_transform_delay_seconds_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < delay_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "变形后重定位: 发送 set_pose（最新停留点）到 t1_slam_nav_ws，等验收结果"
    "（通常零点几秒）（%s）",
    source.c_str());
  const auto outcome = nav_->set_pose_latest();
  if (outcome.ok) {
    RCLCPP_INFO(
      get_logger(), "变形后重定位完成: %s（%s）",
      outcome.detail.c_str(), source.c_str());
    return true;
  }
  RCLCPP_WARN(
    get_logger(), "变形后重定位失败(不影响后续流程): %s（%s）",
    outcome.detail.c_str(), source.c_str());
  return false;
}

std::string MqttGuideBridgeNode::point_display_name(
  const std::string & waypoint, const std::string & fallback) const
{
  for (const auto & entry : point_map_->points()) {
    if (entry.code == waypoint) {
      return entry.name;
    }
  }
  return fallback;
}

// ---------------------------------------------------------------------------
// 导航指令话术甄别（2026-10-22 现场需求）：App 下发 navigate 的 texts
// 若是小T带看复述话术（末尾「要去看X，没问题」类固定指令）则不是
// 讲解词——返回 true，调用方清除 texts 让到达讲解词回退映射表（映射
// 表权威，与迎宾词同口径）。甄别特征：navigate_phrase_template 的
// 前后缀（如「要去看」+「没问题」）同时出现即命中（{point} 部分是
// 开放的点位名，不逐点生成——前后缀组合特征已足够强），外加静态
// agent_monitor.navigate_phrases 的语料部分；模板与静态表均未配置时
// 不甄别（保持 v0.20.0 原行为：App 指定的 texts 照播）
// ---------------------------------------------------------------------------
bool MqttGuideBridgeNode::navigate_texts_are_agent_reply(
  const std::vector<std::string> & texts) const
{
  if (navigate_phrase_template_.empty() && navigate_route_phrases_.empty()) {
    return false;  // 无可匹配的指令特征（配置关闭）：不甄别
  }
  std::string joined;
  for (const auto & text : texts) {
    joined += text;
  }
  for (const auto & phrase : navigate_route_phrases_) {
    if (phrase_matches(joined, phrase)) {
      return true;
    }
  }
  static const std::string kToken = "{point}";
  const auto token_pos = navigate_phrase_template_.find(kToken);
  if (token_pos != std::string::npos) {
    const auto prefix = normalize_phrase_text(
      navigate_phrase_template_.substr(0, token_pos));
    const auto suffix = normalize_phrase_text(
      navigate_phrase_template_.substr(token_pos + kToken.size()));
    if (!prefix.empty() && !suffix.empty() &&
      normalize_phrase_text(joined).find(prefix) != std::string::npos &&
      normalize_phrase_text(joined).find(suffix) != std::string::npos)
    {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// 讲解模块（navigate，两个来源共用：MQTT 下行（App 触发，回
// navigate.result）与本地注入（AgentMonitor 命中 agent_monitor.
// navigate_phrases 语料，2026-10-17 车型导览：agent 复述「好的，您是想
// 了解下腾势 B5这款车吗，请稍等」-> point_name=「腾势 B5」经映射表
// 解析取 code 后导航，不发回执））：终止残留播报会话（防御，正常无
// 残留）-> 点位名解析 -> 发起导航 -> 到达后由 worker 开讲解。导览业务
// 自此占用（在途不靠估算窗口，以 TourRunner 导航状态为准；到达开讲解
// 时另设讲解窗口）。2026-10-21 现场需求：目标是迎宾点（code 命中
// welcome_point_code 或显示名含「迎宾」）时直接升级为迎宾准备流程
//（到达变四足收摄像头+播就位语，不播引导语、不带讲解词）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_navigate(const CommandEvent & event)
{
  // ---- 原地送客分流（2026-10-28 现场需求）：目标是送客点（point_name
  // 命中 send_off_point_code，默认 Z；2026-10-17 联调确认 App 下发的
  // 就是 code，原始名精确匹配即可）时不发起导航——Z 是虚拟点位（原地
  // 送客仪式），不查点位映射表也不发导航 goal，转 handle_inplace_send_off
  // 原地完成 变形双足 + 播送客语（伴随右手挥手）+ 送客二维码上屏 ----
  if (!send_off_point_code_.empty() && event.point_name == send_off_point_code_) {
    handle_inplace_send_off(event);
    return;
  }
  // ---- 原地播报分流（2026-10-28 现场需求，与 Z 点原地送客同款）：
  // 目标是播报点（point_name 命中 broadcast_point_code，默认 W）时同样
  // 不发起导航——W 是虚拟航点（原地播报），不发导航 goal，但语料须
  // 配置在点位映射表，转 handle_inplace_broadcast 原地完成 变形双足 +
  // 播报映射表 W 条目播放词（伴随说话动作）----
  if (!broadcast_point_code_.empty() && event.point_name == broadcast_point_code_) {
    handle_inplace_broadcast(event);
    return;
  }
  // 映射表为空（App 未推过表）＝ 配置缺失：直接拒绝本次导航——
  // 播报提示（现场可听到）+ 回执 navigate.result/failed:point_not_found，
  // 不启动导航也不占用业务（配好表后 App 重发即可）
  if (point_map_->size() == 0U) {
    RCLCPP_WARN(
      get_logger(), "导航被拒绝: 点位映射表为空（未配置），%s",
      event.local ? "不发协议回执（语料触发）" : "回执 failed/point_not_found");
    speak_hint(point_map_missing_text_, "point_map_missing");
    if (!event.local) {  // 语料触发不发回执，拒绝语音照播（现场能听到）
      send_action_result(
        NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_POINT_NOT_FOUND);
    }
    return;
  }
  // 点位解析（v0.25.0 P-039）：point_name 经映射表两级解析——先按
  // code 直接匹配（2026-10-17 联调确认 App 下发的就是 code，如「A」），
  // 未命中再 name->code（兜底兼容人话名如「比亚迪ATTO 1」-> A）。
  // 未命中直接拒绝（2026-10-17 现场需求：
  // 映射表是唯一权威，未命中=App 与 Robot 表不同步，透传航点名只会把
  // 错误下发给导航模块）——播报提示（含原始点位名，现场能听到缺哪个
  // 点）+ 回执 failed:point_not_found，不启动导航也不占用业务
  std::string waypoint;
  if (const auto code = point_map_->resolve(event.point_name)) {
    waypoint = *code;
    RCLCPP_INFO(
      get_logger(), "导航点位解析: 「%s」-> 航点 %s（映射表命中）",
      event.point_name.c_str(), waypoint.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(), "导航被拒绝: 「%s」未命中点位映射表（%zu 条），%s",
      event.point_name.c_str(), point_map_->size(),
      event.local ? "不发协议回执（语料触发）" : "回执 failed/point_not_found");
    speak_hint(
      replace_point_placeholder(point_map_unmatched_text_, event.point_name),
      "point_map_unmatched");
    if (!event.local) {  // 语料触发不发回执，拒绝语音照播（含缺失点位名）
      send_action_result(
        NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_POINT_NOT_FOUND);
    }
    return;
  }
  cancel_session();
  // 新业务到达 -> 迎宾二维码让位（前往展位，屏幕回默认表情）
  screen_->cancel("navigate");
  // ---- 导航出发播报（2026-10-18 现场需求）：收到导航指令无论是语音
  //（语料触发）还是 MQTT 消息，受理后先播「稍等，马上出发」再起身
  // 变形（先开口、立刻动——与变形音的叠加观感同哲学，不等播完）。
  // 空串 = 禁用；迎宾准备去迎宾点的导航不播（agent 已说「好嘞，马上
  // 前去迎宾点」，再说就重复——独立 handle_welcome_prep 链路天然跳过）----
  speak_hint(navigate_depart_text_, "navigate_depart");
  // ---- 导航前置变形（2026-10-18 现场需求）：先变形成人形再发起导航
  //——四足迎宾准备模式下有人叫导航，机器人要先站起来。已在人形零动作
  // 立即通过（常见路径毫秒级）；变形失败不阻断导航（McTransform 内部
  // 已记日志，与迎宾「变形失败照常播报」同哲学）。MQTT 下行与本地注入
  // 都走本函数，一处覆盖两个来源 ----
  transform_->ensure_biped("导航:" + event.operation_id);
  if (transform_->in_flight()) {
    RCLCPP_INFO(
      get_logger(), "导航前置变形进行中，变形完成后发起导航: %s",
      event.operation_id.c_str());
  }
  transform_->wait_done();  // worker 阻塞等待（与 handle_scene 迎宾变形同模式）
  // 双足形态门禁（2026-10-18 现场需求）：变形流程未确认「已是双足且
  // 运行中」则不得向 t1_slam_nav_ws 发 navigate goal——变形失败路径
  //（运控不可用/查形态失败/动作被拒/未到位/无已知转移路径）一律
  // 拒绝导航并回 failed/transform_failed，不带着未确认的姿态出发
  if (!transform_->biped_confirmed()) {
    RCLCPP_WARN(
      get_logger(), "导航被拒: 双足形态未确认（变形失败或运控不可用），"
      "不下发导航指令: %s", event.operation_id.c_str());
    speak_hint(navigate_transform_failed_text_, "navigate_transform_failed");
    if (!event.local) {  // 语料触发不发回执（Server 不认识本地 id）
      send_action_result(
        NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_TRANSFORM_FAILED);
    }
    return;
  }
  // 变形后重定位（2026-10-18 现场需求）：前置变形真的变了形（180°
  // 转身把 AMCL 带偏）-> 先发 set_pose（最新停留点）等终态再发导航
  // goal——set_pose 在途时发 go 会被「任务在跑」拒，必须等它做完；
  // 已是人形零动作路径跳过（2026-10-21 现场需求：已是人形不
  // set_pose）。失败不阻断导航（记日志
  // 继续出发），但成败交由调用方把关语料触发的上报：已是人形跳过
  // 路径视为成功照常上报，变形后 set_pose 失败不上报
  const bool relocate_ok =
    relocate_after_transform("导航:" + event.operation_id);
  // 引导语显示名（{point} 替换用）：映射表反查 code 对应的车型名，
  // 查不到用原始点位名（表结构上 code 必来自表内，反查失败仅防御）
  nav_point_display_ = point_display_name(waypoint, event.point_name);
  // ---- 迎宾点升级（2026-10-21 现场需求）：navigate 目标是迎宾点
  //（code 命中 welcome_point_code 或显示名含「迎宾」，与 worker 引导语
  // to_welcome 同口径）时直接升级为迎宾准备流程——到达变四足收摄像头、
  // 播就位语，不再按车型讲解（迎宾点不是车型，讲解词语义不通）；引导语
  // 同样不播（出发语「稍等，马上出发」已够）。置位放在所有早退路径之后
  //（解析失败/双足未确认均已 return），不会残留标志污染下次导航；回执
  // 与上报不受影响（nav_local_ 仍按来源，App 触发照回 navigate.result） ----
  if ((!welcome_point_code_.empty() && waypoint == welcome_point_code_) ||
    nav_point_display_.find("迎宾") != std::string::npos)
  {
    nav_welcome_prep_ = true;
    RCLCPP_INFO(
      get_logger(),
      "导航目标为迎宾点: 升级为迎宾准备流程（不播引导语、不带讲解词，"
      "到达变四足+播就位语）: point=%s operation_id=%s",
      waypoint.c_str(), event.operation_id.c_str());
  }
  // 讲解词甄别（2026-10-22 现场需求）：App 下发 navigate 带的 texts 若
  // 是小T带看复述话术（「要去看X，没问题」类指令）则不作讲解词——
  // 清除后到达讲解词回退映射表（映射表权威，与迎宾词同口径）；未命中
  // 指令特征（如 App 明确指定的随动播报）保持原行为照播
  std::vector<std::string> nav_texts = event.texts;
  if (!nav_texts.empty() && navigate_texts_are_agent_reply(nav_texts)) {
    RCLCPP_INFO(
      get_logger(),
      "导航 texts 命中带看指令话术，不作讲解词（到达讲解词走映射表）: %s",
      event.operation_id.c_str());
    nav_texts.clear();
  }
  RCLCPP_INFO(
    get_logger(), "导航开始: point=%s 讲解词=%zu 条 operation_id=%s",
    waypoint.c_str(), nav_texts.size(), event.operation_id.c_str());
  // v0.24.0（P-038）：记录 operation_id 供到达/失败/中止时回
  // navigate.result。单业务模型下在途导航不会被新 navigate 替换
  //（business_busy 先行拒绝并回 failed/busy），此处无需为旧导航回 aborted。
  // 语料触发（local，2026-10-17 车型导览）：终态不发协议回执
  nav_operation_id_ = event.operation_id;
  nav_local_ = event.local;
  nav_triggered_ = false;  // 是否已发 triggered（P-044 终态配对标志，下方按需置位）
  if (nav_welcome_prep_) {
    // 迎宾准备升级：不带讲解词（到达只做变形+就位语，讲解词无意义）
    tour_->start(waypoint, {});
  } else {
    tour_->start(waypoint, nav_texts);
  }
  // 新导航重置途中播报定时与池标志（受理事件按目标重新安排）
  nav_chatter_next_at_.reset();
  nav_chatter_welcome_ = false;
  nav_chatter_first_pending_ = false;
  business_ = Business::kTour;
  business_busy_until_.reset();  // 导航在途：占用由 tour_->navigating() 表达
  // v0.26.0（P-040）语音触发上报：语料触发的导航进入执行流程后回推
  // navigate.triggered 给 App（point_name 取映射表显示名；App 下发的
  // navigate 不产生本上报——App 已知自己触发了什么）。2026-10-19 现场
  // 需求：**只有前置重定位成功才发**（已是人形跳过路径视为成功——
  // 2026-10-21 需求不再强制 set_pose；变形后 set_pose 失败=不上报——
  // 定位未拉回的导航不该让 App 感知；导航本身照常出发）
  if (event.local) {
    if (relocate_ok) {
      send_nav_triggered(NAVIGATE_TRIGGERED_TYPE, nav_point_display_);
      nav_triggered_ = true;  // 终态（到达/失败/中止）据此配发 navigate.completed（P-044）
    } else {
      RCLCPP_WARN(
        get_logger(),
        "前置重定位失败，不发 navigate.triggered 上报（只有重定位成功"
        "才发，2026-10-19 需求），终态亦不发 navigate.completed: point=%s",
        nav_point_display_.c_str());
    }
  }
}

// ---------------------------------------------------------------------------
// 原地送客模块（2026-10-28 现场需求，仅 worker 线程）：App 下发
// navigate 去送客点 Z——Z 是虚拟点位，不导航，原地完成送客仪式：
//   1) 变形双足（站姿送客，与导航前置变形同模式阻塞 worker 等待）
//   2) 送客二维码上屏（先屏后声：感谢语提及「扫码收下选车小资料」；
//      navigate 载荷无 qr_sha256，用默认图 default_qr_path_，随包
//      QRCode.png 开箱即用；后台准备与变形并行重叠）
//   3) 播感谢语 send_off_done_text，伴随右手挥手（send_off_wave_
//      motion_id，默认 1001；触发点 = TTS 受理成功，与播报伴随动作
//      同时序，见 play_current 的 source 前缀分流）
// 双足未确认（运控不可用/变形被拒）与导航同口径拒绝：播
// navigate_transform_failed_text + 回 failed/transform_failed，二维码
// 后台准备收割丢弃不显示。回执：MQTT 来源在主体完成（二维码上屏 +
// 播报会话已立）后发 navigate.result(success)——与迎宾三场景路由的
// scene.result 同款完成语义；语料触发（local）不发协议回执。
// 感谢语播完业务即释放（终态回流/字数估算窗口兜底），二维码常驻时长
// 由屏幕 display_show_timeout_seconds 独立计时（新业务到达让位）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_inplace_send_off(const CommandEvent & event)
{
  const std::string source = "送客原地:" + event.operation_id;
  RCLCPP_INFO(
    get_logger(), "送客点导航分流: 原地送客（不导航）: %s", source.c_str());
  cancel_session();          // 终止残留播报会话（防御，与 navigate 同款）
  screen_->cancel(source);   // 新业务到达 -> 旧二维码让位（随后上屏送客二维码）
  // 二维码后台准备先于变形启动（~3.5s 与变形并行重叠；已是双足的
  // 零动作路径下 get() 等待即准备耗时，worker 阻塞与既有变形等待同模式）
  start_send_off_qr_prepare(event.qr_sha256);
  // 原地变形双足（送客站姿；已是双足零动作立即通过，常见路径毫秒级）
  transform_->ensure_biped(source);
  if (transform_->in_flight()) {
    RCLCPP_INFO(
      get_logger(), "送客变形双足进行中，变形完成后上屏二维码+播感谢语: %s",
      source.c_str());
  }
  transform_->wait_done();
  // 双足门禁（与 handle_navigate 同口径）：未确认双足不进入送客仪式——
  // 站不起来挥手/上屏都没意义，拒绝并回执，二维码后台准备收割不显示
  if (!transform_->biped_confirmed()) {
    RCLCPP_WARN(
      get_logger(), "原地送客被拒: 双足形态未确认（变形失败或运控不可用）: %s",
      source.c_str());
    finish_send_off_qr(false, "");  // 收割后台准备防泄漏（不显示）
    speak_hint(navigate_transform_failed_text_, "send_off_transform_failed");
    if (!event.local) {  // 语料触发不发协议回执（Server 不认识本地 id）
      send_action_result(
        NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_TRANSFORM_FAILED);
    }
    return;
  }
  // 二维码上屏（先屏后声；准备失败/未配置默认图只告警不阻断感谢语，
  // 屏幕与音频相互独立）
  finish_send_off_qr(true, source);
  // 播感谢语 + 伴随右手挥手（与到达式送客同款：短内容不静音，播完
  // 要能被唤醒迎接下一位顾客；空串 = 禁用播报（挥手随播报走不单独
  // 挥），到达即释放业务，二维码/回执照常）
  if (!send_off_done_text_.empty()) {
    const auto busy_until = estimate_busy_until(
      {send_off_done_text_}, tour_speak_interval_seconds_);
    start_session(
      {send_off_done_text_}, TTS_MODE_SEQUENCE,
      tour_speak_interval_seconds_, /*loop_forever=*/false, source,
      /*mute_mic=*/false);
    // 与到达式送客同款：导览业务占用进入播报窗口（终态回流即刻释放）
    business_ = Business::kTour;
    business_busy_until_ = busy_until;
  } else {
    business_ = Business::kNone;
    business_busy_until_.reset();
  }
  // MQTT 来源：主体完成（变形已确认 + 二维码已上屏 + 播报会话已立）
  // 回 navigate.result(success)；texts 播报与二维码显示属本地执行细节
  // 不影响结果（与 route_welcome_scene 的 scene.result 同语义）
  if (!event.local) {
    send_action_result(NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_SUCCESS);
  }
}

// ---------------------------------------------------------------------------
// 原地播报模块入口（2026-10-28 现场需求，与 Z 点原地送客同款分流）：
// App 下发 navigate 去播报点（point_name 命中 broadcast_point_code，
// 默认 W）——W 是虚拟航点（不导航、无需存在于航点表），但须配置在
// 点位映射表：播报语料 = 映射表该 code 条目的 text 播放词（App 推表
// 改词即生效；MQTT 消息只作触发，携带的 texts 不作播报内容）。
// 流程：表空/W 未命中表 -> 拒绝（与 navigate 同口径：语音提示 +
// failed/point_not_found，不变形不占业务）；命中但未配 text -> 本次
// 不播词不变形（与迎宾词「表未配 text 则不播词」同口径，回执照常
// success）；否则 变形双足（阻塞 worker 等变形，与导航前置变形同
// 模式）-> 播报映射表语料并伴随说话动作（source 前缀「播报原地:」
// 不在 play_current 的 送客挥手/重定位豁免 名单，天然走说话动作
// 轮换池 trigger_speech_motion，含 motion_repeat_seconds 动作续播）。
// 双足未确认与导航/送客同口径拒绝（播 navigate_transform_failed_text
// + failed/transform_failed）。MQTT 来源主体完成（变形已确认 + 播报
// 会话已立）回 navigate.result(success)，语料触发（local）不发回执
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_inplace_broadcast(const CommandEvent & event)
{
  const std::string source = "播报原地:" + event.operation_id;
  // 分流已精确命中 code（event.point_name == broadcast_point_code_）
  const std::string & code = event.point_name;
  RCLCPP_INFO(
    get_logger(), "播报点导航分流: 原地播报（不导航）: %s 点位=%s",
    source.c_str(), code.c_str());
  // 映射表为空 = 配置缺失：直接拒绝（与 handle_navigate 同口径）——
  // 播报提示（现场能听到）+ 回执 failed/point_not_found，不变形不占业务
  //（配好表后 App 重发即可）
  if (point_map_->size() == 0U) {
    RCLCPP_WARN(
      get_logger(), "原地播报被拒绝: 点位映射表为空（未配置），%s",
      event.local ? "不发协议回执（语料触发）" : "回执 failed/point_not_found");
    speak_hint(point_map_missing_text_, "point_map_missing");
    if (!event.local) {  // 语料触发不发回执，拒绝语音照播（现场能听到）
      send_action_result(
        NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_POINT_NOT_FOUND);
    }
    return;
  }
  // W 未配在映射表 = 配置缺失：拒绝同上（提示含点位名，现场能听到
  // 缺哪个点）
  if (!point_map_->resolve(code)) {
    RCLCPP_WARN(
      get_logger(), "原地播报被拒绝: 「%s」未命中点位映射表（%zu 条），%s",
      code.c_str(), point_map_->size(),
      event.local ? "不发协议回执（语料触发）" : "回执 failed/point_not_found");
    speak_hint(
      replace_point_placeholder(point_map_unmatched_text_, code),
      "point_map_unmatched");
    if (!event.local) {
      send_action_result(
        NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_POINT_NOT_FOUND);
    }
    return;
  }
  // 播报语料 = 映射表该条目 text（未配置 = 无可播内容，与迎宾词同口径：
  // 本次不播词；区别于迎宾的是 W 的主体就是播报，变形也一并跳过——
  // 站起来对着空气比划没意义，回执照常 success）
  const auto point_text = point_map_->text_for_code(code);
  if (!point_text || point_text->empty()) {
    RCLCPP_WARN(
      get_logger(), "原地播报: 映射表点位 %s 未配置播放词 text，本次不播报"
      "（不变形不占业务）: %s", code.c_str(), source.c_str());
    if (!event.local) {
      send_action_result(NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_SUCCESS);
    }
    return;
  }
  RCLCPP_INFO(
    get_logger(), "原地播报语料: 映射表点位 %s 播放词（%zu 字节）",
    code.c_str(), point_text->size());
  cancel_session();          // 终止残留播报会话（防御，与 navigate 同款）
  screen_->cancel(source);   // 新业务到达 -> 旧二维码让位（本流程不上屏新二维码）
  // 原地变形双足（讲话动作需要站立姿态；已是双足零动作立即通过，
  // 常见路径毫秒级）
  transform_->ensure_biped(source);
  if (transform_->in_flight()) {
    RCLCPP_INFO(
      get_logger(), "播报变形双足进行中，变形完成后播报映射表语料: %s",
      source.c_str());
  }
  transform_->wait_done();
  // 双足门禁（与 handle_navigate/handle_inplace_send_off 同口径）：未确认
  // 双足不进入播报——站不起来做讲话动作没意义，拒绝并回执
  if (!transform_->biped_confirmed()) {
    RCLCPP_WARN(
      get_logger(), "原地播报被拒: 双足形态未确认（变形失败或运控不可用）: %s",
      source.c_str());
    speak_hint(navigate_transform_failed_text_, "broadcast_transform_failed");
    if (!event.local) {  // 语料触发不发协议回执（Server 不认识本地 id）
      send_action_result(
        NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_FAILED,
        RESULT_REASON_TRANSFORM_FAILED);
    }
    return;
  }
  // 播报映射表语料 + 伴随说话动作（短内容不静音，播完要能被唤醒
  // 接待后续业务）
  const auto busy_until = estimate_busy_until(
    {*point_text}, tour_speak_interval_seconds_);
  start_session(
    {*point_text}, TTS_MODE_SEQUENCE,
    tour_speak_interval_seconds_, /*loop_forever=*/false, source,
    /*mute_mic=*/false);
  // 与原地送客同款：导览业务占用进入播报窗口（终态回流即刻释放）
  business_ = Business::kTour;
  business_busy_until_ = busy_until;
  // MQTT 来源：主体完成（变形已确认 + 播报会话已立）回
  // navigate.result(success)；texts 播报属本地执行细节不影响结果
  //（与 handle_inplace_send_off 同语义）
  if (!event.local) {
    send_action_result(NAVIGATE_RESULT_TYPE, event.operation_id, ACTION_RESULT_SUCCESS);
  }
}

// ---------------------------------------------------------------------------
// 迎宾准备模块（type=welcome_prep，2026-10-18 现场需求，原仅本地注入：
// AgentMonitor 命中 agent_monitor.welcome_prep_phrases 语料，如「整装
// 待发，即刻就位 需要先导航去Y点」，等 agent 说完该句再触发；2026-10-22
// 起 MQTT scene 三场景路由亦可到达，见 route_welcome_scene）。
// 送客模式（type=send_off，2026-10-21 现场需求，send_off=true）：命中
// agent_monitor.send_off_phrases 语料（如「热情相送，宾至如归」），
// 链路完全同款，仅到达后播感谢语 send_off_done_text（而非就位语）。
// 流程：点位名经映射表解析（同 navigate 口径，未命中拒绝+语音提示）
// -> 先确保人形（2026-10-18 导航前置：四足状态下要先站起来才能走）
// -> 发起导航（占导览业务；语音触发回推 navigate.triggered）-> worker
// 到达分支接管：变四足收摄像头 + 播就位语/感谢语。
// 返回 true = 导航已发起；false = 前置校验失败被拒（映射表空/点位未
// 命中/双足未确认）——MQTT 路由来源据此回 scene.result，本地注入
// 调用方不使用返回值
// ---------------------------------------------------------------------------
bool MqttGuideBridgeNode::handle_welcome_prep(const CommandEvent & event, bool send_off)
{
  // 日志/变形 source 前缀：送客与迎宾准备共用本函数，文案按模式区分
  const char * const label = send_off ? "送客" : "迎宾准备";
  // 映射表为空 = 配置缺失：直接拒绝（与 handle_navigate 同口径）——
  // 播报提示 + 不启动导航不占业务（配好表后重新触发即可）
  if (point_map_->size() == 0U) {
    RCLCPP_WARN(
      get_logger(), "%s被拒绝: 点位映射表为空（未配置）", label);
    speak_hint(point_map_missing_text_, "point_map_missing");
    return false;
  }
  // 点位解析（映射表唯一权威，与 handle_navigate 同口径）：先精确两级
  // 解析（code 优先、name 兜底），未命中再按名称关键词子串匹配
  //（2026-10-19 现场需求：语料点位部分写「迎宾」两字不绑定 code——
  // 查 name 含关键词的条目取其 code，App 换迎宾点 code 改表即生效，
  // 无需改 yaml 重启；写 code 如 Y 仍兼容，精确解析优先）。仍未命中
  // 直接拒绝——播报提示 + 不启动导航不占业务（语料触发不发协议回执）
  std::string waypoint;
  if (const auto code = point_map_->resolve(event.point_name)) {
    waypoint = *code;
    RCLCPP_INFO(
      get_logger(), "%s点位解析: 「%s」-> 航点 %s（映射表命中）",
      label, event.point_name.c_str(), waypoint.c_str());
  } else if (const auto code = point_map_->resolve_by_name_keyword(event.point_name)) {
    waypoint = *code;
    RCLCPP_INFO(
      get_logger(), "%s点位解析: 「%s」-> 航点 %s（名称关键词命中）",
      label, event.point_name.c_str(), waypoint.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(), "%s被拒绝: 「%s」未命中点位映射表（%zu 条）",
      label, event.point_name.c_str(), point_map_->size());
    speak_hint(
      replace_point_placeholder(point_map_unmatched_text_, event.point_name),
      "point_map_unmatched");
    return false;
  }
  cancel_session();
  screen_->cancel(send_off ? "send_off" : "welcome_prep");
  // 导航前置变形（2026-10-18 现场需求）：先变人形再导航（与 handle_
  // navigate 同款；四足迎宾准备状态下被再次叫去迎宾点也要能走；
  // 送客同——四足待命时要先站起来才能带顾客走）
  transform_->ensure_biped(std::string(label) + ":" + event.operation_id);
  if (transform_->in_flight()) {
    RCLCPP_INFO(
      get_logger(), "%s导航前置变形进行中，变形完成后发起导航: %s",
      label, event.operation_id.c_str());
  }
  transform_->wait_done();
  // 双足形态门禁（与 handle_navigate 同款）：未确认双足不下发导航指令。
  // 本地注入不发协议回执，拒绝语音照播（现场能听到）
  if (!transform_->biped_confirmed()) {
    RCLCPP_WARN(
      get_logger(), "%s被拒: 双足形态未确认（变形失败或运控不可用），"
      "不下发导航指令: %s", label, event.operation_id.c_str());
    speak_hint(navigate_transform_failed_text_, "welcome_prep_transform_failed");
    return false;
  }
  // 变形后重定位（与 handle_navigate 同款，2026-10-18 现场需求）：
  // 变了形先 set_pose（最新停留点）再发导航 goal；已是人形跳过
  //（2026-10-21 需求，不强制 set_pose）。成败同样把关语音触发上报
  //（与 handle_navigate 同口径）：已是人形跳过路径视为成功照常上报，
  // 变形后 set_pose 失败不上报
  const bool relocate_ok =
    relocate_after_transform(std::string(label) + ":" + event.operation_id);
  nav_point_display_ = point_display_name(waypoint, event.point_name);
  RCLCPP_INFO(
    get_logger(),
    "%s导航开始: point=%s operation_id=%s（到达后变四足收摄像头，再播%s）",
    label, waypoint.c_str(), event.operation_id.c_str(),
    send_off ? "感谢语" : "就位语");
  nav_operation_id_ = event.operation_id;
  nav_local_ = true;           // 本地注入：到达/失败不发协议回执
  nav_triggered_ = false;      // P-044 终态配对标志，下方按 relocate_ok 置位
  // 到达分支据此走四足变形而非车型讲解（两标志互斥：送客播感谢语）
  nav_welcome_prep_ = !send_off;
  nav_send_off_ = send_off;
  // ---- 送客二维码后台准备（2026-10-22 现场需求）：感谢语提及「扫码
  // 收选车小资料」，到达变形完成后上屏（到达分支 finish_send_off_qr
  // 收割，先屏后声）。阶段 1（缓存比对/下载 + 转码 + 推送，~3.5s）
  // 与导航并行重叠，导航几十秒到达时通常早已完成——准备公共段已抽
  // start_send_off_qr_prepare（2026-10-28 起原地送客共用）----
  if (send_off) {
    start_send_off_qr_prepare(event.qr_sha256);
  }
  tour_->start(waypoint, {});  // 不带讲解词（到达只做变形+就位语/感谢语）
  // 重置定时与池标志（受理事件按目标重新安排途中播报，2026-10-21）
  nav_chatter_next_at_.reset();
  nav_chatter_welcome_ = false;
  nav_chatter_first_pending_ = false;
  business_ = Business::kTour;
  business_busy_until_.reset();  // 导航在途：占用由 tour_->navigating() 表达
  // 语音触发上报（与语料触发 navigate 同面）：App 侧可知机器人正去迎宾点
  //（2026-10-19 需求：只有变形后重定位成功才发，与 handle_navigate 同
  // 口径；2026-10-22 起 MQTT scene 路由可达本函数——MQTT 来源不发
  // triggered/completed：trigger 枚举只有 voice，App 自己下发的指令
  // 自知触发来源，重复上报反成噪音，与 handle_navigate 的 event.local
  // 门禁同口径）
  if (event.local && relocate_ok) {
    send_nav_triggered(NAVIGATE_TRIGGERED_TYPE, nav_point_display_);
    nav_triggered_ = true;  // 到达/被停据此配发 navigate.completed（P-044）
  } else if (!relocate_ok) {
    RCLCPP_WARN(
      get_logger(),
      "前置重定位失败，不发 navigate.triggered 上报（%s，只有"
      "重定位成功才发，2026-10-19 需求），终态亦不发 navigate.completed: point=%s",
      label, nav_point_display_.c_str());
  }
  return true;  // 导航已发起（到达后的变形/播报由 worker 到达分支接管）
}

// ---------------------------------------------------------------------------
// 送客二维码后台准备·阶段 1（2026-10-22 现场需求抽公共段，仅 worker
// 线程调用）：缓存比对/下载 + 转码 + 推送（~3.5s）异步启动，与主流程
//（到达式送客的导航 / 原地送客的变形）并行重叠，finish_send_off_qr
// 收割。内容来源：qr_sha256 优先（MQTT scene 载荷快照，缓存命中免下载
// ——navigate 载荷无此字段，原地送客恒走默认图分支）；未带则默认图片
// default_qr_path_（未配置不显示）。线程模型同 handle_scene 的 qr_task：
// 凭据先拷贝再按值捕获（避免与连接线程凭据轮换竞争），prepare 全程
// subprocess 不触碰 ROS 客户端，跨线程 happens-before 由 future.get() 建立
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::start_send_off_qr_prepare(const std::string & qr_sha256)
{
  const std::string robot_id = robot_id_;
  const std::string password = password_;
  send_off_qr_task_ = std::async(
    std::launch::async, [this, robot_id, password, qr_sha256]() {
      std::string image_path;
      if (!qr_sha256.empty()) {
        auto cached = qr_cache_->cached_path(qr_sha256);
        if (cached.has_value()) {
          image_path = *cached;
        } else {
          const auto fetch = qr_cache_->fetch(robot_id, password, qr_sha256);
          if (!fetch.ok) {
            RCLCPP_WARN(
              get_logger(),
              "送客二维码下载失败(本次不显示,不影响流程): hash=%.12s… %s",
              qr_sha256.c_str(), fetch.detail.c_str());
            return false;
          }
          image_path = fetch.image_path;
        }
      } else if (!default_qr_path_.empty()) {
        image_path = default_qr_path_;
      } else {
        RCLCPP_INFO(
          get_logger(), "送客未携带 qr_sha256 且未配置默认二维码，本次不显示");
        return false;
      }
      const auto prep = screen_->prepare(
        image_path, qr_sha256.empty() ? "send-off" : qr_sha256);
      if (!prep.ok) {
        RCLCPP_WARN(
          get_logger(), "送客二维码准备失败(本次不显示,不影响流程): %s",
          prep.detail.c_str());
        return false;
      }
      return true;
    });
}

// ---------------------------------------------------------------------------
// 送客二维码收敛（2026-10-22 现场需求，仅 worker 线程）：收割
// send_off_qr_task_ 后台准备结果；show=true 且准备成功时 play_prepared
// 上屏（到达变四足完成后、感谢语开播前——感谢语提及「扫码收选车
// 小资料」，先屏后声），show=false 仅收割防泄漏（导航失败/停止路径，
// 不显示）。future 无效（非送客/未启动/已收割）时 no-op，幂等可安全
// 重入；上屏失败只告警不阻断感谢语（屏幕与音频相互独立）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::finish_send_off_qr(bool show, const std::string & source)
{
  if (!send_off_qr_task_.valid()) {
    return;
  }
  const bool ready = send_off_qr_task_.get();
  if (!show || !ready) {
    return;
  }
  const auto display = screen_->play_prepared(source);
  if (!display.ok) {
    RCLCPP_WARN(
      get_logger(), "送客二维码显示失败(不影响感谢语): %s",
      display.detail.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "送客二维码已上屏: %s", source.c_str());
}

// ---------------------------------------------------------------------------
// 音频会话（音频单会话策略：任一新会话替换旧会话并从头开始）。
// mute_mic（2026-10-17 现场需求）：仅迎宾讲解与导航到达讲解词需要
// 静音麦克风（长内容，防打断收益大）；其余播报（App tts / 重定位等
// 短播报）不掐麦克风，唤醒抑制照常武装承担防自唤醒保护
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::start_session(
  std::vector<std::string> texts, const std::string & mode, int interval_seconds,
  bool loop_forever, const std::string & source, bool mute_mic)
{
  if (session_) {
    RCLCPP_INFO(
      get_logger(), "替换播报会话: old=%s new=%s",
      session_source_.c_str(), source.c_str());
  }
  // 业务占用窗口由调用方管理（handle_command/handle_scene/到达讲解处置，
  // tts.stop/导航终态清理），本函数不触碰 business_*
  // 单轮播报（tts/迎宾/讲解）期间抑制唤醒
  //（看门狗续期，见头文件说明）；循环播报（PlaylistSession 仍保留该能力，
  // 当前无调用方）不抑制——被唤醒进入对话属可接受业务行为。
  // 替换会话时：旧单轮会话的续期由新会话类型接管（循环=停止并快速恢复）
  if (!loop_forever) {
    suppress_renew_active_ = true;
    suppress_stop_at_.reset();
    suppress_next_at_ = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(suppress_renew_interval_ms_);
    if (mute_mic) {
      set_mic_muted(true);  // 先静音（SSH 慢）：TTS 发出时无音频可打断
      // 静音成功 = 无音频即无唤醒，唤醒抑制整套跳过（无服务调用与续期
      // 日志）；静音失败才武装，作为降级保护全量生效（arm+续期+收尾）
      suppress_armed_ = !mic_muted_.load();
    } else {
      // 不静音麦克风（mute_mic=false）：唤醒抑制全量武装（arm+续期+
      // 收尾），只拦唤醒词识别、不影响麦克风收音。先幂等释放旧会话
      // 可能遗留的静音（如静音中的讲解被本类播报替换）
      set_mic_muted(false);
      suppress_armed_ = true;
    }
    if (suppress_armed_) {
      suppress_wakeup(static_cast<std::uint32_t>(suppress_ttl_ms_));
    }
  } else if (suppress_renew_active_) {
    // 单轮被循环会话替换：停续期，刷短 TTL 立即解除（新会话自行抑制）
    suppress_renew_active_ = false;
    suppress_stop_at_.reset();
    set_mic_muted(false);
    if (suppress_armed_) {
      suppress_wakeup(static_cast<std::uint32_t>(suppress_tail_flush_ms_));
    }
    suppress_armed_ = false;
  }
  session_.emplace(
    std::move(texts), mode, interval_seconds, loop_forever);
  session_source_ = source;
  { // 供 notify_tts_terminal 前缀匹配（跨线程读，queue_mutex_ 保护）
    std::lock_guard<std::mutex> lock(queue_mutex_);
    tts_trace_prefix_ = source;
    tts_early_stop_ = false;
  }
  // 新会话立即首播：重置下一跳时刻，避免沿用被替换会话的排程
  //（worker 在下一个循环判定 now >= session_next_at_ 即播）
  session_next_at_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    get_logger(), "播报会话开始: %s mode=%s 条数=%zu interval=%ds %s",
    source.c_str(), mode.c_str(), session_->size(), interval_seconds,
    loop_forever ? "(循环)" : "(单轮)");
}

// ---------------------------------------------------------------------------
// 唤醒抑制原子调用：SetAgentSuppression(WAKEUP, ttl)。失败仅告警不影响
// 播报（抑制是尽力而为的防打断增强）。worker 线程调用。
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::suppress_wakeup(std::uint32_t ttl_ms)
{
  if (!suppress_wakeup_ || !suppress_client_) {
    return;
  }
  auto request = std::make_shared<aimdk_msgs::srv::SetAgentSuppression::Request>();
  request->header.header.stamp = now();
  request->suppression_type =
    aimdk_msgs::srv::SetAgentSuppression::Request::SUPPRESSION_TYPE_WAKEUP;
  request->ttl_ms = ttl_ms;

  auto future = suppress_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    suppress_client_->remove_pending_request(future);
    RCLCPP_WARN(
      get_logger(), "唤醒抑制请求超时（播报仍将进行，可能被唤醒打断）");
    return;
  }
  const auto response = future.get();
  if (response->header.header.code == 0) {
    // INFO 级（原 DEBUG）：抑制是否真的在发需可观测——真机排查
    // "播报被唤醒打断"时确认抑制调用链（2026-10-15 排查启用）
    RCLCPP_INFO(
      get_logger(), "唤醒抑制: ttl=%ums（续期=%s）",
      ttl_ms, suppress_renew_active_ ? "进行中" : "停止");
  } else {
    RCLCPP_WARN(
      get_logger(), "唤醒抑制被拒: code=%ld msg=%s（播报仍将进行）",
      response->header.header.code, response->header.message.c_str());
  }
}

// ---------------------------------------------------------------------------
// 运控板麦克风硬抑制：SSH 执行 pactl set-source-mute（PulseAudio
// 默认音源静音/恢复）。
// 静音 = 播报期间无音频进入唤醒/对话链路（弱打断与对话轮强打断皆无源）。
// 幂等（状态已到位直接返回）；静音失败仅告警（退回唤醒抑制保护），
// 恢复失败立即重试一次（失败则机器人失聪，ERROR 提示人工 pactl 恢复）。
// 仅 worker 线程与关停路径（worker 已 join 后）调用，无并发
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::set_mic_muted(bool mute)
{
  if (!mic_mute_enabled_ || mic_muted_.load() == mute) {
    return;
  }
  // PulseAudio 标准工具：静音默认音源（@DEFAULT_SOURCE@ 由 pactl 解析，
  // 不依赖厂商脚本 aw89xxx_mute.sh，目标机器未装脚本也可用）
  const std::string cmd = "sshpass -p '" + mic_mute_pass_ + "' ssh "
    "-o StrictHostKeyChecking=accept-new -o ConnectTimeout=3 " + mic_mute_target_ +
    " 'pactl set-source-mute @DEFAULT_SOURCE@ " + std::string(mute ? "true" : "false") +
    "' > /dev/null 2>&1";
  int rc = std::system(cmd.c_str());
  if (rc != 0 && !mute) {
    rc = std::system(cmd.c_str());  // 恢复失败=失聪：立即重试一次
  }
  if (rc != 0) {
    RCLCPP_ERROR(
      get_logger(), "运控板麦克风%s失败: %s（rc=%d%s）",
      mute ? "静音" : "恢复", mic_mute_target_.c_str(), rc,
      mute ? "" : "；请人工执行 pactl set-source-mute @DEFAULT_SOURCE@ false");
    return;  // 状态未变更
  }
  mic_muted_.store(mute);
  RCLCPP_INFO(
    get_logger(), "运控板麦克风已%s", mute ? "静音（播报防打断）" : "恢复");
}

// ---------------------------------------------------------------------------
// 播报终态回流（AgentMonitor 回调线程）：本会话 trace 前缀命中（agent
// 回流的 item_id 带后缀，前缀=会话来源标识）则置提前停止并唤醒 worker
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::notify_tts_terminal(const std::string & trace_id)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (!tts_trace_prefix_.empty() && trace_id.rfind(tts_trace_prefix_, 0) == 0) {
    tts_early_stop_ = true;
    queue_cv_.notify_all();
  }
}

// 播会话当前条并推进/排程；播完（单轮模式）会话自行结束
TtsClient::Result MqttGuideBridgeNode::play_current()
{
  if (!session_) {
    return {};
  }
  const std::string text = session_->current();
  const std::size_t index = session_->index();
  TtsCommand command;
  command.text = text;
  command.priority_level = tts_priority_level_;
  command.priority_weight = tts_priority_weight_;
  // is_interrupted=false：①SDK 仲裁侧同优先级排队不强抢（L7 本身已压制
  // 唤醒级 L6 播报）；②agent 唤醒强打断（strong_interrupt）以会话的
  // interruptible 快照决策，实测 true 时唤醒词直接 request_stop 掐断
  // 迎宾/讲解播报（绕过优先级仲裁），false 时不触发强打断（真机日志验证）
  command.is_interrupted = false;
  command.trace_id = session_source_;    // 来源标识透传给 SDK 作链路对账

  const auto result = tts_->play(command);
  if (result.ok) {
    // 本句语音时长估算（lead + 字数×scale，未封顶）：动作续播窗口与
    // 尾部宽限共用同一公式
    const int64_t speech_ms = suppress_tail_lead_ms_ +
      static_cast<int64_t>(estimate_speech_seconds(text, tts_speech_ms_per_char_)) *
      1000LL * suppress_tail_scale_;
    // 原地送客感谢语（2026-10-28 现场需求）：伴随右手挥手而非说话动作
    // 池；重定位不触发动作的既有豁免保持不变。原地播报（W 点，前缀
    // 「播报原地:」）不在豁免名单——走本分支说话动作池（2026-10-28
    // 现场需求：播报映射表语料伴随讲话动作）
    if (session_source_.rfind("送客原地:", 0) == 0) {
      trigger_preset_motion(send_off_wave_motion_id_);
    } else if (session_source_.rfind("重定位:", 0) != 0) {
      trigger_speech_motion();
      // 动作续播窗口（2026-10-28 现场需求：讲解句长动作短，手部动作
      // 太短）：本句播完前每隔 motion_repeat_seconds 秒由 worker 续触
      // 轮换池下一个动作（interrupt=true 打断旧动作无缝接续），窗口
      // 到期/终态回流/会话取消即停手
      if (motion_repeat_seconds_ > 0) {
        const auto speak_now = std::chrono::steady_clock::now();
        motion_renew_until_ =
          speak_now + std::chrono::milliseconds(speech_ms);
        motion_renew_next_at_ =
          speak_now + std::chrono::seconds(motion_repeat_seconds_);
      }
    }
    const bool more = session_->advance();
    if (!more) {
      // 单轮模式一轮播完（受理完最后一条，语音仍在播）：会话结束，续期
      // 转入尾部宽限——lead + 字数估算×scale（参数可 yaml 校准），
      // 到点停续期并立即解除唤醒抑制
      RCLCPP_INFO(
        get_logger(), "播报[轮%zu %zu/%zu] ok（单轮播完，会话结束: %s）",
        session_->round_no(), index + 1, session_->size(),
        session_source_.c_str());
      if (suppress_renew_active_) {
        int64_t tail_ms = speech_ms;
        if (suppress_tail_max_ms_ > 0) {
          tail_ms = std::min(tail_ms, static_cast<int64_t>(suppress_tail_max_ms_));
        }
        suppress_stop_at_ = std::chrono::steady_clock::now() +
          std::chrono::milliseconds(tail_ms);
      }
      // 续播窗口保留（最后一句语音仍在播），到期或终态回流时停手
      session_.reset();
      session_source_.clear();
      return result;
    }
    // interval 语义 = 每两条内容之间的间隔：PlayTts 异步返回≠播完，
    // 按语音时长估算叠加 interval 再排下一条
    session_next_at_ = std::chrono::steady_clock::now() +
      std::chrono::seconds(session_->next_delay_seconds());
    RCLCPP_INFO(
      get_logger(), "播报[轮%zu %zu/%zu] ok text_len=%zu priority=%d weight=%d (%lldms)",
      session_->round_no(), index + 1, session_->size(), text.size(),
      tts_priority_level_, tts_priority_weight_,
      static_cast<long long>(result.elapsed_ms));
  } else {
    // 被 SDK 仲裁拒绝（通道被更高优先级占用）或受理超时：
    // 短间隔重试同一条抢占空闲窗口（索引不推进）
    session_next_at_ = std::chrono::steady_clock::now() +
      std::chrono::seconds(tts_retry_seconds_);
    RCLCPP_WARN(
      get_logger(), "播报[轮%zu %zu/%zu] 失败(%s)，%ds后重试",
      session_->round_no(), index + 1, session_->size(), result.detail.c_str(),
      tts_retry_seconds_);
  }
  return result;
}

void MqttGuideBridgeNode::cancel_session()
{
  if (session_) {
    RCLCPP_INFO(get_logger(), "终止播报会话: %s", session_source_.c_str());
  }
  session_.reset();
  session_source_.clear();
  // 会话取消（语音已被打断）：动作续播窗口一并关闭，手立刻停
  motion_renew_until_.reset();
  // 抑制随会话一并终止（2026-10-17 生命周期排查）：尾部宽限
  // suppress_stop_at_ 仅在「最后一条受理成功」时设置，会话被取消后续期
  // 链没有任何收尾条件——worker 的抑制分支将永续重发 SetAgentSuppression
  //（唤醒被永久抑制，现场喊不醒机器人），若被取消的会话 mute_mic=true
  // 则麦克风还残留静音。触发路径：重定位成功播报（finish_relocate 所建、
  // 不占业务）进行中来 navigate / 讲解估算窗口到期后的残留会话被
  // handle_navigate 的防御性 cancel 清掉。tts.stop 分支在此之前已自行
  // 清理（此处幂等跳过）；start_session 替换会话不经本函数，新会话的
  // 抑制由其自行武装，不受影响
  if (suppress_renew_active_) {
    suppress_renew_active_ = false;
    suppress_stop_at_.reset();
    set_mic_muted(false);
    if (suppress_armed_) {
      suppress_wakeup(static_cast<std::uint32_t>(suppress_tail_flush_ms_));
    }
    suppress_armed_ = false;
  }
  // 业务占用由调用方清理（tts.stop 停一切、handle_navigate 转入导览占用）
}

// ---------------------------------------------------------------------------
// 等待单条播报结束（worker 线程，迎宾引导语专用）：AgentMonitor 回流的
// TtsStatus 终态（END/STOP/ERROR，trace 前缀命中时置 tts_early_stop_）
// 提前唤醒；字数估算兜底 BEST_EFFORT 丢事件（公式与尾部宽限同款：
// lead + 每字ms×字数×scale，可选封顶）。返回是否终态回流（false =
// 估算到期或关停放行——语音大概率已播完/流程将退出，照常放行不卡死）。
// ---------------------------------------------------------------------------
bool MqttGuideBridgeNode::wait_speech_done(const std::string & text)
{
  int64_t wait_ms = suppress_tail_lead_ms_ +
    static_cast<int64_t>(estimate_speech_seconds(text, tts_speech_ms_per_char_)) *
    1000LL * suppress_tail_scale_;
  if (suppress_tail_max_ms_ > 0) {
    wait_ms = std::min(wait_ms, static_cast<int64_t>(suppress_tail_max_ms_));
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(wait_ms);
  std::unique_lock<std::mutex> lock(queue_mutex_);
  const bool woken = queue_cv_.wait_until(lock, deadline, [this]() {
      return tts_early_stop_ || !running_.load();
    });
  // 消费终态标志：引导语不占业务，误留给 worker 的 tts_early 分支虽被
  // 守卫挡住（该分支语义是释放播报类业务占用），清掉更干净；后续迎宾
  // 词会话的终态事件在其开播后自行回流，互不干扰
  if (woken && tts_early_stop_) {
    tts_early_stop_ = false;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// 上行发布：tts.result / navigate.result / scene.result 回执 +
// presence / heartbeat
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::send_tts_result(
  const std::string & operation_id, const std::string & outcome,
  const std::string & reason)
{
  publish_receipt(
    operation_id, build_tts_result_event(robot_id_, operation_id, outcome, reason));
}

void MqttGuideBridgeNode::send_action_result(
  const std::string & result_type, const std::string & operation_id,
  const std::string & outcome, const std::string & reason)
{
  publish_receipt(
    operation_id,
    build_action_result_event(robot_id_, result_type, operation_id, outcome, reason));
}

void MqttGuideBridgeNode::publish_receipt(
  const std::string & operation_id, std::optional<TtsResultReceipt> receipt)
{
  std::string envelope;
  {
    // 契约：同一 operation_id 必须重发同一回执（同 event_id），
    // 因此首次生成后缓存信封原文，重发原样复用。仅 worker 线程访问，免锁
    const auto it = receipt_cache_.find(operation_id);
    if (it != receipt_cache_.end()) {
      envelope = it->second;
    } else {
      if (!receipt) {
        RCLCPP_ERROR(
          get_logger(), "回执参数非法: operation_id=%s（不发送）",
          operation_id.c_str());
        return;
      }
      envelope = receipt->envelope_json;
      // 有界缓存（与 dedupe 同容量）：超限时淘汰一半，保留当前活跃指令
      if (receipt_cache_.size() > static_cast<std::size_t>(dedupe_capacity_)) {
        std::size_t to_remove = receipt_cache_.size() / 2;
        for (auto rit = receipt_cache_.begin();
          rit != receipt_cache_.end() && to_remove > 0;)
        {
          if (rit->first != operation_id) {
            rit = receipt_cache_.erase(rit);
            --to_remove;
          } else {
            ++rit;
          }
        }
      }
      receipt_cache_.emplace(operation_id, envelope);
    }
  }
  // 不再单独打回执日志：publish_json 的"MQTT 发出"行已含完整 payload
  publish_json(up_commands_topic_, envelope, 1, false);
}

void MqttGuideBridgeNode::send_nav_triggered(
  const std::string & triggered_type, const std::string & point_name)
{
  // v0.26.0（P-040）：仅语料触发（event.local）调用方才走到这里——
  // App 下发指令不产生本上报（App 已知自己触发了什么），本地拒绝
  //（busy/point_not_found）也不发布，只表达「已开始执行」
  auto receipt = build_nav_triggered_event(
    robot_id_, triggered_type, point_name, TRIGGER_VOICE);
  if (!receipt) {
    RCLCPP_ERROR(
      get_logger(), "语音触发上报参数非法: type=%s point=%s（不发送）",
      triggered_type.c_str(), point_name.c_str());
    return;
  }
  publish_json(up_commands_topic_, receipt->envelope_json, 1, false);
}

void MqttGuideBridgeNode::send_nav_completed(
  const std::string & completed_type, const std::string & point_name,
  const std::string & outcome, const std::string & reason)
{
  // v0.30.0（P-044）：仅语料触发且已发过 *.triggered（nav_triggered_）
  // 的调用方才走到这里——App 下发指令的终态走 *.result 回执，未发
  // triggered 的动作不补孤儿 completed（一一对应，按时间序关联）
  auto receipt = build_nav_completed_event(
    robot_id_, completed_type, point_name, TRIGGER_VOICE, outcome, reason);
  if (!receipt) {
    RCLCPP_ERROR(
      get_logger(), "语音动作终态上报参数非法: type=%s point=%s outcome=%s（不发送）",
      completed_type.c_str(), point_name.c_str(), outcome.c_str());
    return;
  }
  publish_json(up_commands_topic_, receipt->envelope_json, 1, false);
}

void MqttGuideBridgeNode::publish_presence(const std::string & state, const std::string & reason)
{
  // 严格按契约 presence.schema.json（additionalProperties=false）
  json payload = {
    {"spec_version", "1.0"},
    {"robot_id", robot_id_},
    {"session_id", session_id_},
    {"boot_id", boot_id_},
    {"state", state},
    {"reason", reason},
    {"occurred_at", nullptr},
  };
  if (state == "online") {
    payload["client_version"] = client_version_;
    payload["protocol_versions"] = json::array({"1.0"});
    payload["capabilities"] = json::array({"text"});
  }
  // 契约：connection_lost 由 broker 代发遗嘱时 occurred_at=null，本进程直发时带时间戳
  if (reason != "connection_lost") {
    payload["occurred_at"] = now_rfc3339();
  }
  publish_json(presence_topic_, payload.dump(), 1, true);
}

void MqttGuideBridgeNode::publish_heartbeat()
{
  // 未完成上线通告前不发心跳（心跳依赖有效 session_id，且语义上先 online）
  if (!subscribed_.load()) {
    return;
  }
  const auto uptime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started_at_).count();
  // 严格按契约 heartbeat.schema.json；本节点无硬件遥测，
  // 电源/网络如实填 unknown，音频输出可用（PlayTts 正常即播报）
  json payload = {
    {"spec_version", "1.0"},
    {"robot_id", robot_id_},
    {"session_id", session_id_},
    {"boot_id", boot_id_},
    {"sequence", ++heartbeat_sequence_},
    {"occurred_at", now_rfc3339()},
    {"uptime_ms", uptime_ms},
    {"client_version", client_version_},
    {"power", {
      {"source", "unknown"},
      {"charging", false},
    }},
    {"network", {
      {"type", "unknown"},
    }},
    {"health", {
      {"overall", "ok"},
      {"audio_input", "unknown"},
      {"audio_output", "ok"},
      {"storage", "unknown"},
    }},
  };
  publish_json(heartbeat_topic_, payload.dump(), 0, false);
}

bool MqttGuideBridgeNode::publish_json(
  const std::string & topic, const std::string & payload, int qos, bool retained)
{
  if (!client_ || !client_->is_connected()) {
    RCLCPP_WARN(get_logger(), "MQTT 未连接，丢弃上行报文: %s", topic.c_str());
    return false;
  }
  try {
    mqtt::properties props{
      mqtt::property(mqtt::property::PAYLOAD_FORMAT_INDICATOR, 1),
      mqtt::property(mqtt::property::CONTENT_TYPE, "application/json"),
    };
    auto message = mqtt::make_message(topic, payload, qos, retained, props);
    client_->publish(message);
    // 日志风格与 tts_demo / robot_com 网关一致；心跳周期性高频，仅打简短一行
    if (topic == heartbeat_topic_) {
      RCLCPP_INFO(get_logger(), "Heartbeat 已正常发出");
    } else {
      RCLCPP_INFO(
        get_logger(), "MQTT 发出 topic=%s qos=%d retain=%s payload=%s",
        topic.c_str(), qos, retained ? "true" : "false",
        log_safe_payload(payload).c_str());
    }
    return true;
  } catch (const mqtt::exception & e) {
    RCLCPP_WARN(get_logger(), "MQTT 发布失败: %s topic=%s", e.what(), topic.c_str());
    return false;
  }
}

bool MqttGuideBridgeNode::seen_recently(const std::string & message_id)
{
  // 仅 worker 线程访问（指令串行处理），免锁
  if (!dedupe_.insert(message_id).second) {
    return true;
  }
  dedupe_order_.push_back(message_id);
  // ponytail: FIFO 有界淘汰，容量 dedupe_capacity_；需要按时间过期语义时再升级
  while (dedupe_order_.size() > static_cast<std::size_t>(dedupe_capacity_)) {
    dedupe_.erase(dedupe_order_.front());
    dedupe_order_.pop_front();
  }
  return false;
}

}  // namespace guide_pulse
