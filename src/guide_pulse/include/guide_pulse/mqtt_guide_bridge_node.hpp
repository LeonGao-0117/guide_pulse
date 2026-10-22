// ============================================================================
// MQTT <-> 展厅讲解 桥接节点：enroll 换凭据 -> 订阅 down/commands
//（tts/tts.stop/scene/navigate/navigate.stop/point_map/relocate）
// -> 播报/导航/迎宾/重定位/动作执行 ->
// 回执与 presence/heartbeat 上行。详见 docs/design.md。
// MQTT：vendored paho（async_client + MQTT v5），下行先进队列由 worker
// 线程处理，重连由专属连接线程驱动（凭据轮换需应用层控制）。
// ============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <mqtt/async_client.h>
#include <rclcpp/rclcpp.hpp>

#include <aimdk_msgs/srv/set_agent_suppression.hpp>
#include <aimdk_msgs/srv/set_mc_preset_motion.hpp>

#include "guide_pulse/guide_protocol.hpp"
#include "guide_pulse/mc_transform.hpp"
#include "guide_pulse/mqtt_guide_bridge_node.hpp"
#include "guide_pulse/nav_executor.hpp"
#include "guide_pulse/playlist.hpp"
#include "guide_pulse/point_map.hpp"
#include "guide_pulse/qr_code_cache.hpp"
#include "guide_pulse/robot_enroll.hpp"
#include "guide_pulse/screen_display.hpp"
#include "guide_pulse/tour.hpp"
#include "guide_pulse/tts_client.hpp"
#include "guide_pulse/waypoint_nav_executor.hpp"

namespace guide_pulse
{

class MqttGuideBridgeNode final : public rclcpp::Node, private mqtt::callback
{
public:
  explicit MqttGuideBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MqttGuideBridgeNode() override;

  // 连接 MQTT broker（阻塞至连接建立或超时）；返回是否成功
  bool start();
  // 停止桥接：停工作线程、取消导航、发布 offline presence、断开连接
  void shutdown() noexcept;

  // 本地注入指令（线程安全）：语料触发等本地事件与 MQTT 下行指令走
  // 完全相同的 worker 串行处理链路（互斥窗口/会话策略全复用）。
  // operation_id/event_id 由调用方本地生成（唯一，无幂等/过期问题）
  void inject_command(CommandEvent event);
  // 播报终态回流（AgentMonitor TtsStatus END/STOP/ERROR 回调，monitor
  // 线程调用）：本会话 trace 前缀命中则唤醒 worker 提前解除麦克风静音
  // 与唤醒抑制（尾部宽限的字数估算偏保守，实际播完即恢复，缩短禁麦
  // 时长；BEST_EFFORT 丢事件时按原估算时序兜底），并即刻释放播报类
  // 业务占用（互斥窗口降级为兜底超时）
  void notify_tts_terminal(const std::string & trace_id);

  // 点位映射表生效通知（2026-10-19 现场需求：导航语料随点位设置动态
  // 生成）。main.cpp 注入；两条路径回调（参数为当前生效的整表，空表
  // = 规则应清空）：1) App 推表 handle_point_map 替换成功后（worker
  // 线程）；2) start() 启动加载持久化表后（主线程，加载成功与损坏按
  // 空表均通知——规则状态与表保持一致）。未设置 = no-op
  std::function<void(const std::vector<PointMapEntry> &)> on_point_map_updated;

private:
  // ---- mqtt::callback ----
  void connected(const std::string & cause) override;
  void connection_lost(const std::string & cause) override;
  void message_arrived(mqtt::const_message_ptr message) override;

  // ---- 内部流程 ----
  void declare_and_load_params();
  bool apply_credential(const RobotCredential & credential);
    // 应用当次 enroll 凭据；robot_id 变化时重建客户端与主题（防御性路径）
  void build_topics();                     // 由 robot_id 生成四个契约主题
  mqtt::connect_options build_connect_options();
    // 每次连接前重建（遗嘱须带本次 session_id，凭据取当次 enroll 结果）
  void connect_loop();                     // 连接管理线程：enroll -> 连接 -> 重试
  void worker_loop();
  void maintain_connection();   // 回调线程外的订阅恢复/上线通告（防 paho 自死锁）
  void handle_command(const CommandEvent & event);  // tts / tts.stop / scene / navigate / navigate.stop / point_map / show_qr / cancel_qr / relocate / welcome_prep（MQTT 下行 + 本地注入）
  void handle_scene(const CommandEvent & event);        // 迎宾模块入口
  void handle_navigate(const CommandEvent & event);     // 讲解模块入口
  // 迎宾准备模块入口（2026-10-18 现场需求，仅本地注入：AgentMonitor 命中
  // agent_monitor.welcome_prep_phrases 语料）：先确保人形（导航前置）->
  // 导航去迎宾点 -> 到达后变四足收摄像头 -> 播就位语。不发协议回执。
  // send_off=true 为送客模式（2026-10-21 现场需求，type=send_off 注入，
  // 命中 agent_monitor.send_off_phrases 语料）：链路同款，仅到达后播
  // 感谢语 send_off_done_text（不再播就位语）
  void handle_welcome_prep(const CommandEvent & event, bool send_off = false);
  void handle_point_map(const CommandEvent & event);    // 点位映射表全量替换入口（v0.25.0）
  void handle_show_qr(const CommandEvent & event);      // 语料触发二维码显示入口（本地注入，占业务互斥 20s）
  void handle_cancel_qr(const CommandEvent & event);    // 语料触发取消二维码显示入口（本地注入，占业务互斥 20s）
  void handle_relocate(const CommandEvent & event);     // 地图重定位入口（MQTT 下行 + 本地注入）：set_pose 受理 -> 等导航模块验收（worker 轮询）-> 终态回执+播报
  // 重定位终态收尾（worker 线程）：回执 relocate.result（验收通过=success /
  // 验收失败或 30s 超时=failed:localization_failed；语料触发不回执，改为
  // 验收通过时发 relocate.triggered 上报——2026-10-19 需求：只有重定位
  // 成功才发消息给后端）-> 释放业务占用 -> 成功时播报（texts 优先，缺省
  // 「{point_name}点 重定位 成功」，重定位前后空格分段防 TTS 连读）；
  // 失败播 relocate_failed_text 提示（2026-10-17 现场需求）。业务在回执
  // 发出后才结束
  void finish_relocate(bool verified);
  // 业务互斥拒绝时的语音提示（worker 线程）："我正在{业务}，请稍后"。
  // 仅非语音业务进行中播（重定位/导航在途——机器人没在说话，需要口头
  // 告知；迎宾/播报/讲解中正在出声，再插一句提示反而打断，不播——
  // 2026-10-17 需求：迎宾不需要说）。is_interrupted=false 同档排队，
  // 失败仅记日志（拒绝结果已回执，提示属本地增强）
  void speak_busy_hint();
  // 迎宾变形（确保人形形态）由 McTransform 模块承担（先变形、后人声的
  // 同步点在 handle_scene 内 transform_->wait_done()）
  // 播报伴随动作：TTS 受理成功时触发预设动作（轮换池取下一个）。
  // 同步短超时（正常毫秒级返回）；失败仅告警绝不影响语音播报
  void trigger_speech_motion();
  // 开播一个音频会话（替换当前会话——音频单会话策略）。mute_mic：仅
  // 迎宾讲解与导航讲解词传 true（静音麦克风防打断）；其余播报 false
  //（不掐麦克风，唤醒抑制照常武装）
  void start_session(
    std::vector<std::string> texts, const std::string & mode, int interval_seconds,
    bool loop_forever, const std::string & source, bool mute_mic);
  TtsClient::Result play_current();  // 播会话当前条并推进/排程（worker 线程）
  void cancel_session();             // 终止当前播报会话（不掐语音）
  // 等待单条播报结束（worker 线程，迎宾引导语专用）：AgentMonitor 回流
  // 的 TtsStatus 终态（trace 前缀命中置 tts_early_stop_）提前唤醒；字数
  // 估算（尾部宽限同款公式：lead + 每字ms×字数×scale，可选封顶）兜底
  // BEST_EFFORT 丢事件。返回是否终态回流（false = 估算到期或关停放行）
  bool wait_speech_done(const std::string & text);
  // 唤醒抑制原子调用：发送 SetAgentSuppression(WAKEUP, ttl_ms)。
  // ttl_ms=0 时服务端用默认 3s（用于会话结束后的快速恢复刷新）
  void suppress_wakeup(std::uint32_t ttl_ms);
  // 运控板麦克风硬抑制：SSH 执行 pactl set-source-mute
  // @DEFAULT_SOURCE@ true/false（PulseAudio 默认音源静音/恢复，
  // 无需厂商脚本）。
  // 背景（2026-10-16 真机日志）：唤醒抑制只拦 wuw 唤醒入口，迎宾前 ~30s
  // 的旧对话会话（cloud_event）仍会把用户语音当对话轮直送云端并
  // request_stop 掐断播报（绕过优先级仲裁与抑制）。静音 = 无音频进入 =
  // 弱打断与对话轮强打断皆无源。与唤醒抑制同生命周期，失败仅告警
  void set_mic_muted(bool mute);
  void publish_presence(const std::string & state, const std::string & reason);
  void publish_heartbeat();
  void send_tts_result(
    const std::string & operation_id, const std::string & outcome,
    const std::string & reason = "");  // 首次生成并缓存，重复原样重发
  // v0.24.0（P-038）：navigate.result / relocate.result / scene.result
  // 一次性最终结果回执。
  // 与 tts.result 共用 receipt_cache_（按 operation_id 键）：恰好一条，
  // 重复指令重发同一信封（同 event_id）。参数非法仅记日志
  void send_action_result(
    const std::string & result_type, const std::string & operation_id,
    const std::string & outcome, const std::string & reason = "");
  // v0.25.0（P-039）point_map.result 同走 send_action_result（共用回执
  // 缓存与幂等通道，无需独立函数）
  // v0.26.0（P-040）语音触发上报：发布恰好一条 navigate.triggered /
  // relocate.triggered（payload {point_name, trigger:voice}，无
  // operation_id）。2026-10-19 现场需求：**只有重定位成功才发**——
  // navigate/relocate 语料触发后，前置重定位（变形后 set_pose / set_pose
  // 验收）成功才调用本函数；重定位失败/被拒/超时不上报（App 不感知
  // 失败的语音触发）。一次性事件不进 receipt_cache_
  //（无指令重发场景，publish 失败与回执同策略仅告警）
  void send_nav_triggered(const std::string & triggered_type, const std::string & point_name);
  // 回执发布公共段：查缓存 -> 构造 -> 缓存（含淘汰）-> 发布（worker 线程）
  void publish_receipt(
    const std::string & operation_id, std::optional<TtsResultReceipt> receipt);
  // 播报一条业务拒绝提示（worker 线程）：不强抢进行中的其他播报、
  // 失败仅告警不影响回执；text 空则跳过（配置禁用）。回执由调用方负责
  void speak_hint(const std::string & text, const char * trace_id);
  // ---- 导航途中随机播报（2026-10-21 现场需求，仅 worker 线程）----
  // 取下一句途中播报词：本次导航固定首句（navigate_chatter_first_text /
  // navigate_chatter_welcome_first_text，受理时按目标定格）待播则先播
  // 固定句；否则按 nav_chatter_welcome_ 选池后随机取一条（不与上一条
  // 连续重复）；池空且无固定句返回空串
  std::string pick_nav_chatter_text();
  // 播报间隔：下一句等待 = text 播报时长估算（同尾部宽限公式）+
  // 静默秒数（间隔语义 = 两句之间的静默，不含播报时间）
  std::chrono::milliseconds nav_chatter_next_delay(const std::string & text) const;
  // 变形后重定位（worker 线程，2026-10-18 现场需求）：导航/迎宾前置
  // 变形**实际发生且成功**（McTransform::biped_transformed）时，同步发
  // 不带航点的 {command: set_pose} 到 t1_slam_nav_ws——变形动作自带约
  // 180° 转身把 AMCL 定位带偏，服务端用最新停留点（最近一次导航结束
  // 实际停的位置）重设初值拉回；本身就是人形（零动作路径）不发送
  //（2026-10-21 现场需求：已是人形不 set_pose，撤销 2026-10-19 的
  // force 强制机制——机器人没动定位未被带偏，强制重定位多此一举；
  // 变形后发送前先等 relocate_after_transform_delay_seconds——落位
  // 余晃期抖动会干扰 set_pose 初值，默认 2s，0 = 落位即发）。
  // 失败/被拒（无停留点/导航栈未起等）只记日志不阻断业务——补偿性
  // 重定位，尽力而为。返回 true = 成功（含未变形跳过路径），false =
  // 变形后尝试了但失败——语料触发的 navigate.triggered 上报据此把关
  //（只有重定位成功才发消息给后端）。调用点：handle_navigate /
  // handle_welcome_prep（均在双足门禁通过后、发导航 goal 前——
  // set_pose 在途时发 go 会被「任务在跑」拒，必须等它终态）、
  // handle_scene（变形 wait_done 后、二维码上屏前，返回值不用——迎宾
  // 无 triggered 上报面）。handle_relocate 不调：它自己的 set_pose 带
  // 航点，初值即重设，无二次修正面
  bool relocate_after_transform(const std::string & source);
  // 航点 code -> 映射表显示名（App 界面展示的人话名，如 A ->
  // 比亚迪ATTO 1）；反查失败返回 fallback（表结构上 code 必来自表内，
  // 仅防御）。navigate 引导语 {point} 替换与 P-040 语音触发上报共用
  std::string point_display_name(
    const std::string & waypoint, const std::string & fallback) const;
  bool publish_json(
    const std::string & topic, const std::string & payload, int qos, bool retained);
  bool seen_recently(const std::string & key);  // operation_id/event 幂等去重

  // ---- 业务互斥（仅 worker 线程访问，免锁）----
  // 同一时刻至多一个业务（迎宾/导览/播报/重定位/二维码显示/取消二维码）
  // 在执行；业务进行中任何新业务指令（tts/scene/navigate/show_qr/
  // cancel_qr）一律拒绝：tts 回 rejected(business_busy)，navigate/scene
  // 回各自 result 的 failed(busy)（v0.24.0 P-038）。tts.stop
  // 永远放行（停止一切并解除占用）；navigate.stop 永远放行（只取消在途
  // 导航，无导航 no-op，不影响播报类业务）；point_map 永远放行（v0.25.0
  // P-039：纯数据同步，无音频/导航/形态副作用，不占也不受业务互斥约束
  // ——映射表同步是控制面维护动作，不能因展厅业务在跑而被拒，App 侧
  // 也不应感知「忙碌」）。show_qr/cancel_qr 自 2026-10-17 起亦为互斥业务
  //（现场需求：显示与取消二维码显示是两个业务，各占 qr_business_
  // timeout_seconds（默认 20s）互斥窗口，与其他业务互斥；例外：cancel_qr
  // 可抢占 kShowQr 窗口——取消类动作，显示保护期内「收到后取消二维码
  // 显示」必须生效，与 tts.stop/navigate.stop 永远放行同语义）。业务
  // 结束：播报类以 TtsStatus 终态回流为准即刻释放（字数估算/迎宾固定
  // 120s 窗口仅兜底超时）；导览另含导航在途期（到达/失败事件即终态）；
  // 二维码业务无播报会话，仅窗口到期自然释放
  enum class Business
  {
    kNone,       // 空闲
    kTts,        // App 下发播报（tts 指令）
    kWelcome,    // 迎宾（scene=welcome，含语料触发）
    kTour,       // 导览（navigate：导航在途 + 到达后讲解）
    kRelocate,   // 重定位（relocate：等待导航模块验收 set_pose，30s 超时）
    kShowQr,     // 二维码显示（show_qr：语料触发上屏，20s 互斥窗口）
    kCancelQr,   // 取消二维码显示（cancel_qr：语料触发撤屏，20s 互斥窗口）
  };
  // 是否有业务占用（导览在途不受估算窗口限制，以导航状态为准；重定位
  // 等待期以 relocate_pending_ 为准——同样不受估算窗口限制，终态只认
  // 导航模块的验收结果或超时）
  bool business_busy() const;
  const char * business_name() const;  // 当前业务名（日志用）
  // 播报估算窗口截止 = lead + Σ(每条字数估算×scale) + 条间间隔。
  // 迎宾/播报/讲解共用（迎宾原有公式抽出）
  std::chrono::steady_clock::time_point estimate_busy_until(
    const std::vector<std::string> & texts, int interval_seconds) const;

  // ---- 参数（值由 declare_and_load_params()/start() 从 ROS 参数统一加载；
  // 默认值唯一源 = mqtt_guide_bridge_node.cpp 的 declare_parameter，yaml
  // 只覆盖部署环境与现场调参项，成员处不再写默认值防三处漂移）----
  std::string broker_uri_;
  std::string enroll_origin_;           // Server HTTP 注册面，如 http://host:8180
  std::string serial_number_;           // 本机序列号（须运维预登记）
  int enroll_timeout_seconds_;
  std::string topic_prefix_;            // robotCar/v1/robots（Server 固定值）
  std::string client_version_;          // presence/heartbeat 契约字段
  std::string tts_service_name_;        // /aimdk_5Fmsgs/srv/PlayTts
  std::string tts_domain_;              // 调用方来源标识
  std::string stop_text_;               // 停止播报使用的近静音文本
  int tts_priority_level_;              // SDK 播报主档位（仲裁 level*100+weight）
  int tts_priority_weight_;             // SDK 同档位加权 0-99
  int tts_retry_seconds_;               // 被仲裁拒绝后的快速重试间隔
  int keep_alive_seconds_;
  int connect_timeout_seconds_;
  int heartbeat_interval_seconds_;
  int tts_request_timeout_seconds_;
  int tts_wait_timeout_seconds_;        // 启动等待服务时长，0=不等待
  int dedupe_capacity_;
  // ---- 唤醒抑制：看门狗续期（TTL 短，worker 周期续期贴住实际播报
  // 时长；SDK 无播完事件，尾部宽限 lead+估算×scale，公式见 cpp）----
  std::string suppress_service_name_;   // /aimdk_5Fmsgs/srv/SetAgentSuppression
  bool suppress_wakeup_;                // false 关闭（恢复默认可打断行为）
  int suppress_ttl_ms_;                 // 每次抑制的 TTL（须大于续期间隔）
  int suppress_renew_interval_ms_;      // 续期间隔（worker 周期刷新）
  int tts_speech_ms_per_char_;          // 每字时长（CJK/数字档，拉丁≈0.73 倍）
  int suppress_tail_lead_ms_;           // 尾部宽限固定项（受理->出声的合成延迟）
  double suppress_tail_scale_;          // 尾部宽限系数（播报中被唤醒则调大）
  int suppress_tail_max_ms_;            // 尾部宽限封顶（0=不封顶）
  int suppress_tail_flush_ms_;          // 停续期收尾刷的 TTL（1=立即解除）
  // ---- 播报期间麦克风硬抑制（SSH pactl，2026-10-16）----
  bool mic_mute_enabled_;               // false 关闭（默认 true）
  std::string mic_mute_target_;         // SSH 目标 user@host（复用 display_remote_*）
  std::string mic_mute_pass_;           // SSH 密码（复用 display_remote_pass）
  std::atomic_bool mic_muted_{false};   // 当前已静音（幂等判据 + 关停恢复）
  // ---- 播报终态提前解除（queue_mutex_ 保护，跨线程）----
  std::string tts_trace_prefix_;        // 当前会话 trace 前缀（start_session 写）
  bool tts_early_stop_{false};          // 终态已回流：worker 尾部宽限期提前解除
  bool suppress_renew_active_{false};   // 续期进行中（仅 worker 线程访问）
  bool suppress_armed_{false};          // 唤醒抑制已实际武装（静音失败降级或 mute_mic=false 的会话）
  std::chrono::steady_clock::time_point suppress_next_at_;  // 下次续期时刻
  std::optional<std::chrono::steady_clock::time_point> suppress_stop_at_;
    // 尾部宽限截止：最后一条已受理（语音仍在播），续期持续到该时刻再停
  // ---- 业务互斥状态（仅 worker 线程访问，免锁）----
  Business business_{Business::kNone};
  std::optional<std::chrono::steady_clock::time_point> business_busy_until_;
    // 占用兜底超时截止：tts/讲解=字数估算，迎宾=固定总超时（不按词长）。
    // 正常路径播报终态回流即刻释放（提前于该截止）；导览在途期不设
    // 窗口（以 tour_->navigating() 为准）
  // ---- 展厅讲解增量参数 ----
  // 迎宾点 code（2026-10-19 现场需求：迎宾词强制以映射表下发为准——
  // scene 未带 texts 时取映射表该 code 条目的 text 作迎宾词；表未配
  // text 则本次迎宾不播词。空串 = 禁用映射表迎宾词）
  std::string welcome_point_code_;
  // 迎宾引导语（2026-10-17 现场需求：scene=welcome 到达后先播引导语
  //「元气满满，小T来迎接我的顾客了」，播完再变形，变形完成播迎宾词）。
  // 空 = 禁用（跳过引导语直接变形，回退原时序）
  std::string welcome_lead_text_;
  int welcome_flow_timeout_seconds_;    // 迎宾占用兜底超时：终态回流即释放
                                        // 占用，此值仅在丢终态事件时兜底
                                        //（不停播报/不动二维码，只放行新业务）
  int tour_speak_interval_seconds_;     // 迎宾/讲解词每两条间隔（载荷无间隔字段）
  std::string nav_executor_name_;       // 当前仅 mock（真机实现后扩展）
  int mock_nav_seconds_;                // Mock 导航模拟耗时
  int mock_relocate_hold_polls_;        // Mock 重定位验收等待窗口（poll 次数）
  int nav_timeout_seconds_;             // 导航在途兜底超时（默认 600s）：
                                        // ~/status 终态断流时按超时收尾释放
                                        // 导览业务，防「一直 busy 拒新业务」
  // ---- 重定位业务参数（2026-10-17：终态=导航模块验收结果）----
  int relocate_timeout_seconds_;        // 等待验收超时（默认 30s）
  // 变形后重定位发送前等待秒数（2026-10-21 现场需求）：变形动作落位后
  // 机器人还有余晃，立即发 set_pose 重设初值会被余晃期抖动干扰，先等
  // 这么多秒再发。0 = 原行为（落位即发）
  int relocate_after_transform_delay_seconds_{2};
  std::string busy_hint_relocate_text_; // 互斥拒绝提示（重定位中，空=不播）
  std::string busy_hint_navigate_text_; // 互斥拒绝提示（导航在途，空=不播）
  std::string navigate_failed_text_;    // 导航失败提示（回执后播，空=不播）
  std::string navigate_waypoint_missing_text_;
                                       // 导航被拒（航点表无该点位）提示
                                       //（回执后播，空=不播；2026-10-19）
  std::string navigate_transform_failed_text_;  // 变形未确认双足拒绝下发的
                                       // 提示（navigate/relocate 共用，空=不播）
  std::string relocate_failed_text_;    // 重定位失败提示（回执后播，空=不播）
  // 导览受理成功后的引导语（2026-10-17 现场需求：确认 [导航] 收到应答
  // accepted=true 后播「请跟我来…」；{point} 替换点位显示名。空=禁用）
  std::string navigate_lead_text_;
  // 导航目标为迎宾点时的引导语（2026-10-19 现场需求：App 下发 navigate
  // 去迎宾点 Y，普通引导语「请跟我来…去看看迎宾点这款车」语义不通——
  // 迎宾点不是车型）：改播这句（与 welcome_prep 触发语料同款）。判定：
  // code 命中 welcome_point_code 或点位显示名含「迎宾」关键词。空 =
  // 回退普通引导语 navigate_lead_text
  std::string navigate_lead_welcome_text_;
  // 导航出发播报语（2026-10-18 现场需求）：收到导航指令（语音/MQTT）
  // 受理后先播「稍等，马上出发」再起身变形人形（先开口、立刻动）；
  // 迎宾准备去迎宾点的导航不播（agent 已说过「整装待发，即刻就位」，
  // handle_welcome_prep 独立链路天然跳过）。空=禁用
  std::string navigate_depart_text_;
  // 重定位出发播报语（2026-10-18 现场需求）：仅 MQTT 下行的 relocate
  // 先播「马上开始重定位，请稍后」再起身变形人形；语料触发不播（agent
  // 已说「好的，这就在A点开始重定位」）。空=禁用
  std::string relocate_depart_text_;
  // ---- 导航途中随机播报（2026-10-21 现场需求）----
  // 安抚语料池（如「请不用着急，跟着我慢慢走」）：受理引导语播完后，
  // 每隔 min~max 秒（随机）挑一句播报（随机取、不与上一条连续重复），
  // 防导航途中长时间沉默太空；到达/取消/失败即停。
  // 去迎宾点的导航（迎宾准备/App 下发）无顾客跟走，改用迎宾专用池
  // navigate_chatter_welcome_texts（两池各自为空 = 对应场景不播）
  std::vector<std::string> navigate_chatter_texts_;
  std::vector<std::string> navigate_chatter_welcome_texts_;  // 去迎宾点专用池
  std::string navigate_chatter_first_text_;         // 普通导航固定首句（空=不固定）
  std::string navigate_chatter_welcome_first_text_;  // 去迎宾点固定首句（空=不固定）
  int navigate_chatter_silence_seconds_{0};  // 一句播完到下一句的静默秒数
                                         //（间隔不含播报时间；加载时钳 >=1；两池共用）
  // 下一次途中播报时刻（nullopt = 未安排：不在导航/对应池为空/已到）；
  // 仅 worker 线程访问，免锁；新导航 start 时重置、受理事件安排
  std::optional<std::chrono::steady_clock::time_point> nav_chatter_next_at_;
  bool nav_chatter_welcome_{false};      // 本次在途导航用迎宾语料池（受理时定格）
  bool nav_chatter_first_pending_{false};  // 本次导航固定首句待播（受理时按配置定格）
  std::size_t nav_chatter_last_index_{0};  // 上一条语料下标（防连续重复）
  std::string nav_point_display_;       // 本次导航点位显示名（映射表 name，引导语用）
  // ---- 点位映射表（v0.25.0 P-039）----
  std::string point_map_file_;          // 落盘路径（空=纯内存，联调用）
  std::string point_map_missing_text_;  // 表空时 navigate 的拒绝提示话术
  // 表空或点位名未命中时 navigate/relocate 的拒绝提示话术（{point}
  // 占位替换原始 point_name；2026-10-17 现场需求：映射表是唯一权威，
  // 未命中直接拒绝，不透传航点名）
  std::string point_map_unmatched_text_;
  std::unique_ptr<PointMapStore> point_map_;  // 表本体（仅 worker 线程访问，免锁）
                                         // 到达讲解词唯一来源 = 表内 text
                                         //（2026-10-19 现场需求：本地
                                         //  navigate_default_texts 已移除，
                                         //  强制映射表权威）
  // ---- 迎宾准备（2026-10-18 现场需求）----
  // 迎宾准备就位播报语：语料触发导航去迎宾点，到达变四足（摄像头收平）
  // 后最后播报（如「元气满满，做好准备迎接顾客」）。空串 = 禁用
  std::string welcome_prep_done_text_;
  // 送客到达播报语（2026-10-21 现场需求）：语料触发送顾客回迎宾点，
  // 到达变四足（摄像头收平）后最后播报的感谢语（如「再次感谢您对
  // 比亚迪的支持，欢迎下次光临」）。空串 = 禁用（到达即释放业务）
  std::string send_off_done_text_;
  // 迎宾准备到达后延迟变形秒数（2026-10-18 测试需求）：到达 Y 点收到
  // 导航 result 后不立即变形，等这么多秒再变四足。0 = 原行为（到达即变形）
  int welcome_prep_transform_delay_seconds_{0};
  // 当前在途导航是否迎宾准备（本地注入）：是则到达后变四足 + 播就位语，
  // 不做默认车型讲解词兜底、不播 navigate 引导语；仅 worker 线程访问
  bool nav_welcome_prep_{false};
  // 当前在途导航是否送客（本地注入，2026-10-21 现场需求）：与迎宾准备
  // 同款到达链路（变四足收摄像头），但最后播感谢语 send_off_done_text；
  // 途中随机播报用普通安抚池（顾客在跟随，「请跟紧我」比迎宾赶路池
  // 贴切）；仅 worker 线程访问
  bool nav_send_off_{false};

  // ---- 迎宾变形（scene=welcome 时确保人形形态）：由 McTransform 模块
  // 承担，参数校验在 start() 内完成（welcome_transform 等）----

  // ---- 播报伴随动作（TTS 受理成功时触发预设动作）----
  std::string mc_source_name_;          // SetMc* 请求 source（变形模块共用）
  bool motion_enabled_;                 // false 关闭（只播语音不做动作）
  std::vector<int32_t> motion_preset_ids_;   // 动作轮换池（参数注入；空=关闭）
  std::size_t motion_next_index_{0};     // 轮换索引（仅 worker 线程访问）
  int motion_request_timeout_seconds_;
  rclcpp::Client<aimdk_msgs::srv::SetMcPresetMotion>::SharedPtr motion_client_;
  rclcpp::CallbackGroup::SharedPtr motion_callback_group_;
  // ---- 迎宾二维码（v0.22.0 P-036：哈希快照 + 文件面按需下载）----
  // 未携带 qr_sha256 的迎宾（含语料触发）显示的默认二维码图片路径；
  // 空或文件缺失 = 不显示（空串在 start() 校验时落定）
  std::string default_qr_path_;
  // 二维码业务互斥窗口秒数（2026-10-17 现场需求：显示/取消二维码显示
  // 是两个业务，与其他业务互斥，超时 20s；窗口只管互斥占用，二维码
  // 常驻时长仍由 display_show_timeout_seconds 独立计时）
  int qr_business_timeout_seconds_;

  // ---- 运行时状态 ----
  std::string robot_id_;               // enroll 响应分配（同 SN 稳定），空=尚未取得
  std::string username_;               // 当次 enroll 的 MQTT 凭据（仅内存）
  std::string password_;
  std::string down_topic_;              // {prefix}/{id}/down/commands（下行指令）
  std::string up_commands_topic_;       // {prefix}/{id}/up/commands（tts.result 回执）
  std::string heartbeat_topic_;
  std::string presence_topic_;
  std::string boot_id_;
  std::string session_id_;              // 每次连接重生成（uuidv7，契约要求）
  std::uint64_t heartbeat_sequence_{0};  // 契约要求从 1 递增
  std::atomic_bool ever_online_{false};
  std::chrono::steady_clock::time_point started_at_;
  std::atomic_bool running_{false};
  std::atomic_bool connected_{false};
  std::atomic_bool subscribed_{false};

  std::unique_ptr<RobotEnroller> enroller_;
  std::unique_ptr<mqtt::async_client> client_;
  std::unique_ptr<TtsClient> tts_;
  std::unique_ptr<ScreenDisplay> screen_;  // 迎宾二维码显示（脸屏）
  std::unique_ptr<QrCodeCache> qr_cache_;  // 迎宾二维码缓存/下载（P-036）
  rclcpp::Client<aimdk_msgs::srv::SetAgentSuppression>::SharedPtr suppress_client_;
  rclcpp::CallbackGroup::SharedPtr suppress_callback_group_;
  std::unique_ptr<McTransform> transform_;  // 迎宾变形（确保人形形态）
  std::unique_ptr<NavExecutor> nav_;    // MockNavExecutor（真机导航对接点）
  std::unique_ptr<TourRunner> tour_;    // 讲解模块状态机
  rclcpp::TimerBase::SharedPtr maintenance_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::thread connect_thread_;          // enroll + MQTT（重）连接驱动线程

  // 连接线程内部状态（仅该线程读写，无需加锁）
  bool credentials_ready_{false};       // 已拿到当次 enroll 凭据
  bool need_enroll_{true};              // 需要发起 enroll（首次/凭据被拒后）
  bool enroll_halted_{false};           // robot_revoked：停止 enroll 重试等运维

  // 下行队列：paho 回调线程生产 payload（topic 已在入队前过滤为本机下行主题），
  // 工作线程消费
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::string> queue_;
  // 本地注入指令队列（语料触发等）：与 queue_ 共用锁与唤醒通道，工作
  // 线程优先消费（语料命中即刻进迎宾流程，不排在 MQTT 积压之后）
  std::deque<CommandEvent> injected_commands_;
  std::thread worker_thread_;

  // 音频会话（仅 worker 线程访问，无锁）：tts / 迎宾 / 讲解均为单轮播完。
  // 任一新会话替换旧会话（音频单会话策略，docs/design.md §3.4）
  std::optional<PlaylistSession> session_;
  std::string session_source_;          // 会话来源（日志：tts/迎宾/讲解 operation_id）
  std::chrono::steady_clock::time_point session_next_at_;

  // message_id 有界去重表：set 查重 + deque 记 FIFO 淘汰顺序。
  // 仅 worker 线程访问（指令串行处理），免锁
  std::unordered_set<std::string> dedupe_;
  std::deque<std::string> dedupe_order_;

  // operation_id -> 已发回执信封原文（含 event_id；tts.result 与
  // navigate.result / scene.result 共用）。契约 v0.6.0/v0.24.0：同一
  // operation_id 的指令重发必须重发同一回执（同 event_id），因此缓存原文
  // 原样重发；与 dedupe_ 同容量上限。仅 worker 线程访问，免锁
  std::unordered_map<std::string, std::string> receipt_cache_;

  // 当前在途（或到达待消费）导航的 operation_id：到达/失败/中止时回
  // navigate.result（v0.24.0）。单业务模型下至多一个在途导航，
  // 仅 worker 线程访问，免锁；cancel/终态后清空
  std::string nav_operation_id_;

  // 在途导航是否本地注入（语料触发，2026-10-17：agent 复述车型名 ->
  // 查点位映射表导航）：是则到达/失败/中止均不发协议回执（Server
  // 不认识本地铸造的 operation_id），仅 worker 线程访问，受理时赋值
  bool nav_local_{false};

  // ---- 重定位等待态（2026-10-17：业务终态 = 导航模块验收结果，非受理）。
  // 仅 worker 线程访问，免锁；relocate_pending_=true 期间业务互斥生效
  //（新业务被拒 + TTS 提示「我正在重定位，请稍后」），终态/超时清空 ----
  bool relocate_pending_{false};      // 已受理、等待导航模块验收（set_pose goal result）
  std::string relocate_operation_id_;  // 在途重定位的 operation_id（回执用）
  std::string relocate_waypoint_;      // 解析后的航点名（验收轮询匹配用）
  std::string relocate_point_name_;    // 原始 point_name（默认播报文案用）
  std::string relocate_display_name_;  // 映射表显示名（受理时定格；语料触发
                                       // 验收通过后的 relocate.triggered 上报用，
                                       // 2026-10-19 需求——等待期 point_map 可能
                                       // 被 App 重推，终态反查会漂移）
  std::vector<std::string> relocate_texts_;  // 指令携带的随动播报（空=播默认）
  bool relocate_local_{false};         // 本地注入（语料触发）：不发协议回执
  std::chrono::steady_clock::time_point relocate_deadline_;
    // 超时截止（relocate_timeout_seconds，默认 30s）：到期仍未拿到验收
    // 结果按 failed/localization_failed 收尾（服务端 ~1.5s 出结果，30s
    // 是 status 丢失/服务卡死的宽裕兜底）
};

}  // namespace guide_pulse
