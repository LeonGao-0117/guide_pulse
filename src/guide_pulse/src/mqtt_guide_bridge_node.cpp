// ============================================================================
// MQTT <-> 展厅讲解 桥接节点实现。
// 框架（连接线程/遗嘱/心跳/队列/幂等）沿用 tts_demo，删 condition 状态机，
// 新增 scene（迎宾）/navigate（导航+汽车讲解）分发与到达事件消费。
// ============================================================================
#include "guide_pulse/mqtt_guide_bridge_node.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <utility>

#include <nlohmann/json.hpp>

#include "guide_pulse/service_call.hpp"

namespace guide_pulse
{

using nlohmann::json;

namespace
{

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
  // 话题，见 接口说明.md PlayTts 节），只能字数估算 + 参数化真机校准
  declare_parameter<int>("tts_speech_ms_per_char", 220);
  declare_parameter<int>("suppress_tail_lead_ms", 0);
  declare_parameter<double>("suppress_tail_scale", 0.8);
  declare_parameter<int>("suppress_tail_max_ms", 0);
  // 停止续期时刷的收尾抑制 TTL（注意 0=云端默认 3000ms；1=立即解除）
  declare_parameter<int>("suppress_tail_flush_ms", 1);

  // ---- 屏幕二维码显示（迎宾场景，v0.22.0）----
  declare_parameter<std::string>(
    "display_emotion_service_name", "/aimdk_5Fmsgs/srv/PlayEmotion");
  declare_parameter<std::string>("display_remote_host", "10.1.1.100");
  declare_parameter<std::string>("display_remote_user", "run");
  declare_parameter<std::string>(
    "display_remote_pass", "");  // 空禁用显示；亦可环境变量注入后写入参数
  declare_parameter<std::string>("display_remote_dir", "/home/run/screen_display");
  declare_parameter<int>("display_request_timeout_seconds", 5);
  declare_parameter<int>("display_show_timeout_seconds", 180);  // 3 分钟
  declare_parameter<int>("display_restore_emotion_id", 90);     // T1 默认表情

  // ---- 迎宾二维码缓存/下载（v0.22.0 P-036：scene 带 qr_sha256 哈希快照，
  // 未命中本地缓存时以 MQTT 凭据从 Server 文件面拉取图片）----
  declare_parameter<std::string>("qr_cache_dir", "/tmp/guide_pulse/qr_cache");
  declare_parameter<int>("qr_download_timeout_seconds", 5);

  // ---- 展厅讲解增量参数（docs/design.md §5）----
  declare_parameter<std::string>("nav_executor", "mock");
  declare_parameter<int>("mock_nav_seconds", 0);
  declare_parameter<int>("tour_speak_interval_seconds", 2);
  // scene 未带 texts 时的本地默认迎宾词
  declare_parameter<std::vector<std::string>>(
    "welcome_default_texts",
    std::vector<std::string>{"欢迎光临！我是展厅讲解机器人，很高兴为您服务。"});

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

  // ---- 播报伴随动作（TTS 受理成功时触发预设动作）----
  declare_parameter<bool>("motion_enabled", true);
  // 预设动作轮换池（McPresetMotion 枚举值：1001=右手挥手，2003~2009
  // 为说话动作族；多条时每次播报轮换一个）
  declare_parameter<std::vector<int64_t>>(
    "motion_preset_ids", std::vector<int64_t>{1001});
  declare_parameter<std::string>(
    "motion_service_name", "/aimdk_5Fmsgs/srv/SetMcPresetMotion");
  declare_parameter<int>("motion_request_timeout_seconds", 2);

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
  tour_speak_interval_seconds_ = std::max(1, static_cast<int>(
    get_parameter("tour_speak_interval_seconds").as_int()));
  welcome_default_texts_ = get_parameter("welcome_default_texts").as_string_array();
  if (welcome_default_texts_.empty()) {
    RCLCPP_WARN(
      get_logger(), "welcome_default_texts 为空，迎宾将使用内置兜底词");
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
  if (screen_cfg.remote_pass.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "display_remote_pass 为空：迎宾二维码显示禁用（TTS 播报不受影响）");
  }
  screen_ = std::make_unique<ScreenDisplay>(shared_from_this(), screen_cfg);

  // ---- 迎宾二维码缓存/下载（v0.22.0 P-036）----
  // 文件面与 enroll 注册面同源（RobotCar Server 单服务多路由）；
  // 下载凭据复用当次 enroll 的 MQTT 密码（HTTP Basic，username=robot_id）
  QrCodeCache::Config qr_cfg;
  qr_cfg.origin = enroll_origin_;
  qr_cfg.cache_dir = get_parameter("qr_cache_dir").as_string();
  qr_cfg.timeout_seconds =
    static_cast<int>(get_parameter("qr_download_timeout_seconds").as_int());
  qr_cache_ = std::make_unique<QrCodeCache>(qr_cfg);

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
  // 的 waypoint_follower 航点巡航服务，docs/design.md §4）
  if (nav_executor_name_ == "waypoint") {
    nav_ = std::make_unique<WaypointNavExecutor>(shared_from_this());
    RCLCPP_INFO(
      get_logger(), "导航执行器: waypoint（/waypoint_follower 航点服务，"
      "point_name 直接映射航点名）");
  } else {
    if (nav_executor_name_ != "mock") {
      RCLCPP_WARN(
        get_logger(), "未知 nav_executor='%s'，回退 mock", nav_executor_name_.c_str());
      nav_executor_name_ = "mock";
    }
    nav_ = std::make_unique<MockNavExecutor>(mock_nav_seconds_);
  }
  tour_ = std::make_unique<TourRunner>(*nav_);

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
// 工作线程：解析 command 信封 -> 回执/过期/幂等过滤 -> tts/tts.stop/scene/
// navigate 分发；音频会话按 next_at 周期调度；导航到达事件经 TourRunner
// 单槽消费（在途导航期间以 250ms 上限等待，保证及时开播讲解）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::worker_loop()
{
  while (running_.load()) {
    std::string item;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      // 计算定时唤醒时刻：取会话下一跳；二维码常驻超时；导航在途/到达
      // 待消费时再钳制 250ms 上限（NavExecutor 回调无本线程唤醒通道，
      // 轮询消费到达槽）
      auto wake_at = std::chrono::steady_clock::time_point::max();
      if (session_) {
        wake_at = std::min(wake_at, session_next_at_);
      }
      if (screen_ && screen_->active()) {
        wake_at = std::min(wake_at, screen_->deadline());
      }
      if (suppress_renew_active_) {
        wake_at = std::min(wake_at, suppress_next_at_);
      }
      if (tour_ && tour_->navigating()) {
        wake_at = std::min(
          wake_at, std::chrono::steady_clock::now() + std::chrono::milliseconds(250));
      }
      if (wake_at != std::chrono::steady_clock::time_point::max()) {
        queue_cv_.wait_until(
          lock, wake_at, [this]() {
            return !running_.load() || !queue_.empty();
          });
      } else {
        queue_cv_.wait(
          lock, [this]() {return !running_.load() || !queue_.empty();});
      }
      if (!running_.load()) {
        break;
      }
      if (!queue_.empty()) {
        item = std::move(queue_.front());
        queue_.pop_front();
      }
    }

    if (!item.empty()) {
      std::string error;
      // 契约 v0.6.0：payload 非法但能解析出 operation_id 的本机 tts 指令，
      // 须回 tts.result(rejected, invalid_request)；解析不出则直接丢弃。
      // scene/navigate 无回执面：载荷非法一律丢弃记日志
      std::string invalid_operation_id;
      std::string invalid_command_type;
      const auto event = parse_command_event(
        item, robot_id_, error, &invalid_operation_id, &invalid_command_type);
      if (!event) {
        if (error == "ignore") {
          // relocate/ota 等由其他组件处理，非本组件职责
          RCLCPP_DEBUG(
            get_logger(), "忽略非本组件指令 type=unknown（由其他组件处理）");
        } else if (!invalid_operation_id.empty()) {
          RCLCPP_ERROR(
            get_logger(),
            "下行指令校验失败: %s operation_id=%s，回执 rejected/invalid_request",
            error.c_str(), invalid_operation_id.c_str());
          send_tts_result(
            invalid_operation_id, TTS_RESULT_REJECTED, TTS_REASON_INVALID_REQUEST);
        } else {
          // 连 operation_id 都解析不出/信封非法/scene 未知值：丢弃记日志
          RCLCPP_ERROR(
            get_logger(), "下行指令校验失败: %s payload=%.120s",
            error.c_str(), item.c_str());
        }
        continue;
      }
      if (event->expired) {
        // 契约：过期 tts 回 rejected(command_expired)；
        // tts.stop/scene/navigate 无回执，过期直接丢弃
        if (event->type == "tts") {
          RCLCPP_WARN(
            get_logger(), "指令已过期: type=tts operation_id=%s，回执 rejected/command_expired",
            event->operation_id.c_str());
          send_tts_result(
            event->operation_id, TTS_RESULT_REJECTED, TTS_REASON_COMMAND_EXPIRED);
        } else {
          RCLCPP_WARN(
            get_logger(), "指令已过期丢弃: type=%s operation_id=%s",
            event->type.c_str(), event->operation_id.c_str());
        }
        continue;
      }
      {
        // 契约 v0.6.0：同一 operation_id 的指令重发必须重发同一回执
        //（同一 event_id），不得重新执行副作用
        std::lock_guard<std::mutex> lock(receipt_mutex_);
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
      const auto now = std::chrono::steady_clock::now();
      if (session_ && now >= session_next_at_) {
        play_current();
      } else if (suppress_renew_active_ && now >= suppress_next_at_) {
        if (suppress_stop_at_.has_value() && now >= *suppress_stop_at_) {
          // 尾部宽限到期（最后一条语音已播完）：停续期，刷短 TTL 立即解除
          suppress_renew_active_ = false;
          suppress_stop_at_.reset();
          suppress_wakeup(static_cast<std::uint32_t>(suppress_tail_flush_ms_));
        } else {
          // 播报进行中：周期续期唤醒抑制（TTL 短，停止续期后自动快速恢复）
          suppress_next_at_ = now +
            std::chrono::milliseconds(suppress_renew_interval_ms_);
          suppress_wakeup(static_cast<std::uint32_t>(suppress_ttl_ms_));
        }
      } else if (screen_ && screen_->active() && now >= screen_->deadline()) {
        // 二维码常驻超时：恢复默认表情（PlayEmotion 单通道，新请求即打断）
        screen_->cancel("超时3min");
      } else if (tour_) {
        // 到达槽消费（导航中 250ms 轮询；空闲时到达事件也会在唤醒后被拾取）
        if (const auto arrival = tour_->poll_arrival()) {
          if (arrival->ok) {
            if (arrival->texts.empty()) {
              RCLCPP_INFO(
                get_logger(), "导航到达: %s（未携带讲解词，不播报）",
                tour_->current_point().c_str());
            } else {
              RCLCPP_INFO(
                get_logger(), "导航到达: %s，开始汽车讲解（%zu 条）",
                tour_->current_point().c_str(), arrival->texts.size());
              start_session(
                std::move(arrival->texts), TTS_MODE_SEQUENCE,
                tour_speak_interval_seconds_, /*loop_forever=*/false,
                "讲解:" + tour_->current_point());
            }
          } else {
            RCLCPP_ERROR(
              get_logger(), "导航失败: %s (%s)（协议无导航回执面，仅记录）",
              tour_->current_point().c_str(), arrival->detail.c_str());
          }
        }
      }
    }
  }
}

void MqttGuideBridgeNode::handle_command(const CommandEvent & event)
{
  // 迎宾流程互斥：迎宾词 TTS 播完前（SDK 无播完事件，
  // 按字数估算设窗口，见 handle_scene），新的 scene/navigate 直接拒绝
  //（两者协议均无回执面，本地丢弃记日志，App 无感知）。tts/tts.stop
  // 不受限：tts 替换会话=迎宾流程终止（窗口随 start_session 失效），
  // tts.stop 停止一切
  if ((event.type == "scene" || event.type == "navigate") &&
    welcome_busy_until_.has_value() &&
    std::chrono::steady_clock::now() < *welcome_busy_until_)
  {
    const auto remain_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      *welcome_busy_until_ - std::chrono::steady_clock::now()).count();
    RCLCPP_WARN(
      get_logger(),
      "迎宾流程进行中(迎宾词播报未完)，拒绝 type=%s operation_id=%s（约%lldms 后结束）",
      event.type.c_str(), event.operation_id.c_str(),
      static_cast<long long>(remain_ms));
    return;
  }

  if (event.type == "tts") {
    // 新播报列表替换旧会话（App 通过重发 tts 更换播报内容），从头开始播；
    // 单轮播报：列表播完一轮自行结束
    // 新业务到达 -> 迎宾二维码让位（恢复默认表情）
    screen_->cancel("tts");
    start_session(
      event.texts, event.mode, event.interval_seconds, /*loop_forever=*/false,
      "tts:" + event.operation_id);
    // 契约 v0.6.0 回执：服务不可达/超时 = 播放子系统不可用 ->
    // rejected(playback_unavailable)；SDK 仲裁/业务拒绝属本地忙碌策略
    //（按 tts_retry_seconds 持续重试），照常回 accepted
    const auto result = play_current();
    if (result.service_failed) {
      send_tts_result(
        event.operation_id, TTS_RESULT_REJECTED, TTS_REASON_PLAYBACK_UNAVAILABLE);
    } else {
      send_tts_result(event.operation_id, TTS_RESULT_ACCEPTED);
    }
    return;
  }

  if (event.type == "scene") {
    handle_scene(event);
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
    // 语音已被 1099 掐断：唤醒抑制立即结束（刷短 TTL 立即解除）
    if (suppress_renew_active_) {
      suppress_renew_active_ = false;
      suppress_stop_at_.reset();
      suppress_wakeup(static_cast<std::uint32_t>(suppress_tail_flush_ms_));
    }
    const bool had_session = session_.has_value();
    const bool had_nav = tour_->navigating();
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
  }
}

// ---------------------------------------------------------------------------
// 迎宾模块（scene=welcome）：确定迎宾词（指令 texts 优先，缺省本地默认）
// -> 开一轮播报。不打断导航；单轮播完自行结束
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_scene(const CommandEvent & event)
{
  // 协议层已保证 scene=welcome（可扩展枚举唯一值），未知值在解析层丢弃

  // ---- 迎宾变形：异步发起，二维码就绪后等变形完成再开播（先变形、
  // 后人声）；已是人形零动作立即通过 ----
  transform_->ensure_biped("迎宾:" + event.operation_id);

  auto texts = make_welcome_texts(event.texts, welcome_default_texts_);
  // ---- 二维码：qr_sha256 与缓存比对，命中免下载；缺失只影响显示 ----
  if (event.qr_sha256.empty()) {
    RCLCPP_INFO(
      get_logger(), "迎宾开始: operation_id=%s 迎宾词=%zu 条 二维码=无（未携带 qr_sha256）",
      event.operation_id.c_str(), texts.size());
    screen_->cancel("scene未携带二维码");
  } else {
    auto image_path = qr_cache_->cached_path(event.qr_sha256);
    const bool cache_hit = image_path.has_value();
    if (!cache_hit) {
      const auto fetch = qr_cache_->fetch(robot_id_, password_, event.qr_sha256);
      if (fetch.ok) {
        image_path = fetch.image_path;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "迎宾二维码下载失败(本次不显示,不影响播报): hash=%.12s… %s",
          event.qr_sha256.c_str(), fetch.detail.c_str());
      }
    }
    if (image_path.has_value()) {
      RCLCPP_INFO(
        get_logger(), "迎宾开始: operation_id=%s 迎宾词=%zu 条 二维码=%s(hash=%.12s…)",
        event.operation_id.c_str(), texts.size(),
        cache_hit ? "缓存命中" : "已下载", event.qr_sha256.c_str());
      // 二维码常驻显示（5 分钟超时/新业务取消由 ScreenDisplay 与 worker 驱动）；
      // 显示失败不阻断迎宾语音（屏幕与音频相互独立）
      const auto show_result = screen_->show(
        *image_path, "迎宾:" + event.operation_id, event.qr_sha256);
      if (!show_result.ok) {
        RCLCPP_WARN(
          get_logger(), "迎宾二维码显示失败(不影响播报): %s",
          show_result.detail.c_str());
      }
    }
  }
  // ---- 等待变形完成后再开播（2026-10-15 需求：先变形、后人声）----
  // wait_done 必然在有限时间返回：变形线程内部每层等待都有上限（单次服务
  // 调用 3s / 动作到位总时限 / 等稳定 10s），运控不可达时更是立即放弃并
  // 照常播报。等待期间新指令在下行队列排队（worker 串行，处理完迎宾才
  // 轮到它们）；二维码的下载/转码/推送（实测 ~3.5s）与变形并行重叠，总
  // 时长 = max(变形, 二维码) 而非两者之和。worker 与本函数同线程调用
  // transform_，无并发；shutdown 先 join worker 再回收变形线程
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

  // 迎宾互斥窗口：公式同唤醒抑制尾部宽限（lead+估算×scale，共用参数）
  // + 条间间隔；赋值须在 start_session 之后（其内部 reset 旧窗口）
  int64_t welcome_busy_ms = 0;
  for (const auto & t : texts) {
    welcome_busy_ms += suppress_tail_lead_ms_ +
      static_cast<int64_t>(
        estimate_speech_seconds(t, tts_speech_ms_per_char_) * 1000.0 *
        suppress_tail_scale_);
  }
  welcome_busy_ms += static_cast<int64_t>(tour_speak_interval_seconds_) *
    static_cast<int64_t>(texts.size() - 1) * 1000;
  start_session(
    std::move(texts), TTS_MODE_SEQUENCE, tour_speak_interval_seconds_,
    /*loop_forever=*/false, "迎宾:" + event.operation_id);
  welcome_busy_until_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(welcome_busy_ms);
}

// ---------------------------------------------------------------------------
// 播报伴随动作：TTS 受理成功时从轮换池取下一个预设动作触发。
// worker 线程同步调用（正常毫秒级返回；异常最多阻塞 motion_request_
// timeout_seconds，不影响 5s 周期的抑制续期）。失败仅告警不重试不阻断。
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::trigger_speech_motion()
{
  if (!motion_enabled_ || !motion_client_ || motion_preset_ids_.empty()) {
    return;  // 功能关闭：只播语音不做动作
  }
  const auto motion_id = motion_preset_ids_[
    motion_next_index_ % motion_preset_ids_.size()];
  ++motion_next_index_;

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
// 讲解模块（navigate）：终止当前播报会话（前往新点位，旧讲解作废；
// 当前语音自然播完不抢占）-> 发起导航 -> 到达后由 worker 开讲解
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::handle_navigate(const CommandEvent & event)
{
  cancel_session();
  // 新业务到达 -> 迎宾二维码让位（前往展位，屏幕回默认表情）
  screen_->cancel("navigate");
  RCLCPP_INFO(
    get_logger(), "导航开始: point=%s 讲解词=%zu 条 operation_id=%s",
    event.point_name.c_str(), event.texts.size(), event.operation_id.c_str());
  tour_->start(event.point_name, event.texts);
}

// ---------------------------------------------------------------------------
// 音频会话（音频单会话策略：任一新会话替换旧会话并从头开始）
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::start_session(
  std::vector<std::string> texts, const std::string & mode, int interval_seconds,
  bool loop_forever, const std::string & source)
{
  if (session_) {
    RCLCPP_INFO(
      get_logger(), "替换播报会话: old=%s new=%s",
      session_source_.c_str(), source.c_str());
  }
  // 新会话 = 迎宾音频被打断/替换，迎宾流程互斥窗口随之失效
  //（迎宾自身重入由 handle_scene 在本函数之后重新设窗口）
  welcome_busy_until_.reset();
  // 单轮播报（tts/迎宾/讲解）期间抑制唤醒
  //（看门狗续期，见头文件说明）；循环播报（PlaylistSession 仍保留该能力，
  // 当前无调用方）不抑制——被唤醒进入对话属可接受业务行为。
  // 替换会话时：旧单轮会话的续期由新会话类型接管（循环=停止并快速恢复）
  if (!loop_forever) {
    suppress_renew_active_ = true;
    suppress_stop_at_.reset();
    suppress_next_at_ = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(suppress_renew_interval_ms_);
    suppress_wakeup(static_cast<std::uint32_t>(suppress_ttl_ms_));
  } else if (suppress_renew_active_) {
    // 单轮被循环会话替换：停续期，刷短 TTL 立即解除（新会话自行抑制）
    suppress_renew_active_ = false;
    suppress_stop_at_.reset();
    suppress_wakeup(static_cast<std::uint32_t>(suppress_tail_flush_ms_));
  }
  session_.emplace(
    std::move(texts), mode, interval_seconds, loop_forever);
  session_source_ = source;
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
    // 语音受理成功即触发伴随动作（动作与语音并行；失败只记日志）
    trigger_speech_motion();
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
        int64_t tail_ms = suppress_tail_lead_ms_ +
          static_cast<int64_t>(estimate_speech_seconds(text, tts_speech_ms_per_char_)) *
          1000LL * suppress_tail_scale_;
        if (suppress_tail_max_ms_ > 0) {
          tail_ms = std::min(tail_ms, static_cast<int64_t>(suppress_tail_max_ms_));
        }
        suppress_stop_at_ = std::chrono::steady_clock::now() +
          std::chrono::milliseconds(tail_ms);
      }
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
  // 会话终止（tts.stop/新 navigate）= 迎宾流程不再继续，互斥窗口失效
  welcome_busy_until_.reset();
}

// ---------------------------------------------------------------------------
// 上行发布：tts.result 回执 + presence / heartbeat
// ---------------------------------------------------------------------------
void MqttGuideBridgeNode::send_tts_result(
  const std::string & operation_id, const std::string & outcome,
  const std::string & reason)
{
  std::string envelope;
  {
    std::lock_guard<std::mutex> lock(receipt_mutex_);
    // 契约：同一 operation_id 必须重发同一回执（同 event_id），
    // 因此首次生成后缓存信封原文，重发原样复用
    const auto it = receipt_cache_.find(operation_id);
    if (it != receipt_cache_.end()) {
      envelope = it->second;
    } else {
      const auto receipt = build_tts_result_event(robot_id_, operation_id, outcome, reason);
      if (!receipt) {
        RCLCPP_ERROR(
          get_logger(), "tts.result 参数非法: operation_id=%s outcome=%s reason=%s",
          operation_id.c_str(), outcome.c_str(), reason.c_str());
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
  std::lock_guard<std::mutex> lock(dedupe_mutex_);
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
