// ============================================================================
// guide_pulse 协议层：后端 -> 机器人 MQTT 下行指令报文的解析/校验、
// 机器人 -> 后端回执报文（tts.result + navigate/scene 一次性结果回执）的
// 构造，以及时间/ID 等工具函数。
// 本层只依赖 nlohmann/json，不依赖 ROS，便于独立单元测试。
// 协议定义详见 docs/protocol.md 与 RobotCar Protocol 契约
// (contracts/server-robot/mqtt/v1，协议 v0.5.0 列表播报 + v0.6.0 tts.result
// + v0.16.0 navigate + v0.17.0 scene + v0.20.0 随动播报 texts + v0.22.0
// scene 可选 qr_sha256 迎宾二维码哈希快照 + v0.23.0 navigate.stop
// + v0.24.0 navigate/scene 一次性结果回执 P-038 + v0.25.0 point_map
// 点位映射表同步 P-039 + v0.21.0 relocate 地图重定位（2026-10-17 接入
// MQTT 下行处理，含 relocate.result 回执） + v0.26.0 语音触发导航/
// 重定位上报 navigate.triggered/relocate.triggered P-040）。
// ============================================================================
#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace guide_pulse
{

// ---------------------------------------------------------------------------
// 点位映射表条目（v0.25.0 P-039 point_map 载荷元素）
// ---------------------------------------------------------------------------
struct PointMapEntry
{
  std::string code;  // 点位代码：1..32 个 Unicode 标量，表内唯一（不透明，
                     // Robot 侧映射到本地航点名，如 A/B/C）
  std::string name;  // 点位名称：1..64 个 Unicode 标量（规则同 point_name，
                     // App 界面展示的人话名，如「比亚迪ATTO 1」）
};

// ---------------------------------------------------------------------------
// 协议常量
// ---------------------------------------------------------------------------
// tts.mode 播报模式：sequence 按列表顺序 / shuffle 每轮重洗
//（机器人侧均单轮播完）
inline constexpr const char * TTS_MODE_SEQUENCE = "sequence";  // 按列表顺序播报
inline constexpr const char * TTS_MODE_SHUFFLE = "shuffle";    // 随机顺序播报（每轮重洗）

// tts.result outcome
inline constexpr const char * TTS_RESULT_ACCEPTED = "accepted";
inline constexpr const char * TTS_RESULT_REJECTED = "rejected";
// tts.result reason 枚举（rejected 必带其一，accepted 禁带 reason）
inline constexpr const char * TTS_REASON_COMMAND_EXPIRED = "command_expired";
inline constexpr const char * TTS_REASON_INVALID_REQUEST = "invalid_request";
inline constexpr const char * TTS_REASON_PLAYBACK_UNAVAILABLE = "playback_unavailable";
// 业务互斥拒绝（v0.23.0 本地扩展）：迎宾/导览/播报任一业务进行中，
// 新业务指令一律拒绝（后端收到后可提示用户稍后再试）
inline constexpr const char * TTS_REASON_BUSINESS_BUSY = "business_busy";

// scene 场景值（可扩展枚举，当前唯一值 welcome；扩枚举须先改契约再改这里）
inline constexpr const char * SCENE_WELCOME = "welcome";

// ---------------------------------------------------------------------------
// 语音触发上报（v0.26.0 P-040）：TTS 话术命中 agent_monitor.*_phrases
// 语料（语音自主触发）并受理开始执行导航/重定位时，发布恰好一条
// navigate.triggered / relocate.triggered（仅上行，payload {point_name,
// trigger}，无 operation_id——协议首个不由 App 指令驱动的上行事件，
// App 下发的指令不产生本上报，本地拒绝触发也不发布）。Server 按
// event_id 去重后推 App（WebSocket robot.navigation.triggered）
// ---------------------------------------------------------------------------
inline constexpr const char * NAVIGATE_TRIGGERED_TYPE = "navigate.triggered";
inline constexpr const char * RELOCATE_TRIGGERED_TYPE = "relocate.triggered";

// 触发类型 trigger（可扩展枚举，当前唯一值 voice；扩枚举须先改契约
// 再改这里，同 SCENE_WELCOME 模式）
inline constexpr const char * TRIGGER_VOICE = "voice";

// ---------------------------------------------------------------------------
// 一次性最终结果回执（v0.24.0 P-038）：navigate / scene 执行完成或失败
// 确定后发布 navigate.result / scene.result，payload {operation_id, outcome,
// reason?}。outcome = success（禁 reason）/ failed（必填 reason，按指令
// 枚举）/ rejected（必填 reason：command_expired / invalid_request，
// 复用上面 TTS_REASON_* 常量）。
// relocate.result（2026-10-17 接入）：MQTT 下行 relocate 由本组件处理
//（此前仅本地语料触发、MQTT 下行静默忽略），failed 枚举 busy /
// point_not_found / localization_failed。
// point_map.result（v0.25.0 P-039）：映射表全量替换完成/失败确定后发布，
// 语义与 navigate.result 完全一致（failed 枚举仅 point_map_failed）
// ---------------------------------------------------------------------------
inline constexpr const char * NAVIGATE_RESULT_TYPE = "navigate.result";
inline constexpr const char * RELOCATE_RESULT_TYPE = "relocate.result";
inline constexpr const char * SCENE_RESULT_TYPE = "scene.result";
inline constexpr const char * POINT_MAP_RESULT_TYPE = "point_map.result";

// 结果回执 outcome
inline constexpr const char * ACTION_RESULT_SUCCESS = "success";
inline constexpr const char * ACTION_RESULT_FAILED = "failed";
inline constexpr const char * ACTION_RESULT_REJECTED = "rejected";

// failed reason 枚举（按指令；见 Protocol error-codes.md P-038/P-039 表）
inline constexpr const char * RESULT_REASON_BUSY = "busy";
inline constexpr const char * RESULT_REASON_POINT_NOT_FOUND = "point_not_found";
inline constexpr const char * RESULT_REASON_NAVIGATION_FAILED = "navigation_failed";
inline constexpr const char * RESULT_REASON_ABORTED = "aborted";
inline constexpr const char * RESULT_REASON_LOCALIZATION_LOST = "localization_lost";
inline constexpr const char * RESULT_REASON_LOCALIZATION_FAILED = "localization_failed";
    // relocate.result 专属：重定位执行失败（服务不可用/被拒归并，如
    // 非站立人形、定位未起、验收窗口占用）
inline constexpr const char * RESULT_REASON_SCENE_FAILED = "scene_failed";
inline constexpr const char * RESULT_REASON_POINT_MAP_FAILED = "point_map_failed";
    // point_map.result 专属：本地应用/存储失败（如磁盘不可写）

// scene.qr_sha256 格式校验（v0.22.0 P-036）：64 位小写十六进制
//（command-event.schema.json pattern ^[0-9a-f]{64}$）
bool valid_qr_sha256(const std::string & sha256);

// 契约限制（command-event.schema.json）
inline constexpr std::size_t TTS_TEXTS_MAX = 20;         // texts 最多 20 条
inline constexpr std::size_t TTS_TEXT_MAX_BYTES = 2048;  // 每条 1..2048 UTF-8 字节
inline constexpr std::size_t POINT_NAME_MAX_SCALARS = 64;  // point_name 1..64 Unicode 标量
inline constexpr std::size_t POINT_CODE_MAX_SCALARS = 32;  // point_map.code 1..32 Unicode 标量
inline constexpr std::size_t POINT_MAP_POINTS_MAX = 256;   // point_map.points 1..256 条

// ---------------------------------------------------------------------------
// 下行设备指令（契约：后端发布到 {prefix}/{robot_id}/down/commands，
// type=tts 播报（有回执）、tts.stop 停止播报（无回执）、
// type=scene 场景指令（无回执，迎宾）、type=navigate 导航（无回执，汽车
// 讲解）、type=navigate.stop 停止导航（无回执，v0.23.0，与 tts.stop 同构）、
// type=point_map 点位映射表同步（v0.25.0 P-039，全量替换本地映射）、
// type=relocate 地图重定位（载荷同 navigate：point_name 必填 + 可选
// texts；2026-10-17 接入 MQTT 下行，见 handle_relocate）
// ---------------------------------------------------------------------------
struct CommandEvent
{
  std::string event_id;           // 信封事件 ID（uuidv7）
  std::string type;               // tts | tts.stop | scene | navigate | navigate.stop
                                   //   | point_map | relocate
                                   //（另: show_qr 仅本地注入——语料触发二维码
                                   //  显示，非 MQTT 协议指令，解析层不产出；
                                   //  relocate/navigate 兼具 MQTT 下行与本地
                                   //  注入（语料触发）两个来源）
  std::string operation_id;       // 幂等键（uuidv7，point_map 为 Server 铸造的
                                   // 子 ID，App 父 ID 不出现在 MQTT 面）
  // type=tts 载荷字段（v0.5.0 播报列表：texts + mode + interval_seconds）
  std::vector<std::string> texts; // tts: 必填；scene/navigate/relocate:
                                   //   可选随动播报（v0.20.0）
  std::string mode;               // sequence 按序循环 | shuffle 随机循环
  int interval_seconds{0};        // 每两条内容之间的播报间隔，恒必填（1..86400）
  // type=scene 载荷字段（v0.17.0：可扩展枚举，当前仅 welcome；
  // v0.22.0 P-036 可选 qr_sha256：Server 下发时该机器人当前迎宾二维码
  // 图片的内容哈希快照（64 位小写十六进制）。Robot 与本地缓存比对：
  // 一致直接显示缓存图片不下载；不一致经二维码文件面（HTTP Basic =
  // 自身 MQTT 凭据）拉取一次并校验后替换缓存再显示（QrCodeCache 组件）；
  // 字段省略本次不显示二维码）
  std::string scene;
  std::string qr_sha256;         // scene: 可选（64 位小写十六进制内容哈希）
  // type=navigate / relocate 载荷字段（v0.16.0：点位名 + 可选随动播报；
  // relocate 载荷形状与 navigate 完全一致：point_name 必填 + texts 可选）
  std::string point_name;         // 位置不透明名（1..64 Unicode 标量，Server 原样转发）
  // type=point_map 载荷字段（v0.25.0 P-039：points 全量下发，Robot 整表
  // 替换本地映射；表内 code 唯一是语义校验，解析层纵深防御）
  std::vector<PointMapEntry> points;
  bool expired{false};            // expires_at 早于当前时间
  bool local{false};              // 本地注入（语料触发等）：非 MQTT 下行，
                                   // 不发协议回执（Server 不认识该 operation_id）
};

// 解析并校验 down/commands 信封（严格 8 字段、uuid、robot_id/direction 匹配、
// expires_at 不得晚于 occurred_at + 2 分钟）。
// type 不属于 tts/tts.stop/scene/navigate/navigate.stop/point_map/relocate
// 时返回 nullopt 且 error 为 "ignore"（非错误，静默忽略：ota 等由其他
// 组件处理）。
// expires_at 已过期时 parsed->expired=true，由调用方决定回执/丢弃。
// 文本长度按契约校验（texts 每条 1..2048 字节；point_name 1..64 码点），无本地截断。
// invalid_operation_id / invalid_command_type：可选出参。当信封确认为本机
// type=tts/navigate/scene/point_map/relocate 指令、但报文校验失败且能提取
// 出合法 operation_id 时填充——调用方应按类型回 tts.result /
// navigate.result / relocate.result / scene.result / point_map.result
// (rejected, invalid_request)（契约：Payload 非法但仍能解析出
// operation_id；point_map 的 code 重复属此类——Server 已校验，Robot 侧
// 校验是纵深防御）。tts.stop/navigate.stop 及未知 scene 枚举值（未来版本
// 指令）不填。信封本身非法（robot_id 不符/解析不出 operation_id）时不填，
// 直接丢弃记日志。
// 成功返回 CommandEvent；失败返回 std::nullopt，error 输出原因。
std::optional<CommandEvent> parse_command_event(
  const std::string & payload, const std::string & robot_id, std::string & error,
  std::string * invalid_operation_id = nullptr,
  std::string * invalid_command_type = nullptr);

// ---------------------------------------------------------------------------
// 上行回执（机器人发布到 {prefix}/{robot_id}/up/commands，QoS 1）：
// tts.result（v0.6.0 受理回执）与 navigate.result / scene.result /
// point_map.result（v0.24.0/v0.25.0 一次性最终结果回执）。同一
// operation_id 的指令重发必须重发同一回执（同 event_id），调用方缓存
// envelope_json 原文重发即可。tts.stop / navigate.stop 无回执（契约），
// 不得构造。
// ---------------------------------------------------------------------------
struct TtsResultReceipt
{
  std::string event_id;       // 回执事件 ID（uuidv7，构造时生成并固定）
  std::string envelope_json;  // 完整回执信封 JSON（重发时原样复用）
};

// 构造 tts.result 回执信封。
// outcome：accepted（reason 必须为空）/ rejected（reason 必须为枚举之一，
// 含 playback_unavailable）。
// 参数非法（未知 outcome、accepted 带 reason、rejected 缺/非法 reason）返回 nullopt。
std::optional<TtsResultReceipt> build_tts_result_event(
  const std::string & robot_id, const std::string & operation_id,
  const std::string & outcome, const std::string & reason = "");

// 构造 navigate.result / relocate.result / scene.result / point_map.result
// 一次性最终结果回执信封（v0.24.0/v0.25.0）。result_type 仅接受
// NAVIGATE_RESULT_TYPE / RELOCATE_RESULT_TYPE / SCENE_RESULT_TYPE /
// POINT_MAP_RESULT_TYPE；
// outcome：success（reason 必须为空）/ failed（reason 必须为该指令的
// failed 枚举之一）/ rejected（reason 必须为 command_expired /
// invalid_request）。参数非法返回 nullopt。
// 信封在构造时生成（契约：TTL 只约束回执自身，不约束执行耗时）。
std::optional<TtsResultReceipt> build_action_result_event(
  const std::string & robot_id, const std::string & result_type,
  const std::string & operation_id, const std::string & outcome,
  const std::string & reason = "");

// 构造 navigate.triggered / relocate.triggered 语音触发上报信封
//（v0.26.0 P-040，仅上行自主事件）。triggered_type 仅接受
// NAVIGATE_TRIGGERED_TYPE / RELOCATE_TRIGGERED_TYPE；trigger 仅接受
// TRIGGER_VOICE（可扩展枚举当前唯一值）；point_name 须为 1..64 个
// Unicode 标量。payload 无 operation_id（契约：无下发指令可关联）。
// 参数非法返回 nullopt
std::optional<TtsResultReceipt> build_nav_triggered_event(
  const std::string & robot_id, const std::string & triggered_type,
  const std::string & point_name, const std::string & trigger);

// 生成一轮播放顺序：sequence 原序返回；shuffle 每轮重新洗牌（Fisher-Yates）。
// 契约建议 shuffle 每轮开始前重新洗牌，避免固定顺序。
std::vector<std::string> make_round_order(
  const std::vector<std::string> & texts, const std::string & mode);

// ---------------------------------------------------------------------------
// 单次播报请求（桥接节点由 CommandEvent 组装，传给 TtsClient）
// ---------------------------------------------------------------------------
struct TtsCommand
{
  std::string text;               // 待合成播报的文本（建议 100 字以内）
  int priority_level{6};          // SDK 优先级主档位，默认 6 (INTERACTION_L6)
  int priority_weight{0};         // 同档位加权 0-99
  bool is_interrupted{true};      // 是否抢占当前同/低优先级播报
  std::string trace_id;           // 业务侧链路追踪 ID，原样透传给 SDK
};

// SDK (aimdk_msgs/TtsPriorityLevel) 合法主优先级档位
bool valid_priority_level(int level);
// 加权合法范围 0-99
bool valid_priority_weight(int weight);

// 当前时间 RFC3339 UTC（毫秒精度），如 2026-09-23T10:00:00.123Z
std::string now_rfc3339();

// 生成 UUID v7（时间戳有序），后端 presence/heartbeat 契约要求
// session_id/boot_id 必须为 uuidv7 格式
std::string generate_uuid_v7();

// 播报时长估算（秒，向上取整），用于循环间隔调度：interval_seconds 语义为
// "每两条内容之间的间隔"，需叠加语音时长。按语种分档计费：
// CJK 字符（含全角标点）与数字按 ms_per_char（默认 220ms，真机实测校准）；
// 拉丁字母 ~160ms/字母；空格与 ASCII 标点不计
int estimate_speech_seconds(const std::string & text, int ms_per_char = 220);

// topic 拼接：{prefix}/{robot_id}/{suffix}
std::string topic_for(
  const std::string & prefix, const std::string & robot_id, const std::string & suffix);

}  // namespace guide_pulse
