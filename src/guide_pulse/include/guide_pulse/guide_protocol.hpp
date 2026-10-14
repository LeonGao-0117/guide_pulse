// ============================================================================
// guide_pulse 协议层：后端 -> 机器人 MQTT 下行指令报文的解析/校验、
// 机器人 -> 后端 tts.result 回执报文的构造，以及时间/ID 等工具函数。
// 本层只依赖 nlohmann/json，不依赖 ROS，便于独立单元测试。
// 协议定义详见 docs/protocol.md 与 RobotCar Protocol 契约
// (contracts/server-robot/mqtt/v1，协议 v0.5.0 列表播报 + v0.6.0 tts.result
// + v0.16.0 navigate + v0.17.0 scene + v0.20.0 随动播报 texts)。
// ============================================================================
#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace guide_pulse
{

// ---------------------------------------------------------------------------
// 协议常量
// ---------------------------------------------------------------------------
// tts.mode 播报模式：sequence 按列表顺序 / shuffle 每轮重洗（2026-10-14 起
// 机器人侧单轮播完，不再无限循环）
inline constexpr const char * TTS_MODE_SEQUENCE = "sequence";  // 按列表顺序播报
inline constexpr const char * TTS_MODE_SHUFFLE = "shuffle";    // 随机顺序播报（每轮重洗）

// tts.result outcome
inline constexpr const char * TTS_RESULT_ACCEPTED = "accepted";
inline constexpr const char * TTS_RESULT_REJECTED = "rejected";
// tts.result reason 枚举（rejected 必带其一，accepted 禁带 reason）
inline constexpr const char * TTS_REASON_COMMAND_EXPIRED = "command_expired";
inline constexpr const char * TTS_REASON_INVALID_REQUEST = "invalid_request";
inline constexpr const char * TTS_REASON_PLAYBACK_UNAVAILABLE = "playback_unavailable";

// scene 场景值（可扩展枚举，当前唯一值 welcome；扩枚举须先改契约再改这里）
inline constexpr const char * SCENE_WELCOME = "welcome";

// 契约限制（command-event.schema.json）
inline constexpr std::size_t TTS_TEXTS_MAX = 20;         // texts 最多 20 条
inline constexpr std::size_t TTS_TEXT_MAX_BYTES = 2048;  // 每条 1..2048 UTF-8 字节
inline constexpr std::size_t POINT_NAME_MAX_SCALARS = 64;  // point_name 1..64 Unicode 标量
// scene.qrcode_png_base64 长度上限（≈2MB PNG 的 base64 展开长度；严格
// 解码与图片大小校验由 ScreenDisplay 组件负责，此处仅防巨型载荷）
inline constexpr std::size_t QRCODE_BASE64_MAX_BYTES = 2796210;

// ---------------------------------------------------------------------------
// 下行设备指令（契约：后端发布到 {prefix}/{robot_id}/down/commands，
// type=tts 播报（有回执）、tts.stop 停止（无回执）、
// type=scene 场景指令（无回执，迎宾）、type=navigate 导航（无回执，汽车讲解））
// ---------------------------------------------------------------------------
struct CommandEvent
{
  std::string event_id;           // 信封事件 ID（uuidv7）
  std::string type;               // tts | tts.stop | scene | navigate
  std::string operation_id;       // 幂等键（uuidv7，App 生成、Server 原样转发）
  // type=tts 载荷字段（v0.5.0 播报列表：texts + mode + interval_seconds）
  std::vector<std::string> texts; // tts: 必填；scene/navigate: 可选随动播报（v0.20.0）
  std::string mode;               // sequence 按序循环 | shuffle 随机循环
  int interval_seconds{0};        // 每两条内容之间的播报间隔，恒必填（1..86400）
  // type=scene 载荷字段（v0.17.0：可扩展枚举，当前仅 welcome；
  // v0.22.0 可选 qrcode_png_base64：迎宾二维码图 PNG 的 base64，
  // 机器人转 MP4 后在脸屏常驻显示至超时/新业务）
  std::string scene;
  std::string qrcode_png_base64;  // scene: 可选（PNG base64，1..2730668 字节≈2MB 图）
  // type=navigate 载荷字段（v0.16.0：点位名 + 可选随动播报）
  std::string point_name;         // 位置不透明名（1..64 Unicode 标量，Server 原样转发）
  bool expired{false};            // expires_at 早于当前时间
};

// 解析并校验 down/commands 信封（严格 8 字段、uuid、robot_id/direction 匹配、
// expires_at 不得晚于 occurred_at + 2 分钟）。
// type 不属于 tts/tts.stop/scene/navigate 时返回 nullopt 且 error 为 "ignore"
// （非错误，静默忽略：relocate/ota 等由其他组件处理）。
// expires_at 已过期时 parsed->expired=true，由调用方决定回执/丢弃。
// 文本长度按契约校验（texts 每条 1..2048 字节；point_name 1..64 码点），无本地截断。
// invalid_operation_id / invalid_command_type：可选出参。当信封确认为本机
// type=tts 指令、但报文校验失败且能提取出合法 operation_id 时填充——调用方应
// 回 tts.result(rejected, invalid_request)（契约：Payload 非法但仍能解析出
// operation_id）。scene/navigate 无回执面，不参与。信封本身非法（robot_id
// 不符/解析不出 operation_id）时不填，直接丢弃记日志。
// 成功返回 CommandEvent；失败返回 std::nullopt，error 输出原因。
std::optional<CommandEvent> parse_command_event(
  const std::string & payload, const std::string & robot_id, std::string & error,
  std::string * invalid_operation_id = nullptr,
  std::string * invalid_command_type = nullptr);

// ---------------------------------------------------------------------------
// 上行回执（机器人发布到 {prefix}/{robot_id}/up/commands，QoS 1）：
// 仅 tts.result（v0.6.0）。同一 operation_id 的指令重发必须重发同一回执
// （同 event_id），调用方缓存 envelope_json 原文重发即可。
// scene / navigate / tts.stop 无回执（契约），不得构造。
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
