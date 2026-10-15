// ============================================================================
// guide_pulse 协议层实现：下行指令报文解析/校验、tts.result 回执构造、
// 播报列表轮序生成、时间与 ID 工具。
// 解析框架沿用 tts_demo（工程 /opt/liwei/tts_demo），删 condition 载荷、
// 新增 scene（v0.17.0）与 navigate（v0.16.0 + v0.20.0 可选 texts）载荷。
// ============================================================================
#include "guide_pulse/guide_protocol.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <limits>
#include <random>
#include <regex>

#include <nlohmann/json.hpp>

namespace guide_pulse
{

bool valid_priority_level(int level)
{
  // 与 SDK《接口说明.md》TtsPriorityLevel 档位表一致（vendored .msg 常量
  // 缺 SYSTEM_L7=7，但真机服务端实测受理且回显 level=7，故白名单含 7）
  static constexpr std::array<int, 7> kLevels = {1, 2, 4, 6, 7, 8, 10};
  return std::find(kLevels.begin(), kLevels.end(), level) != kLevels.end();
}

bool valid_priority_weight(int weight)
{
  return weight >= 0 && weight <= 99;
}

namespace
{

using nlohmann::json;

// 宽松 uuidv7 判断（契约 session/boot/event/operation 均为 v7）
bool is_uuid_v7(const std::string & value)
{
  static const std::regex pattern(
    "^[0-9a-f]{8}-[0-9a-f]{4}-7[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$");
  return std::regex_match(value, pattern);
}

// RFC3339 -> 系统时钟（支持 Z / ±hh:mm 时区与小数秒）
bool parse_rfc3339(const std::string & value, std::chrono::system_clock::time_point & out)
{
  std::tm tm_buf{};
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  char fraction[16] = {0};
  char tz = 0;
  int tz_hour = 0, tz_minute = 0;
  const auto parsed = std::sscanf(
    value.c_str(), "%d-%d-%dT%d:%d:%d%15[0-9]%c", &year, &month, &day, &hour,
    &minute, &second, fraction, &tz);
  if (parsed < 6) {
    return false;
  }
  tm_buf.tm_year = year - 1900;
  tm_buf.tm_mon = month - 1;
  tm_buf.tm_mday = day;
  tm_buf.tm_hour = hour;
  tm_buf.tm_min = minute;
  tm_buf.tm_sec = second;
  std::time_t epoch = timegm(&tm_buf);
  if (tz == '+' || tz == '-') {
    const char * tail = value.c_str() + value.size() - 6;
    if (std::sscanf(tail, "%d:%d", &tz_hour, &tz_minute) == 2) {
      const std::time_t offset = (tz_hour * 3600 + tz_minute * 60);
      epoch += (tz == '+') ? -offset : offset;
    }
  }
  out = std::chrono::system_clock::from_time_t(epoch);
  return true;
}

// 系统时钟 -> RFC3339 UTC（毫秒精度）
std::string format_rfc3339(const std::chrono::system_clock::time_point & tp)
{
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    tp.time_since_epoch()).count();
  const std::time_t seconds = static_cast<std::time_t>(ms / 1000);
  const int millis = static_cast<int>(ms % 1000);
  std::tm tm_buf{};
  gmtime_r(&seconds, &tm_buf);
  char buffer[64];
  std::snprintf(
    buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
    tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
    tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, millis);
  return buffer;
}

// UTF-8 码点计数（point_name 契约按 Unicode 标量 1..64 校验，非字节数）
std::size_t utf8_scalar_count(const std::string & value)
{
  std::size_t count = 0;
  for (const unsigned char c : value) {
    if ((c & 0xC0U) != 0x80U) {
      ++count;  // 非续字节即一个码点的起点
    }
  }
  return count;
}

// scene/navigate 的可选随动播报 texts（v0.20.0）：出现时必须 1..20 条、
// 每条 1..2048 字节；不出现返回 true 且不改动 out
bool parse_optional_texts(
  const json & body, std::vector<std::string> & out, std::string & error)
{
  if (!body.contains("texts")) {
    return true;
  }
  const auto & texts = body["texts"];
  if (!texts.is_array()) {
    error = "texts must be an array";
    return false;
  }
  if (texts.empty() || texts.size() > TTS_TEXTS_MAX) {
    error = "texts must contain 1..20 items (contract)";
    return false;
  }
  out.reserve(texts.size());
  for (const auto & item : texts) {
    if (!item.is_string()) {
      error = "texts items must be strings";
      return false;
    }
    const auto text = item.get<std::string>();
    if (text.empty() || text.size() > TTS_TEXT_MAX_BYTES) {
      error = "texts item length must be within 1..2048 (contract)";
      return false;
    }
    out.push_back(std::move(text));
  }
  return true;
}

}  // namespace

bool valid_qr_sha256(const std::string & sha256)
{
  if (sha256.size() != 64U) {
    return false;
  }
  return std::all_of(sha256.begin(), sha256.end(), [](unsigned char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}

std::optional<CommandEvent> parse_command_event(
  const std::string & payload, const std::string & robot_id, std::string & error,
  std::string * invalid_operation_id, std::string * invalid_command_type)
{
  error.clear();
  if (invalid_operation_id != nullptr) {
    invalid_operation_id->clear();
  }
  if (invalid_command_type != nullptr) {
    invalid_command_type->clear();
  }

  json root = json::parse(payload, nullptr, false);
  if (root.is_discarded() || !root.is_object()) {
    error = "command payload is not a valid JSON object";
    return std::nullopt;
  }

  // 契约信封 8 字段，多余/缺失一律拒绝（与 tts_demo require_exact_keys 一致）
  const std::array<const char *, 8> kEnvelopeKeys = {
    "spec_version", "event_id", "robot_id", "direction", "type", "occurred_at",
    "expires_at", "payload"};
  for (const auto * key : kEnvelopeKeys) {
    if (!root.contains(key)) {
      error = std::string("missing field: ") + key;
      return std::nullopt;
    }
  }
  for (const auto & item : root.items()) {
    const bool known = std::find_if(
      kEnvelopeKeys.begin(), kEnvelopeKeys.end(),
      [&item](const char * key) {return item.key() == key;}) != kEnvelopeKeys.end();
    if (!known) {
      error = "unknown field: " + item.key();
      return std::nullopt;
    }
  }

  if (!root["spec_version"].is_string() || root["spec_version"].get<std::string>() != "1.0") {
    error = "unsupported spec_version (only 1.0)";
    return std::nullopt;
  }
  if (!root["event_id"].is_string() || !is_uuid_v7(root["event_id"].get<std::string>())) {
    error = "event_id must be a uuidv7";
    return std::nullopt;
  }
  if (!root["robot_id"].is_string() || root["robot_id"].get<std::string>() != robot_id) {
    error = "robot_id mismatch";
    return std::nullopt;
  }
  if (!root["direction"].is_string() || root["direction"].get<std::string>() != "down") {
    error = "direction must be down";
    return std::nullopt;
  }
  if (!root["type"].is_string()) {
    error = "type must be a string";
    return std::nullopt;
  }
  if (!root["payload"].is_object()) {
    error = "payload must be an object";
    return std::nullopt;
  }

  CommandEvent event;
  event.event_id = root["event_id"].get<std::string>();
  event.type = root["type"].get<std::string>();

  // 本组件处理 tts / tts.stop / scene / navigate；relocate / ota 等由其他组件处理
  if (event.type != "tts" && event.type != "tts.stop" && event.type != "scene" &&
    event.type != "navigate")
  {
    error = "ignore";
    return std::nullopt;
  }

  const auto & body = root["payload"];
  if (!body.contains("operation_id") || !body["operation_id"].is_string() ||
    !is_uuid_v7(body["operation_id"].get<std::string>()))
  {
    // 连 operation_id 都解析不出：契约要求直接丢弃记日志，不回执
    error = "payload.operation_id must be a uuidv7";
    return std::nullopt;
  }
  event.operation_id = body["operation_id"].get<std::string>();

  // 后续校验失败时，若为本机 tts 指令且 operation_id 已合法提取，调用方应回
  // tts.result(rejected, invalid_request)。scene/navigate 无回执面：载荷非法
  // 一律丢弃记日志，不参与该机制
  const auto fill_invalid_receipt_id = [&]() {
      if (invalid_operation_id != nullptr && event.type == "tts") {
        *invalid_operation_id = event.operation_id;
        if (invalid_command_type != nullptr) {
          *invalid_command_type = event.type;
        }
      }
    };

  if (event.type == "tts") {
    // v0.5.0 播报列表载荷：严格 4 字段 operation_id/texts/mode/interval_seconds；
    // 残留旧版 text/loop 字段一律按 unknown fields 拒绝
    const bool has_texts = body.contains("texts") && body["texts"].is_array();
    const bool has_mode = body.contains("mode") && body["mode"].is_string();
    const bool has_interval =
      body.contains("interval_seconds") && body["interval_seconds"].is_number_integer();
    if (!has_texts || !has_mode || !has_interval) {
      fill_invalid_receipt_id();
      error = "tts payload requires texts(array)/mode(string)/interval_seconds(int)";
      return std::nullopt;
    }
    if (body.size() != 4U) {
      fill_invalid_receipt_id();
      error = "tts payload has unknown fields (text/loop removed in v0.5.0)";
      return std::nullopt;
    }
    const auto & texts = body["texts"];
    if (texts.empty() || texts.size() > TTS_TEXTS_MAX) {
      fill_invalid_receipt_id();
      error = "texts must contain 1..20 items (contract)";
      return std::nullopt;
    }
    event.texts.reserve(texts.size());
    for (const auto & item : texts) {
      if (!item.is_string()) {
        fill_invalid_receipt_id();
        error = "texts items must be strings";
        return std::nullopt;
      }
      const auto text = item.get<std::string>();
      // 每条长度按契约 1..2048 校验（UTF-8 字节），不做本地截断
      if (text.empty() || text.size() > TTS_TEXT_MAX_BYTES) {
        fill_invalid_receipt_id();
        error = "texts item length must be within 1..2048 (contract)";
        return std::nullopt;
      }
      event.texts.push_back(std::move(text));
    }
    event.mode = body["mode"].get<std::string>();
    if (event.mode != TTS_MODE_SEQUENCE && event.mode != TTS_MODE_SHUFFLE) {
      fill_invalid_receipt_id();
      error = "mode must be sequence|shuffle";
      return std::nullopt;
    }
    event.interval_seconds = body["interval_seconds"].get<int>();
    if (event.interval_seconds < 1 || event.interval_seconds > 86400) {
      fill_invalid_receipt_id();
      error = "interval_seconds must be within 1..86400";
      return std::nullopt;
    }
  } else if (event.type == "scene") {
    // v0.17.0 场景载荷：operation_id + scene 必填、texts 可选（v0.20.0）、
    // qr_sha256 可选（v0.22.0 P-036 迎宾二维码哈希快照）。合法字段白名单
    // {operation_id, scene, texts, qr_sha256}，其余（含 extra 等
    // 未知字段）一律拒绝。scene 为可扩展枚举，当前契约唯一值 welcome：
    // 未知值属于「未来版本的指令」，本机丢弃记日志（无回执面），不按错误处理
    if (!body.contains("scene") || !body["scene"].is_string()) {
      error = "scene payload requires scene(string)";
      return std::nullopt;
    }
    for (const auto & [key, value] : body.items()) {
      (void)value;
      if (key != "operation_id" && key != "scene" && key != "texts" &&
        key != "qr_sha256")
      {
        error = "scene payload has unknown fields";
        return std::nullopt;
      }
    }
    event.scene = body["scene"].get<std::string>();
    if (event.scene != SCENE_WELCOME) {
      error = "unsupported scene value (only welcome in this version)";
      return std::nullopt;
    }
    if (body.contains("qr_sha256")) {
      if (!body["qr_sha256"].is_string()) {
        error = "qr_sha256 must be string";
        return std::nullopt;
      }
      event.qr_sha256 = body["qr_sha256"].get<std::string>();
      // 契约：64 位小写十六进制（command-event.schema.json pattern）
      if (!valid_qr_sha256(event.qr_sha256)) {
        error = "qr_sha256 must be 64 lowercase hex chars";
        return std::nullopt;
      }
    }
    if (!parse_optional_texts(body, event.texts, error)) {
      return std::nullopt;
    }
  } else if (event.type == "navigate") {
    // v0.16.0 导航载荷：operation_id + point_name 必填、texts 可选（v0.20.0）。
    // 合法字段集合仅 {operation_id, point_name} 或 {operation_id, point_name, texts}
    if (!body.contains("point_name") || !body["point_name"].is_string()) {
      error = "navigate payload requires point_name(string)";
      return std::nullopt;
    }
    const bool has_optional_texts =
      body.size() == 3U && body.contains("texts");
    if (body.size() != 2U && !has_optional_texts) {
      error = "navigate payload has unknown fields";
      return std::nullopt;
    }
    event.point_name = body["point_name"].get<std::string>();
    // 契约按 Unicode 标量（码点）限 1..64，非 UTF-8 字节数（如「3号展台」4 标量 12 字节）
    const auto scalars = utf8_scalar_count(event.point_name);
    if (scalars < 1U || scalars > POINT_NAME_MAX_SCALARS) {
      error = "point_name length must be within 1..64 unicode scalars (contract)";
      return std::nullopt;
    }
    if (!parse_optional_texts(body, event.texts, error)) {
      return std::nullopt;
    }
  } else if (body.size() != 1U) {
    // tts.stop 无回执：payload 只允许 operation_id
    error = "tts.stop payload must contain only operation_id";
    return std::nullopt;
  }

  // 过期判断与 TTL 窗口校验（契约信封规则，四类指令通用）
  std::chrono::system_clock::time_point expires_at;
  if (!root["expires_at"].is_string() ||
    !parse_rfc3339(root["expires_at"].get<std::string>(), expires_at))
  {
    fill_invalid_receipt_id();
    error = "expires_at must be RFC3339";
    return std::nullopt;
  }
  std::chrono::system_clock::time_point occurred_at;
  if (!root["occurred_at"].is_string() ||
    !parse_rfc3339(root["occurred_at"].get<std::string>(), occurred_at))
  {
    fill_invalid_receipt_id();
    error = "occurred_at must be RFC3339";
    return std::nullopt;
  }
  // 契约跨字段语义：expires_at 默认 occurred_at + 60s，不得晚于 + 2 分钟；
  // 越窗事件必须拒绝不得执行
  if (expires_at > occurred_at + std::chrono::seconds(120)) {
    fill_invalid_receipt_id();
    error = "expires_at must not be later than occurred_at + 2 minutes";
    return std::nullopt;
  }
  event.expired = std::chrono::system_clock::now() >= expires_at;

  return event;
}

std::optional<TtsResultReceipt> build_tts_result_event(
  const std::string & robot_id, const std::string & operation_id,
  const std::string & outcome, const std::string & reason)
{
  // 契约：accepted 禁带 reason；rejected 必带 reason 枚举之一
  const bool outcome_ok =
    outcome == TTS_RESULT_ACCEPTED || outcome == TTS_RESULT_REJECTED;
  const std::array<const char *, 3> kReasons = {
    TTS_REASON_COMMAND_EXPIRED, TTS_REASON_INVALID_REQUEST,
    TTS_REASON_PLAYBACK_UNAVAILABLE};
  const bool reason_ok =
    (outcome == TTS_RESULT_ACCEPTED && reason.empty()) ||
    (outcome == TTS_RESULT_REJECTED &&
      std::find(kReasons.begin(), kReasons.end(), reason) != kReasons.end());
  if (!outcome_ok || !reason_ok) {
    return std::nullopt;
  }

  json payload = {
    {"operation_id", operation_id},
    {"outcome", outcome},
  };
  if (outcome == TTS_RESULT_REJECTED) {
    payload["reason"] = reason;
  }

  TtsResultReceipt receipt;
  receipt.event_id = generate_uuid_v7();
  const auto occurred_at = std::chrono::system_clock::now();
  // 事件信封 TTL：与契约示例一致取 60s（仅作 Server 侧过期参考）
  json envelope = {
    {"spec_version", "1.0"},
    {"event_id", receipt.event_id},
    {"robot_id", robot_id},
    {"direction", "up"},
    {"type", "tts.result"},
    {"occurred_at", format_rfc3339(occurred_at)},
    {"expires_at", format_rfc3339(occurred_at + std::chrono::seconds(60))},
    {"payload", std::move(payload)},
  };
  receipt.envelope_json = envelope.dump();
  return receipt;
}

std::vector<std::string> make_round_order(
  const std::vector<std::string> & texts, const std::string & mode)
{
  if (mode != TTS_MODE_SHUFFLE || texts.size() < 2U) {
    // sequence 按原序；单条列表洗牌无意义
    return texts;
  }
  // shuffle：每轮开始前重新洗牌（契约建议），Fisher-Yates
  auto order = texts;
  static thread_local std::mt19937_64 gen(std::random_device{}());
  std::shuffle(order.begin(), order.end(), gen);
  return order;
}

std::string now_rfc3339()
{
  return format_rfc3339(std::chrono::system_clock::now());
}

int estimate_speech_seconds(const std::string & text, int ms_per_char)
{
  // 按语种分档计费（tts_demo 真机实测校准结论，直接沿用）：
  //   - CJK 字符（codepoint >= 0x2E80，含汉字/假名/全角标点）与数字：
  //     ms_per_char（默认 220ms）
  //   - 拉丁字母：~160ms/字母
  //   - 空格与 ASCII 标点不计
  constexpr int kMsPerLatinLetter = 160;
  constexpr std::uint32_t kCjkStart = 0x2E80U;

  int64_t total_ms = 0;
  const auto * p = reinterpret_cast<const unsigned char *>(text.data());
  const auto * end = p + text.size();
  while (p < end) {
    const unsigned char c = *p;
    std::size_t len = 1;
    std::uint32_t cp = c;
    if ((c & 0xE0U) == 0xC0U) {
      len = 2;
      cp = c & 0x1FU;
    } else if ((c & 0xF0U) == 0xE0U) {
      len = 3;
      cp = c & 0x0FU;
    } else if ((c & 0xF8U) == 0xF0U) {
      len = 4;
      cp = c & 0x07U;
    } else if ((c & 0xC0U) == 0x80U) {
      ++p;  // 非法/孤立续字节，跳过
      continue;
    }
    for (std::size_t i = 1; i < len && p + i < end; ++i) {
      cp = (cp << 6) | (p[i] & 0x3FU);
    }
    p += len;

    if (cp < 0x80U) {
      const bool is_letter =
        (cp >= 'a' && cp <= 'z') || (cp >= 'A' && cp <= 'Z');
      const bool is_digit = cp >= '0' && cp <= '9';
      if (is_letter) {
        total_ms += kMsPerLatinLetter;
      } else if (is_digit) {
        total_ms += ms_per_char;  // 数字按音节读，与 CJK 同档
      }
    } else if (cp >= kCjkStart) {
      total_ms += ms_per_char;
    }
  }
  return static_cast<int>((total_ms + 999) / 1000);  // 向上取整
}

std::string generate_uuid_v7()
{
  // UUID v7：48bit Unix 毫秒时间戳 + version 7 + variant 10 + 74bit 随机
  static thread_local std::mt19937_64 gen(std::random_device{}());
  std::uniform_int_distribution<std::uint64_t> dist(0, std::numeric_limits<std::uint64_t>::max());

  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const auto ts = static_cast<std::uint64_t>(ms);
  const std::uint64_t rand_a = dist(gen);
  const std::uint64_t rand_b = dist(gen);

  std::uint8_t b[16];
  b[0] = static_cast<std::uint8_t>(ts >> 40);
  b[1] = static_cast<std::uint8_t>(ts >> 32);
  b[2] = static_cast<std::uint8_t>(ts >> 24);
  b[3] = static_cast<std::uint8_t>(ts >> 16);
  b[4] = static_cast<std::uint8_t>(ts >> 8);
  b[5] = static_cast<std::uint8_t>(ts);
  b[6] = static_cast<std::uint8_t>((rand_a >> 56) & 0x0FU) | 0x70U;  // version 7
  b[7] = static_cast<std::uint8_t>(rand_a >> 48);
  b[8] = static_cast<std::uint8_t>((rand_b >> 56) & 0x3FU) | 0x80U;  // variant 10xx
  b[9] = static_cast<std::uint8_t>(rand_b >> 48);
  b[10] = static_cast<std::uint8_t>(rand_b >> 40);
  b[11] = static_cast<std::uint8_t>(rand_b >> 32);
  b[12] = static_cast<std::uint8_t>(rand_b >> 24);
  b[13] = static_cast<std::uint8_t>(rand_b >> 16);
  b[14] = static_cast<std::uint8_t>(rand_b >> 8);
  b[15] = static_cast<std::uint8_t>(rand_b);

  char buffer[40];
  std::snprintf(
    buffer, sizeof(buffer),
    "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
    b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
    b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
  return buffer;
}

std::string topic_for(
  const std::string & prefix, const std::string & robot_id, const std::string & suffix)
{
  std::string topic = prefix;
  while (!topic.empty() && topic.back() == '/') {
    topic.pop_back();
  }
  return topic + "/" + robot_id + "/" + suffix;
}

}  // namespace guide_pulse
