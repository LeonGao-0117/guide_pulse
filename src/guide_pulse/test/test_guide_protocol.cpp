// ============================================================================
// guide_pulse 协议层单元测试（无 ROS 依赖）
// 覆盖：down/commands 五类指令解析/校验（tts v0.5.0 / tts.stop / scene
// v0.17.0+v0.20.0 / navigate v0.16.0+v0.20.0 / navigate.stop v0.23.0）、
// tts.result 回执构造（v0.6.0）、播报列表轮序、优先级档位校验、工具函数。
// 框架与夹具沿用 tts_demo test_tts_protocol.cpp，删 condition 用例、
// 新增 scene / navigate 用例。
// ============================================================================
#include <algorithm>
#include <chrono>
#include <ctime>
#include <regex>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "guide_pulse/guide_protocol.hpp"

namespace
{

using nlohmann::json;

TEST(GuideProtocolTest, PriorityValidation)
{
  // SYSTEM_L7=7：vendored .msg 常量缺失但真机服务端受理（is_success=true
  // 且回显 level=7），故为合法档位（迎宾/讲解防唤醒打断用，见 design.md）
  for (const int level : {1, 2, 4, 6, 7, 8, 10}) {
    EXPECT_TRUE(guide_pulse::valid_priority_level(level)) << "level=" << level;
  }
  for (const int level : {0, 3, 5, 9, 11, -1, 100}) {
    EXPECT_FALSE(guide_pulse::valid_priority_level(level)) << "level=" << level;
  }
  EXPECT_TRUE(guide_pulse::valid_priority_weight(0));
  EXPECT_TRUE(guide_pulse::valid_priority_weight(99));
  EXPECT_FALSE(guide_pulse::valid_priority_weight(-1));
  EXPECT_FALSE(guide_pulse::valid_priority_weight(100));
}

TEST(GuideProtocolTest, TopicForJoinsParts)
{
  EXPECT_EQ(
    guide_pulse::topic_for("robotCar/v1/robots", "robot-001", "down/commands"),
    "robotCar/v1/robots/robot-001/down/commands");
  EXPECT_EQ(
    guide_pulse::topic_for("robotCar/v1/robots", "robot-001", "up/commands"),
    "robotCar/v1/robots/robot-001/up/commands");
  // 尾部斜杠容忍
  EXPECT_EQ(
    guide_pulse::topic_for("robotCar/v1/robots/", "r1", "presence"),
    "robotCar/v1/robots/r1/presence");
}

TEST(GuideProtocolTest, NowRfc3339Format)
{
  const auto stamp = guide_pulse::now_rfc3339();
  // 形如 2026-09-23T10:00:00.123Z
  ASSERT_GE(stamp.size(), 24u);
  EXPECT_EQ(stamp[4], '-');
  EXPECT_EQ(stamp[7], '-');
  EXPECT_EQ(stamp[10], 'T');
  EXPECT_EQ(stamp.back(), 'Z');
}

// 后端契约：presence/heartbeat 的 session_id/boot_id 必须为 uuidv7
TEST(GuideProtocolTest, GenerateUuidV7MatchesContractPattern)
{
  const std::regex pattern(
    "^[0-9a-f]{8}-[0-9a-f]{4}-7[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$");
  const auto a = guide_pulse::generate_uuid_v7();
  const auto b = guide_pulse::generate_uuid_v7();
  EXPECT_TRUE(std::regex_match(a, pattern)) << a;
  EXPECT_TRUE(std::regex_match(b, pattern)) << b;
  EXPECT_NE(a, b);
  // 时间戳有序：先后生成的 uuidv7 字典序非降（同毫秒内随机段无序，仅比较前 13 位）
  EXPECT_LE(a.substr(0, 13), b.substr(0, 13));
}

// 动态时间夹具：默认 occurred_at=当前时刻、expires_at=+60s（契约默认 TTL），
// 「未过期」断言不受真实时钟影响；过期/越窗用例显式传入钉死的时间戳。
std::string rfc3339_offset_now(int seconds_ahead)
{
  const auto t = std::chrono::system_clock::to_time_t(
    std::chrono::system_clock::now() + std::chrono::seconds(seconds_ahead));
  std::tm tm_buf{};
  gmtime_r(&t, &tm_buf);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm_buf);
  return buffer;
}

std::string build_command_json(
  const std::string & type, const std::string & payload_json,
  const std::string & expires_at = "", const std::string & robot_id = "robot-001",
  const std::string & occurred_at = "")
{
  return R"({"spec_version":"1.0","event_id":"01a0cea0-1111-7222-8333-444455556666",)"
         R"("robot_id":")" + robot_id +
         R"(","direction":"down","type":")" + type +
         R"(","occurred_at":")" + (occurred_at.empty() ? rfc3339_offset_now(0) : occurred_at) +
         R"(","expires_at":")" + (expires_at.empty() ? rfc3339_offset_now(60) : expires_at) +
         R"(","payload":)" + payload_json + "}";
}

constexpr const char * kOperationId =
  "01a0cea0-2222-7333-8444-555566667777";

// ---------------------------------------------------------------------------
// type=tts（v0.5.0 播报列表）
// ---------------------------------------------------------------------------
TEST(CommandEventTest, ParseValidTtsSequencePlaylist)
{
  const auto payload = build_command_json(
    "tts",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","texts":["该吃药了","量血压了","多喝水"],"mode":"sequence","interval_seconds":30})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->type, "tts");
  EXPECT_EQ(event->operation_id, kOperationId);
  ASSERT_EQ(event->texts.size(), 3U);
  EXPECT_EQ(event->texts[0], "该吃药了");
  EXPECT_EQ(event->mode, "sequence");
  EXPECT_EQ(event->interval_seconds, 30);
  EXPECT_FALSE(event->expired);
}

TEST(CommandEventTest, ParseTtsStop)
{
  const auto payload = build_command_json(
    "tts.stop", R"({"operation_id":")" + std::string(kOperationId) + R"("})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->type, "tts.stop");
  EXPECT_EQ(event->operation_id, kOperationId);
}

TEST(CommandEventTest, ExpiredEventFlagged)
{
  const auto payload = build_command_json(
    "tts.stop", R"({"operation_id":")" + std::string(kOperationId) + R"("})",
    rfc3339_offset_now(-30), "robot-001", rfc3339_offset_now(-90));
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_TRUE(event->expired);
}

TEST(CommandEventTest, NonGuideTypeIgnored)
{
  // ota.start 等由其他组件处理：error=="ignore" 静默忽略（relocate 自
  // 2026-10-17 起由本组件处理，见 RelocateEventTest）
  for (const auto * type : {"ota.start", "unknown.thing"}) {
    const auto payload = build_command_json(
      type, R"({"operation_id":")" + std::string(kOperationId) + R"("})");
    std::string error;
    const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
    EXPECT_FALSE(event.has_value()) << type;
    EXPECT_EQ(error, "ignore") << type;
  }
}

TEST(CommandEventTest, RejectsInvalidEnvelopes)
{
  const std::string valid_payload = R"({"operation_id":")" +
    std::string(kOperationId) + R"("})";
  const std::string valid_body =
    build_command_json("tts.stop", valid_payload);

  // 非 JSON / 非对象
  std::string error;
  EXPECT_FALSE(guide_pulse::parse_command_event("not-json", "robot-001", error));
  EXPECT_FALSE(guide_pulse::parse_command_event("[1,2]", "robot-001", error));

  // robot_id 不匹配 / direction 错误 / 非法 uuid
  auto replaced = [](std::string body, const std::string & from,
    const std::string & to) {
      auto pos = body.find(from);
      return body.replace(pos, from.size(), to);
    };
  EXPECT_FALSE(guide_pulse::parse_command_event(
    replaced(valid_body, "\"robot-001\"", "\"robot-002\""), "robot-001", error));
  EXPECT_FALSE(guide_pulse::parse_command_event(
    replaced(valid_body, "\"down\"", "\"up\""), "robot-001", error));
  EXPECT_FALSE(guide_pulse::parse_command_event(
    replaced(valid_body, "01a0cea0-1111-7222-8333-444455556666",
      "not-a-uuid"), "robot-001", error));

  // 信封未知字段
  json with_extra = json::parse(valid_body);
  with_extra["extra"] = 1;
  EXPECT_FALSE(guide_pulse::parse_command_event(with_extra.dump(), "robot-001", error));
}

TEST(CommandEventTest, RejectsLegacyLoopField)
{
  // v0.5.0 废除的 text/loop 残留字段一律拒绝，且填 invalid_operation_id 回执通道
  const auto payload = build_command_json(
    "tts",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","texts":["hi"],"mode":"sequence","interval_seconds":30,"loop":true})");
  std::string error;
  std::string invalid_op;
  std::string invalid_type;
  const auto event = guide_pulse::parse_command_event(
    payload, "robot-001", error, &invalid_op, &invalid_type);
  EXPECT_FALSE(event.has_value());
  EXPECT_EQ(invalid_op, kOperationId);
  EXPECT_EQ(invalid_type, "tts");
}

TEST(CommandEventTest, RejectsInvalidPlaylist)
{
  // 空 texts / 越界 interval / 非法 mode
  const std::vector<std::string> bad_payloads = {
    R"({"operation_id":")" + std::string(kOperationId) + R"(","texts":[],"mode":"sequence","interval_seconds":30})",
    R"({"operation_id":")" + std::string(kOperationId) + R"(","texts":["ok"],"mode":"random","interval_seconds":30})",
    R"({"operation_id":")" + std::string(kOperationId) + R"(","texts":["ok"],"mode":"sequence","interval_seconds":0})",
    R"({"operation_id":")" + std::string(kOperationId) + R"(","texts":["ok"],"mode":"sequence","interval_seconds":86401})",
  };
  for (const auto & body : bad_payloads) {
    std::string error;
    std::string invalid_op;
    EXPECT_FALSE(guide_pulse::parse_command_event(
      build_command_json("tts", body), "robot-001", error, &invalid_op))
      << body;
    EXPECT_EQ(invalid_op, kOperationId) << body;
  }
}

TEST(CommandEventTest, RejectsExpiresBeyondTwoMinuteWindow)
{
  // 契约跨字段语义：expires_at 不得晚于 occurred_at + 2 分钟
  const auto payload = build_command_json(
    "tts.stop", R"({"operation_id":")" + std::string(kOperationId) + R"("})",
    rfc3339_offset_now(180), "robot-001", rfc3339_offset_now(0));
  std::string error;
  EXPECT_FALSE(guide_pulse::parse_command_event(payload, "robot-001", error));
}

// ---------------------------------------------------------------------------
// type=scene（v0.17.0 迎宾 + v0.20.0 可选 texts）
// ---------------------------------------------------------------------------
TEST(SceneEventTest, ParseWelcomeWithTexts)
{
  const auto payload = build_command_json(
    "scene",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","scene":"welcome","texts":["欢迎光临","请随我来"]})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->type, "scene");
  EXPECT_EQ(event->scene, "welcome");
  ASSERT_EQ(event->texts.size(), 2U);
  EXPECT_EQ(event->texts[0], "欢迎光临");
  // 合法载荷不填回执通道（invalid_operation_id 仅校验失败时填充）
  std::string invalid_op;
  EXPECT_TRUE(invalid_op.empty());
}

TEST(SceneEventTest, ParseWelcomeWithoutTexts)
{
  const auto payload = build_command_json(
    "scene", R"({"operation_id":")" + std::string(kOperationId) + R"(","scene":"welcome"})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->scene, "welcome");
  EXPECT_TRUE(event->texts.empty());  // 缺省：迎宾词回落本地默认
}

TEST(SceneEventTest, RejectsUnknownSceneValue)
{
  // scene 为可扩展枚举，当前唯一值 welcome：未知值丢弃（未来版本指令）
  const auto payload = build_command_json(
    "scene", R"({"operation_id":")" + std::string(kOperationId) + R"(","scene":"farewell"})");
  std::string error;
  std::string invalid_op;
  EXPECT_FALSE(guide_pulse::parse_command_event(
    payload, "robot-001", error, &invalid_op));
  EXPECT_NE(error.find("unsupported scene"), std::string::npos);
  EXPECT_TRUE(invalid_op.empty());  // 无回执面
}

// ---------------------------------------------------------------------------
// scene.qr_sha256（v0.22.0 P-036 迎宾二维码哈希快照：64 位小写十六进制）
// ---------------------------------------------------------------------------
TEST(SceneQrSha256Test, ParsesValidSha)
{
  const std::string sha =
    "9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08";
  const auto payload = build_command_json(
    "scene", R"({"operation_id":")" + std::string(kOperationId) +
    R"(","scene":"welcome","texts":["欢迎光临"],"qr_sha256":")" + sha +
    R"("})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->qr_sha256, sha);
}

TEST(SceneQrSha256Test, AbsentMeansNoQrcode)
{
  // 省略 qr_sha256（App 未上传/已清除）：合法，机器人本次不显示二维码
  const auto payload = build_command_json(
    "scene", R"({"operation_id":")" + std::string(kOperationId) + R"(","scene":"welcome"})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_TRUE(event->qr_sha256.empty());
}

TEST(SceneQrSha256Test, RejectsInvalidSha)
{
  const std::string good64(64, 'a');
  const std::vector<std::string> bad_shas = {
    std::string(63, 'a'),                  // 长度 63
    std::string(65, 'a'),                  // 长度 65
    std::string(64, 'A'),                  // 大写十六进制
    std::string(64, 'g'),                  // 非 hex 字符
    "",                                    // 空串（携带即非法，省略须整体缺字段）
  };
  for (const auto & sha : bad_shas) {
    const auto payload = build_command_json(
      "scene", R"({"operation_id":")" + std::string(kOperationId) +
      R"(","scene":"welcome","qr_sha256":")" + sha + R"("})");
    std::string error;
    const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
    EXPECT_FALSE(event.has_value()) << "sha='" << sha << "'";
    if (!event.has_value()) {
      EXPECT_NE(error.find("qr_sha256"), std::string::npos) << error;
    }
  }
  // 非字符串类型
  const auto payload = build_command_json(
    "scene", R"({"operation_id":")" + std::string(kOperationId) +
    R"(","scene":"welcome","qr_sha256":123})");
  std::string error;
  EXPECT_FALSE(guide_pulse::parse_command_event(payload, "robot-001", error));
}

TEST(SceneEventTest, RejectsInvalidScenePayloads)
{
  const std::vector<std::string> bad_payloads = {
    R"({"operation_id":")" + std::string(kOperationId) + R"("})",                       // 缺 scene
    R"({"scene":"welcome"})",                                                          // 缺 operation_id
    R"({"operation_id":")" + std::string(kOperationId) + R"(","scene":123})",          // scene 非字符串
    R"({"operation_id":")" + std::string(kOperationId) + R"(","scene":"welcome","texts":[]})",  // 空 texts
    R"({"operation_id":")" + std::string(kOperationId) + R"(","scene":"welcome","extra":1})",   // 未知字段
  };
  for (const auto & body : bad_payloads) {
    std::string error;
    EXPECT_FALSE(guide_pulse::parse_command_event(
      build_command_json("scene", body), "robot-001", error)) << body;
  }
}

// ---------------------------------------------------------------------------
// type=navigate（v0.16.0 导航 + v0.20.0 可选 texts）
// ---------------------------------------------------------------------------
TEST(NavigateEventTest, ParseWithTexts)
{
  const auto payload = build_command_json(
    "navigate",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","point_name":"3号展台","texts":["这是我们的主打车型","续航一千公里"]})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->type, "navigate");
  EXPECT_EQ(event->point_name, "3号展台");
  ASSERT_EQ(event->texts.size(), 2U);
}

TEST(NavigateEventTest, ParseWithoutTexts)
{
  const auto payload = build_command_json(
    "navigate", R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":"充电桩A"})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->point_name, "充电桩A");
  EXPECT_TRUE(event->texts.empty());
}

TEST(NavigateEventTest, PointNameLimitedByUnicodeScalars)
{
  // 契约按 Unicode 标量 1..64（非字节数）：65 个汉字 = 195 字节，合法；
  // 65 个汉字 = 65 标量 > 64，非法。65 个 ASCII 字母 = 65 标量，非法
  const std::string ascii65(65, 'a');
  const std::string han64 = std::string(64 * 3, '\x00');  // 占位，下方构造
  std::string han;
  for (int i = 0; i < 65; ++i) {
    han += "\xe4\xb8\x80";  // 「一」U+4E00
  }
  const std::string han_64 = han.substr(0, 64 * 3);  // 64 个汉字 = 64 标量
  (void)han64;

  std::string error;
  // 65 汉字（65 标量、195 字节）拒绝
  EXPECT_FALSE(guide_pulse::parse_command_event(
    build_command_json(
      "navigate",
      R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":")" + han +
      R"("})"),
    "robot-001", error));
  // 64 汉字（64 标量、192 字节）接受——字节超 64 但标量未超
  EXPECT_TRUE(guide_pulse::parse_command_event(
    build_command_json(
      "navigate",
      R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":")" + han_64 +
      R"("})"),
    "robot-001", error));
  // 65 ASCII 字母拒绝
  EXPECT_FALSE(guide_pulse::parse_command_event(
    build_command_json(
      "navigate",
      R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":")" + ascii65 +
      R"("})"),
    "robot-001", error));
}

TEST(NavigateEventTest, RejectsInvalidNavigatePayloads)
{
  const std::vector<std::string> bad_payloads = {
    R"({"operation_id":")" + std::string(kOperationId) + R"(","texts":["a"]})",   // 缺 point_name
    R"({"point_name":"东门"})",                                                   // 缺 operation_id
    R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":""})",  // 空点位名
    R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":"x","texts":[]})",  // 空 texts
    R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":"x","speed":2})",   // 未知字段
  };
  for (const auto & body : bad_payloads) {
    std::string error;
    EXPECT_FALSE(guide_pulse::parse_command_event(
      build_command_json("navigate", body), "robot-001", error)) << body;
  }
}

// ---------------------------------------------------------------------------
// type=relocate（v0.21.0 地图重定位，2026-10-17 接入；载荷形状与 navigate
// 完全一致：operation_id + point_name 必填、texts 可选）
// ---------------------------------------------------------------------------
TEST(RelocateEventTest, ParseWithTexts)
{
  const auto payload = build_command_json(
    "relocate",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","point_name":"A","texts":["A点重定位成功"]})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->type, "relocate");
  EXPECT_EQ(event->point_name, "A");
  ASSERT_EQ(event->texts.size(), 1U);
  EXPECT_EQ(event->texts[0], "A点重定位成功");
}

TEST(RelocateEventTest, ParseWithoutTexts)
{
  const auto payload = build_command_json(
    "relocate", R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":"东门路口"})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->type, "relocate");
  EXPECT_EQ(event->point_name, "东门路口");
  EXPECT_TRUE(event->texts.empty());
}

TEST(RelocateEventTest, RejectsInvalidPayloads)
{
  const std::vector<std::string> bad_payloads = {
    R"({"operation_id":")" + std::string(kOperationId) + R"(","texts":["a"]})",   // 缺 point_name
    R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":""})",  // 空点位名
    R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":"A","texts":[]})",  // 空 texts
    R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":"A","mode":"sequence"})",  // 未知字段
  };
  for (const auto & body : bad_payloads) {
    std::string error;
    std::string invalid_op;
    std::string invalid_type;
    EXPECT_FALSE(guide_pulse::parse_command_event(
      build_command_json("relocate", body), "robot-001", error,
      &invalid_op, &invalid_type)) << body;
    // 载荷非法但能解析出 operation_id：填 invalid 通道回 rejected/invalid_request
    EXPECT_EQ(invalid_op, kOperationId) << body;
    EXPECT_EQ(invalid_type, "relocate") << body;
  }
}

// ---------------------------------------------------------------------------
// type=navigate.stop（v0.23.0 停止导航，与 tts.stop 同构仅 operation_id）
// ---------------------------------------------------------------------------
TEST(NavigateStopEventTest, ParsesMinimalPayload)
{
  const auto payload = build_command_json(
    "navigate.stop", R"({"operation_id":")" + std::string(kOperationId) + R"("})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->type, "navigate.stop");
  EXPECT_EQ(event->operation_id, kOperationId);
  EXPECT_FALSE(event->expired);
}

TEST(NavigateStopEventTest, RejectsExtraPayloadFields)
{
  // 契约 navigateStopPayload：additionalProperties=false，仅 operation_id
  //（不带 point_name/texts——停止当前导航无需知道目标）
  const auto payload = build_command_json(
    "navigate.stop",
    R"({"operation_id":")" + std::string(kOperationId) + R"(","point_name":"东门"})");
  std::string error;
  EXPECT_FALSE(guide_pulse::parse_command_event(payload, "robot-001", error));
  EXPECT_NE(error.find("only operation_id"), std::string::npos);
}

// ---------------------------------------------------------------------------
// type=point_map（v0.25.0 P-039 点位映射表全量下发）
// ---------------------------------------------------------------------------
TEST(PointMapEventTest, ParsesValidPointMap)
{
  // 现场样例形态：A..E 五点（比亚迪/腾势车型展位）
  const auto payload = build_command_json(
    "point_map",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"比亚迪ATTO 1"},)"
    R"({"code":"B","name":"比亚迪 M9 DM-i"},{"code":"C","name":"腾势 B5"},)"
    R"({"code":"D","name":"腾势 D9"},{"code":"E","name":"腾势 Z9 GT"}]})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  EXPECT_EQ(event->type, "point_map");
  EXPECT_EQ(event->operation_id, kOperationId);
  ASSERT_EQ(event->points.size(), 5U);
  EXPECT_EQ(event->points[0].code, "A");
  EXPECT_EQ(event->points[0].name, "比亚迪ATTO 1");
  EXPECT_EQ(event->points[4].code, "E");
  EXPECT_EQ(event->points[4].name, "腾势 Z9 GT");
  EXPECT_FALSE(event->expired);
}

TEST(PointMapEventTest, ParsesSinglePoint)
{
  const auto payload = build_command_json(
    "point_map",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"P001","name":"东门路口"}]})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  ASSERT_EQ(event->points.size(), 1U);
  EXPECT_EQ(event->points[0].code, "P001");
}

// v0.28.0（P-042）条目可选播放文本 text：带 text 条目解析出原值，
// 不带 text 条目保持 nullopt（省略 = 该点位无播放文本，两态共存合法）
TEST(PointMapEventTest, ParsesOptionalTextEntries)
{
  const auto payload = build_command_json(
    "point_map",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"比亚迪ætəʊ 1","text":"这就是比亚迪ætəʊ 1的展车了"},)"
    R"({"code":"B","name":"比亚迪ætəʊ 3"}]})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  ASSERT_EQ(event->points.size(), 2U);
  EXPECT_EQ(
    event->points[0].text,
    std::optional<std::string>("这就是比亚迪ætəʊ 1的展车了"));
  EXPECT_FALSE(event->points[1].text.has_value());
}

// v0.28.0（P-042）text 非法反例：空串（minLength 1，省略与空串不等价）、
// 非字符串、超 2048 Unicode 标量——均按 invalid_request 拒并填回执通道
TEST(PointMapEventTest, RejectsInvalidTextEntries)
{
  const std::string too_long(2049, 'a');
  const std::vector<std::string> bad_bodies = {
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"x","text":""}]})",                // 空串
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"x","text":123}]})",               // 非字符串
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"x","text":null}]})",              // JSON null（非省略）
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"x","text":")" + too_long + R"("}]})",  // 超 2048 标量
  };
  for (const auto & body : bad_bodies) {
    std::string error;
    std::string invalid_op;
    std::string invalid_type;
    EXPECT_FALSE(guide_pulse::parse_command_event(
      build_command_json("point_map", body), "robot-001", error, &invalid_op,
      &invalid_type)) << body;
    EXPECT_NE(error.find("text"), std::string::npos) << body;
    EXPECT_EQ(invalid_op, kOperationId) << body;
    EXPECT_EQ(invalid_type, "point_map") << body;
  }
}

// 恰 2048 Unicode 标量的 text 合法（边界值；中文按码点计非字节）
TEST(PointMapEventTest, AcceptsTextAtScalarBoundary)
{
  const std::string text_2048(2048, 'a');
  const auto payload = build_command_json(
    "point_map",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"x","text":")" + text_2048 + R"("}]})");
  std::string error;
  const auto event = guide_pulse::parse_command_event(payload, "robot-001", error);
  ASSERT_TRUE(event.has_value()) << error;
  ASSERT_EQ(event->points.size(), 1U);
  ASSERT_TRUE(event->points[0].text.has_value());
  EXPECT_EQ(event->points[0].text->size(), 2048U);
}

TEST(PointMapEventTest, RejectsInvalidPointMapPayloads)
{
  // 结构反例：缺 points / 空 points / 条目多字段 / 缺 name / 非法类型
  const std::vector<std::string> bad_payloads = {
    R"({"operation_id":")" + std::string(kOperationId) + R"("})",                 // 缺 points
    R"({"operation_id":")" + std::string(kOperationId) + R"(","points":[]})",     // 空 points
    R"({"operation_id":")" + std::string(kOperationId) + R"(","points":"A"})",    // points 非数组
    R"({"operation_id":")" + std::string(kOperationId) + R"(","points":[{}]})",   // 条目缺字段
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"x","extra":1}]})",                          // 条目未知字段
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":123}]})",                                    // name 非字符串
    R"({"operation_id":")" + std::string(kOperationId) + R"(","points":[],"extra":1})",  // 顶层未知字段
  };
  for (const auto & body : bad_payloads) {
    std::string error;
    std::string invalid_op;
    std::string invalid_type;
    EXPECT_FALSE(guide_pulse::parse_command_event(
      build_command_json("point_map", body), "robot-001", error, &invalid_op,
      &invalid_type)) << body;
    // 结构非法但 operation_id 已解析：填回执通道（rejected/invalid_request）
    EXPECT_EQ(invalid_op, kOperationId) << body;
    EXPECT_EQ(invalid_type, "point_map") << body;
  }
}

TEST(PointMapEventTest, RejectsDuplicateCodes)
{
  // code 表内唯一是语义校验（Schema 无法表达）：Robot 侧纵深防御，
  // 按契约回 rejected/invalid_request
  const auto payload = build_command_json(
    "point_map",
    R"({"operation_id":")" + std::string(kOperationId) +
    R"(","points":[{"code":"A","name":"x"},{"code":"A","name":"y"}]})");
  std::string error;
  std::string invalid_op;
  std::string invalid_type;
  EXPECT_FALSE(guide_pulse::parse_command_event(
    payload, "robot-001", error, &invalid_op, &invalid_type));
  EXPECT_NE(error.find("unique"), std::string::npos);
  EXPECT_EQ(invalid_op, kOperationId);
  EXPECT_EQ(invalid_type, "point_map");
}

TEST(PointMapEventTest, RejectsLengthViolations)
{
  // code 1..32 标量、name 1..64 标量（Unicode 标量计，同 point_name）
  std::string han33;
  for (int i = 0; i < 33; ++i) {
    han33 += "\xe4\xb8\x80";  // 「一」
  }
  std::string han65;
  for (int i = 0; i < 65; ++i) {
    han65 += "\xe4\xb8\x80";
  }
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"code 超长（33 汉字）",
      R"({"operation_id":")" + std::string(kOperationId) + R"(","points":[{"code":")" +
      han33 + R"(","name":"x"}]})"},
    {"name 超长（65 汉字）",
      R"({"operation_id":")" + std::string(kOperationId) + R"(","points":[{"code":"A","name":")" +
      han65 + R"("}]})"},
    {"code 为空",
      R"({"operation_id":")" + std::string(kOperationId) + R"(","points":[{"code":"","name":"x"}]})"},
    {"name 为空",
      R"({"operation_id":")" + std::string(kOperationId) + R"(","points":[{"code":"A","name":""}]})"},
  };
  for (const auto & [label, body] : cases) {
    std::string error;
    EXPECT_FALSE(guide_pulse::parse_command_event(
      build_command_json("point_map", body), "robot-001", error)) << label;
  }
}

TEST(PointMapEventTest, PointsOver256Rejected)
{
  // points 上限 256 条（契约 maxItems）
  std::string body = R"({"operation_id":")" + std::string(kOperationId) + R"(","points":[)";
  for (int i = 0; i < 257; ++i) {
    if (i > 0) {
      body += ",";
    }
    body += "{\"code\":\"P" + std::to_string(i) + "\",\"name\":\"n\"}";
  }
  body += "]}";
  std::string error;
  EXPECT_FALSE(guide_pulse::parse_command_event(
    build_command_json("point_map", body), "robot-001", error));
  EXPECT_NE(error.find("1..256"), std::string::npos);
}

// ---------------------------------------------------------------------------
// 播报时长估算（沿用 tts_demo 真机校准结论）
// ---------------------------------------------------------------------------
TEST(GuideProtocolTest, EstimateSpeechSecondsByCharacterCount)
{
  // 10 个汉字 ≈ 10 * 220ms = 2.2s -> 向上取整 3s
  EXPECT_EQ(guide_pulse::estimate_speech_seconds("一二三四五六七八九十"), 3);
  // 空串 0 秒
  EXPECT_EQ(guide_pulse::estimate_speech_seconds(""), 0);
}

// ---------------------------------------------------------------------------
// 播放轮序（sequence 原序 / shuffle 重洗）
// ---------------------------------------------------------------------------
TEST(MakeRoundOrderTest, SequenceKeepsOrder)
{
  const auto order = guide_pulse::make_round_order({"a", "b", "c"}, "sequence");
  EXPECT_EQ(order, (std::vector<std::string>{"a", "b", "c"}));
}

TEST(MakeRoundOrderTest, ShuffleIsPermutation)
{
  const std::vector<std::string> texts{"a", "b", "c", "d", "e"};
  const auto order = guide_pulse::make_round_order(texts, "shuffle");
  ASSERT_EQ(order.size(), texts.size());
  std::vector<std::string> sorted_order = order;
  std::sort(sorted_order.begin(), sorted_order.end());
  EXPECT_EQ(sorted_order, texts);  // 顺序可变，集合不变
}

TEST(MakeRoundOrderTest, SingleItemShuffleNoop)
{
  const auto order = guide_pulse::make_round_order({"only"}, "shuffle");
  EXPECT_EQ(order, (std::vector<std::string>{"only"}));
}

// ---------------------------------------------------------------------------
// tts.result 回执构造（v0.6.0；scene/navigate 无回执面）
// ---------------------------------------------------------------------------
TEST(BuildTtsResultTest, AcceptedWithoutReason)
{
  const auto receipt = guide_pulse::build_tts_result_event(
    "robot-001", kOperationId, "accepted");
  ASSERT_TRUE(receipt.has_value());
  const auto envelope = json::parse(receipt->envelope_json);
  EXPECT_EQ(envelope["direction"], "up");
  EXPECT_EQ(envelope["type"], "tts.result");
  EXPECT_EQ(envelope["payload"]["operation_id"], kOperationId);
  EXPECT_EQ(envelope["payload"]["outcome"], "accepted");
  EXPECT_FALSE(envelope["payload"].contains("reason"));
}

TEST(BuildTtsResultTest, RejectedRequiresEnumReason)
{
  for (const char * reason :
    {"command_expired", "invalid_request", "playback_unavailable"})
  {
    const auto receipt = guide_pulse::build_tts_result_event(
      "robot-001", kOperationId, "rejected", reason);
    ASSERT_TRUE(receipt.has_value()) << reason;
    EXPECT_EQ(json::parse(receipt->envelope_json)["payload"]["reason"], reason);
  }
}

TEST(BuildTtsResultTest, RejectsInvalidArguments)
{
  // accepted 带 reason / rejected 缺 reason / 未知 outcome / 未知 reason
  EXPECT_FALSE(guide_pulse::build_tts_result_event(
    "robot-001", kOperationId, "accepted", "command_expired"));
  EXPECT_FALSE(guide_pulse::build_tts_result_event(
    "robot-001", kOperationId, "rejected"));
  EXPECT_FALSE(guide_pulse::build_tts_result_event(
    "robot-001", kOperationId, "maybe"));
  EXPECT_FALSE(guide_pulse::build_tts_result_event(
    "robot-001", kOperationId, "rejected", "not_a_reason"));
}

TEST(BuildTtsResultTest, EachBuildGetsFreshEventId)
{
  const auto a = guide_pulse::build_tts_result_event("robot-001", kOperationId, "accepted");
  const auto b = guide_pulse::build_tts_result_event("robot-001", kOperationId, "accepted");
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_NE(a->event_id, b->event_id);
}

// ---------------------------------------------------------------------------
// navigate.result / relocate.result / scene.result / point_map.result
// 一次性最终结果回执构造（v0.24.0 P-038 / v0.25.0 P-039）
// ---------------------------------------------------------------------------
using guide_pulse::ACTION_RESULT_FAILED;
using guide_pulse::ACTION_RESULT_REJECTED;
using guide_pulse::ACTION_RESULT_SUCCESS;
using guide_pulse::NAVIGATE_RESULT_TYPE;
using guide_pulse::RELOCATE_RESULT_TYPE;
using guide_pulse::RESULT_REASON_ABORTED;
using guide_pulse::RESULT_REASON_BUSY;
using guide_pulse::RESULT_REASON_LOCALIZATION_FAILED;
using guide_pulse::RESULT_REASON_LOCALIZATION_LOST;
using guide_pulse::RESULT_REASON_NAVIGATION_FAILED;
using guide_pulse::RESULT_REASON_POINT_NOT_FOUND;
using guide_pulse::RESULT_REASON_SCENE_FAILED;
using guide_pulse::SCENE_RESULT_TYPE;
using guide_pulse::TTS_REASON_COMMAND_EXPIRED;
using guide_pulse::TTS_REASON_INVALID_REQUEST;

TEST(BuildActionResultTest, SuccessForbidsReason)
{
  for (const char * type :
    {NAVIGATE_RESULT_TYPE, RELOCATE_RESULT_TYPE, SCENE_RESULT_TYPE})
  {
    const auto receipt = guide_pulse::build_action_result_event(
      "robot-001", type, kOperationId, ACTION_RESULT_SUCCESS);
    ASSERT_TRUE(receipt.has_value()) << type;
    const auto envelope = json::parse(receipt->envelope_json);
    EXPECT_EQ(envelope["direction"], "up");
    EXPECT_EQ(envelope["type"], type);
    EXPECT_EQ(envelope["payload"]["operation_id"], kOperationId);
    EXPECT_EQ(envelope["payload"]["outcome"], "success");
    EXPECT_FALSE(envelope["payload"].contains("reason"));
    // success 带 reason 非法
    EXPECT_FALSE(guide_pulse::build_action_result_event(
      "robot-001", type, kOperationId, ACTION_RESULT_SUCCESS, "busy"));
  }
}

TEST(BuildActionResultTest, FailedReasonsPerType)
{
  // navigate.result failed 枚举（P-038 error-codes 表）
  for (const char * reason :
    {RESULT_REASON_BUSY, RESULT_REASON_POINT_NOT_FOUND,
      RESULT_REASON_NAVIGATION_FAILED, RESULT_REASON_ABORTED,
      RESULT_REASON_LOCALIZATION_LOST})
  {
    const auto receipt = guide_pulse::build_action_result_event(
      "robot-001", NAVIGATE_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED, reason);
    ASSERT_TRUE(receipt.has_value()) << reason;
    EXPECT_EQ(
      json::parse(receipt->envelope_json)["payload"]["reason"], reason);
  }
  // scene.result failed 枚举：busy / scene_failed
  for (const char * reason : {RESULT_REASON_BUSY, RESULT_REASON_SCENE_FAILED}) {
    const auto receipt = guide_pulse::build_action_result_event(
      "robot-001", SCENE_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED, reason);
    ASSERT_TRUE(receipt.has_value()) << reason;
    EXPECT_EQ(
      json::parse(receipt->envelope_json)["payload"]["reason"], reason);
  }
  // relocate.result failed 枚举：busy / point_not_found / localization_failed
  for (const char * reason :
    {RESULT_REASON_BUSY, RESULT_REASON_POINT_NOT_FOUND,
      RESULT_REASON_LOCALIZATION_FAILED})
  {
    const auto receipt = guide_pulse::build_action_result_event(
      "robot-001", RELOCATE_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED, reason);
    ASSERT_TRUE(receipt.has_value()) << reason;
    EXPECT_EQ(
      json::parse(receipt->envelope_json)["payload"]["reason"], reason);
  }
  // 跨指令枚举互不接受：scene 不认 navigation_failed，navigate 不认
  // scene_failed，relocate 不认 navigation_failed / localization_lost
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", SCENE_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED,
    RESULT_REASON_NAVIGATION_FAILED));
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", NAVIGATE_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED,
    RESULT_REASON_SCENE_FAILED));
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", RELOCATE_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED,
    RESULT_REASON_NAVIGATION_FAILED));
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", RELOCATE_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED,
    RESULT_REASON_LOCALIZATION_LOST));
}

TEST(BuildActionResultTest, RejectedSharesExpiryAndInvalidRequest)
{
  for (const char * type :
    {NAVIGATE_RESULT_TYPE, RELOCATE_RESULT_TYPE, SCENE_RESULT_TYPE})
  {
    for (const char * reason : {TTS_REASON_COMMAND_EXPIRED, TTS_REASON_INVALID_REQUEST}) {
      const auto receipt = guide_pulse::build_action_result_event(
        "robot-001", type, kOperationId, ACTION_RESULT_REJECTED, reason);
      ASSERT_TRUE(receipt.has_value()) << type << " " << reason;
      EXPECT_EQ(json::parse(receipt->envelope_json)["payload"]["reason"], reason);
    }
  }
}

TEST(BuildActionResultTest, RejectsInvalidArguments)
{
  // 未知回执类型（ota.result——升级回执由其他组件负责）
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", "ota.result", kOperationId, ACTION_RESULT_SUCCESS));
  // failed/rejected 缺 reason；success/failed/rejected 之外的 outcome
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", NAVIGATE_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED));
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", SCENE_RESULT_TYPE, kOperationId, ACTION_RESULT_REJECTED));
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", NAVIGATE_RESULT_TYPE, kOperationId, "accepted"));
  // rejected 不接受 failed 枚举的 reason
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", NAVIGATE_RESULT_TYPE, kOperationId, ACTION_RESULT_REJECTED,
    RESULT_REASON_BUSY));
}

TEST(BuildActionResultTest, EachBuildGetsFreshEventId)
{
  const auto a = guide_pulse::build_action_result_event(
    "robot-001", NAVIGATE_RESULT_TYPE, kOperationId, ACTION_RESULT_SUCCESS);
  const auto b = guide_pulse::build_action_result_event(
    "robot-001", NAVIGATE_RESULT_TYPE, kOperationId, ACTION_RESULT_SUCCESS);
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_NE(a->event_id, b->event_id);
}

// ---------------------------------------------------------------------------
// point_map.result 回执构造（v0.25.0 P-039：failed 枚举仅 point_map_failed）
// ---------------------------------------------------------------------------
using guide_pulse::POINT_MAP_RESULT_TYPE;
using guide_pulse::RESULT_REASON_POINT_MAP_FAILED;

TEST(BuildPointMapResultTest, SuccessWithoutReason)
{
  const auto receipt = guide_pulse::build_action_result_event(
    "robot-001", POINT_MAP_RESULT_TYPE, kOperationId, ACTION_RESULT_SUCCESS);
  ASSERT_TRUE(receipt.has_value());
  const auto envelope = json::parse(receipt->envelope_json);
  EXPECT_EQ(envelope["direction"], "up");
  EXPECT_EQ(envelope["type"], "point_map.result");
  EXPECT_EQ(envelope["payload"]["operation_id"], kOperationId);
  EXPECT_EQ(envelope["payload"]["outcome"], "success");
  EXPECT_FALSE(envelope["payload"].contains("reason"));
  // success 带 reason 非法
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", POINT_MAP_RESULT_TYPE, kOperationId, ACTION_RESULT_SUCCESS,
    "point_map_failed"));
}

TEST(BuildPointMapResultTest, FailedOnlyAcceptsPointMapFailed)
{
  const auto receipt = guide_pulse::build_action_result_event(
    "robot-001", POINT_MAP_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED,
    RESULT_REASON_POINT_MAP_FAILED);
  ASSERT_TRUE(receipt.has_value());
  EXPECT_EQ(
    json::parse(receipt->envelope_json)["payload"]["reason"], "point_map_failed");
  // 跨指令枚举不接受：busy/navigation_failed 等属 navigate.result
  for (const char * reason :
    {"busy", "navigation_failed", "scene_failed", "point_not_found"})
  {
    EXPECT_FALSE(guide_pulse::build_action_result_event(
      "robot-001", POINT_MAP_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED,
      reason)) << reason;
  }
  // failed 缺 reason 非法
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", POINT_MAP_RESULT_TYPE, kOperationId, ACTION_RESULT_FAILED));
}

TEST(BuildPointMapResultTest, RejectedSharesExpiryAndInvalidRequest)
{
  for (const char * reason : {TTS_REASON_COMMAND_EXPIRED, TTS_REASON_INVALID_REQUEST}) {
    const auto receipt = guide_pulse::build_action_result_event(
      "robot-001", POINT_MAP_RESULT_TYPE, kOperationId, ACTION_RESULT_REJECTED,
      reason);
    ASSERT_TRUE(receipt.has_value()) << reason;
    EXPECT_EQ(json::parse(receipt->envelope_json)["payload"]["reason"], reason);
  }
  // rejected 不接受 failed 枚举
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", POINT_MAP_RESULT_TYPE, kOperationId, ACTION_RESULT_REJECTED,
    RESULT_REASON_POINT_MAP_FAILED));
  // 非法 type 拼写（如 point_map 不带 .result）
  EXPECT_FALSE(guide_pulse::build_action_result_event(
    "robot-001", "point_map", kOperationId, ACTION_RESULT_SUCCESS));
}

TEST(CommandEventTest, InvalidNavigateScenePayloadsFillReceiptId)
{
  // v0.24.0：navigate/scene 载荷结构性非法（operation_id 已解析）须填
  // invalid_operation_id + invalid_command_type 供调用方回 rejected/
  // invalid_request；未知 scene 枚举值是未来指令，不填
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"navigate", R"({"operation_id":")" + std::string(kOperationId) + R"("})"},
    {"navigate",
      R"({"operation_id":")" + std::string(kOperationId) +
      R"(","point_name":"","texts":["x"]})"},
    {"scene",
      R"({"operation_id":")" + std::string(kOperationId) + R"(","extra":1})"},
    {"scene",
      R"({"operation_id":")" + std::string(kOperationId) +
      R"(","scene":"welcome","qr_sha256":"zz"})"},
  };
  for (const auto & [type, body] : cases) {
    std::string error;
    std::string invalid_op;
    std::string invalid_type;
    EXPECT_FALSE(guide_pulse::parse_command_event(
      build_command_json(type, body), "robot-001", error, &invalid_op,
      &invalid_type)) << type << " " << body;
    EXPECT_EQ(invalid_op, kOperationId) << type;
    EXPECT_EQ(invalid_type, type) << type;
  }

  // 未知 scene 枚举值（farewell）：未来版本指令，不填回执通道
  std::string error;
  std::string invalid_op;
  EXPECT_FALSE(guide_pulse::parse_command_event(
    build_command_json(
      "scene", R"({"operation_id":")" + std::string(kOperationId) +
      R"(","scene":"farewell"})"),
    "robot-001", error, &invalid_op));
  EXPECT_TRUE(invalid_op.empty());
}

// ---------------------------------------------------------------------------
// 语音触发上报构造（v0.26.0 P-040：navigate.triggered / relocate.triggered
// 仅上行自主事件，payload {point_name, trigger} 无 operation_id）
// ---------------------------------------------------------------------------
using guide_pulse::NAVIGATE_TRIGGERED_TYPE;
using guide_pulse::RELOCATE_TRIGGERED_TYPE;
using guide_pulse::TRIGGER_VOICE;

TEST(BuildNavTriggeredTest, EnvelopeShapePerContract)
{
  // 两个类型同构：direction=up、payload 恰好 point_name + trigger，
  // 不得携带 operation_id（契约 additionalProperties=false）
  for (const char * type : {NAVIGATE_TRIGGERED_TYPE, RELOCATE_TRIGGERED_TYPE}) {
    const auto receipt = guide_pulse::build_nav_triggered_event(
      "robot-001", type, "比亚迪ATTO 1", TRIGGER_VOICE);
    ASSERT_TRUE(receipt.has_value()) << type;
    const auto envelope = json::parse(receipt->envelope_json);
    EXPECT_EQ(envelope["spec_version"], "1.0");
    EXPECT_EQ(envelope["robot_id"], "robot-001");
    EXPECT_EQ(envelope["direction"], "up");
    EXPECT_EQ(envelope["type"], type);
    EXPECT_EQ(envelope["payload"]["point_name"], "比亚迪ATTO 1");
    EXPECT_EQ(envelope["payload"]["trigger"], "voice");
    EXPECT_FALSE(envelope["payload"].contains("operation_id"));
    EXPECT_EQ(envelope["payload"].size(), 2U);
  }
}

TEST(BuildNavTriggeredTest, RejectsInvalidArguments)
{
  // 未知上报类型（tts.result / navigate.result 均非本构造面的类型）
  EXPECT_FALSE(guide_pulse::build_nav_triggered_event(
    "robot-001", "tts.result", "A", TRIGGER_VOICE));
  EXPECT_FALSE(guide_pulse::build_nav_triggered_event(
    "robot-001", guide_pulse::NAVIGATE_RESULT_TYPE, "A", TRIGGER_VOICE));
  // 触发类型可扩展枚举：当前唯一值 voice，其余一律拒绝
  EXPECT_FALSE(guide_pulse::build_nav_triggered_event(
    "robot-001", NAVIGATE_TRIGGERED_TYPE, "A", "app"));
  EXPECT_FALSE(guide_pulse::build_nav_triggered_event(
    "robot-001", RELOCATE_TRIGGERED_TYPE, "A", ""));
  // point_name 1..64 Unicode 标量：空串 / 65 标量均非法（按码点非字节）
  EXPECT_FALSE(guide_pulse::build_nav_triggered_event(
    "robot-001", NAVIGATE_TRIGGERED_TYPE, "", TRIGGER_VOICE));
  EXPECT_FALSE(guide_pulse::build_nav_triggered_event(
    "robot-001", RELOCATE_TRIGGERED_TYPE, std::string(65, 'A'), TRIGGER_VOICE));
  // 64 个中文 = 64 标量（UTF-8 字节数 192 远大于 64）：合法，验证按码点计
  std::string cjk64;
  for (int i = 0; i < 64; ++i) {
    cjk64 += "点";
  }
  EXPECT_TRUE(guide_pulse::build_nav_triggered_event(
    "robot-001", NAVIGATE_TRIGGERED_TYPE, cjk64, TRIGGER_VOICE));
}

TEST(BuildNavTriggeredTest, EachBuildGetsFreshEventId)
{
  // 一次触发恰好一条：每次构造独立 event_id（QoS 1 重发须由调用方复用
  // 同一信封原文，与回执面同策略）
  const auto a = guide_pulse::build_nav_triggered_event(
    "robot-001", NAVIGATE_TRIGGERED_TYPE, "A", TRIGGER_VOICE);
  const auto b = guide_pulse::build_nav_triggered_event(
    "robot-001", NAVIGATE_TRIGGERED_TYPE, "A", TRIGGER_VOICE);
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_NE(a->event_id, b->event_id);
}

// ---------------------------------------------------------------------------
// v0.30.0（P-044）navigate.completed / relocate.completed 终态上报构造
// ---------------------------------------------------------------------------
using guide_pulse::ACTION_RESULT_FAILED;
using guide_pulse::ACTION_RESULT_REJECTED;
using guide_pulse::ACTION_RESULT_SUCCESS;
using guide_pulse::NAVIGATE_COMPLETED_TYPE;
using guide_pulse::RELOCATE_COMPLETED_TYPE;
using guide_pulse::RESULT_REASON_ABORTED;
using guide_pulse::RESULT_REASON_LOCALIZATION_FAILED;
using guide_pulse::RESULT_REASON_NAVIGATION_FAILED;
using guide_pulse::TRIGGER_VOICE;

TEST(BuildNavCompletedTest, BuildsSuccessEnvelope)
{
  // 两类型同构：direction=up、success 时 payload 恰好 point_name +
  // trigger + outcome（禁 reason），不得携带 operation_id（契约
  // additionalProperties=false）
  for (const char * type : {NAVIGATE_COMPLETED_TYPE, RELOCATE_COMPLETED_TYPE}) {
    const auto receipt = guide_pulse::build_nav_completed_event(
      "robot-001", type, "比亚迪ATTO 1", TRIGGER_VOICE, ACTION_RESULT_SUCCESS);
    ASSERT_TRUE(receipt.has_value()) << type;
    const auto envelope = json::parse(receipt->envelope_json);
    EXPECT_EQ(envelope["spec_version"], "1.0");
    EXPECT_EQ(envelope["robot_id"], "robot-001");
    EXPECT_EQ(envelope["direction"], "up");
    EXPECT_EQ(envelope["type"], type);
    EXPECT_EQ(envelope["payload"]["point_name"], "比亚迪ATTO 1");
    EXPECT_EQ(envelope["payload"]["trigger"], "voice");
    EXPECT_EQ(envelope["payload"]["outcome"], "success");
    EXPECT_FALSE(envelope["payload"].contains("reason"));
    EXPECT_FALSE(envelope["payload"].contains("operation_id"));
    EXPECT_EQ(envelope["payload"].size(), 3U);
  }
}

TEST(BuildNavCompletedTest, BuildsFailedEnvelopeWithPerTypeReason)
{
  // failed 必填 reason：navigate.completed 可用 aborted/navigation_failed，
  // relocate.completed 可用 localization_failed；reason 枚举按指令分型
  //（同对应 *.result 的 failed 枚举）
  const auto nav = guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_FAILED, RESULT_REASON_ABORTED);
  ASSERT_TRUE(nav.has_value());
  auto nav_envelope = json::parse(nav->envelope_json);
  EXPECT_EQ(nav_envelope["payload"]["outcome"], "failed");
  EXPECT_EQ(nav_envelope["payload"]["reason"], "aborted");
  EXPECT_EQ(nav_envelope["payload"].size(), 4U);

  const auto reloc = guide_pulse::build_nav_completed_event(
    "robot-001", RELOCATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_FAILED, RESULT_REASON_LOCALIZATION_FAILED);
  ASSERT_TRUE(reloc.has_value());
  auto reloc_envelope = json::parse(reloc->envelope_json);
  EXPECT_EQ(reloc_envelope["payload"]["outcome"], "failed");
  EXPECT_EQ(reloc_envelope["payload"]["reason"], "localization_failed");
}

TEST(BuildNavCompletedTest, RejectsInvalidArguments)
{
  // 未知上报类型（tts.result / navigate.result / triggered 均非本构造面）
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", "tts.result", "A", TRIGGER_VOICE, ACTION_RESULT_SUCCESS));
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", guide_pulse::NAVIGATE_RESULT_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_SUCCESS));
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", guide_pulse::NAVIGATE_TRIGGERED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_SUCCESS));
  // 触发类型可扩展枚举：当前唯一值 voice
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", "app", ACTION_RESULT_SUCCESS));
  // point_name 1..64 Unicode 标量：空串 / 65 标量均非法（按码点非字节）
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "", TRIGGER_VOICE,
    ACTION_RESULT_SUCCESS));
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", RELOCATE_COMPLETED_TYPE, std::string(65, 'A'), TRIGGER_VOICE,
    ACTION_RESULT_SUCCESS));
  // 没有 rejected：自主动作无指令可拒绝（契约明确排除该分支）
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_REJECTED, TTS_REASON_COMMAND_EXPIRED));
  // success 禁 reason / failed 必填 reason
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_SUCCESS, RESULT_REASON_NAVIGATION_FAILED));
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_FAILED));
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_FAILED, ""));
  // failed reason 按指令分型：navigation_failed/aborted 不属于
  // relocate.completed 的枚举（localization_failed 归 relocate，
  // navigate 侧用 localization_lost）
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", RELOCATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_FAILED, RESULT_REASON_NAVIGATION_FAILED));
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", RELOCATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_FAILED, RESULT_REASON_ABORTED));
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_FAILED, RESULT_REASON_LOCALIZATION_FAILED));
  // 未知 reason 一律拒绝
  EXPECT_FALSE(guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_FAILED, "transform_failed"));
}

TEST(BuildNavCompletedTest, EachBuildGetsFreshEventId)
{
  // 一次动作恰好一条：每次构造独立 event_id（QoS 1 重发须由调用方复用
  // 同一信封原文，与 triggered/回执面同策略）
  const auto a = guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_SUCCESS);
  const auto b = guide_pulse::build_nav_completed_event(
    "robot-001", NAVIGATE_COMPLETED_TYPE, "A", TRIGGER_VOICE,
    ACTION_RESULT_SUCCESS);
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_NE(a->event_id, b->event_id);
}

}  // namespace
