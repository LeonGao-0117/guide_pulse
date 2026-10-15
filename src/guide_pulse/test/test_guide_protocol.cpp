// ============================================================================
// guide_pulse 协议层单元测试（无 ROS 依赖）
// 覆盖：down/commands 四类指令解析/校验（tts v0.5.0 / tts.stop / scene
// v0.17.0+v0.20.0 / navigate v0.16.0+v0.20.0）、tts.result 回执构造（v0.6.0）、
// 播报列表轮序、优先级档位校验、工具函数。
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
  // relocate / ota.start 等由其他组件处理：error=="ignore" 静默忽略
  for (const auto * type : {"relocate", "ota.start", "unknown.thing"}) {
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
  // scene 无回执面：即使载荷非法也不填 invalid_operation_id
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

}  // namespace
