// ============================================================================
// robot_enroll 单元测试：响应解析、错误码映射、serial_number 校验。
// HTTP 传输本身由集成测试经 tools/enroll_mock_server.py 覆盖。
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

#include "guide_pulse/robot_enroll.hpp"

namespace
{

using guide_pulse::EnrollError;
using guide_pulse::parse_enroll_error;
using guide_pulse::parse_enroll_response;
using guide_pulse::read_serial_number_file;
using guide_pulse::valid_serial_number;

// 与 OpenAPI 一致的合法 200 响应样例
const char * kValidBody = R"({
  "robot_id": "robot-001",
  "username": "robot-001",
  "password": "0123456789abcdef0123456789abcdef",
  "expires_at": "2026-10-08T12:00:00Z"
})";

TEST(RobotEnrollTest, ParseValidResponse)
{
  const auto result = parse_enroll_response(kValidBody);
  ASSERT_EQ(result.error, EnrollError::NONE);
  EXPECT_EQ(result.credential.robot_id, "robot-001");
  EXPECT_EQ(result.credential.username, "robot-001");
  EXPECT_EQ(result.credential.password, "0123456789abcdef0123456789abcdef");
  EXPECT_EQ(result.credential.expires_at, "2026-10-08T12:00:00Z");
}

TEST(RobotEnrollTest, ParseRejectsNonJson)
{
  const auto result = parse_enroll_response("not json");
  EXPECT_EQ(result.error, EnrollError::BAD_RESPONSE);
}

TEST(RobotEnrollTest, ParseRejectsMissingField)
{
  // 缺 password
  const std::string body = R"({"robot_id":"robot-001","username":"robot-001",)"
    R"("expires_at":"2026-10-08T12:00:00Z"})";
  const auto result = parse_enroll_response(body);
  EXPECT_EQ(result.error, EnrollError::BAD_RESPONSE);
  EXPECT_NE(result.detail.find("password"), std::string::npos);
}

TEST(RobotEnrollTest, ParseRejectsWrongType)
{
  const std::string body = R"({"robot_id":123,"username":"u",)"
    R"("password":"p","expires_at":"e"})";
  EXPECT_EQ(parse_enroll_response(body).error, EnrollError::BAD_RESPONSE);
}

TEST(RobotEnrollTest, ParseRejectsTopicUnsafeRobotId)
{
  // robot_id 会被拼进订阅主题：通配符/分隔符/空白必须拒绝
  for (const char * bad : {"robot/001", "robot/+", "robot/#", "robot 001",
    "robot#001", "", "a+b"}) {
    const std::string body = std::string(R"({"robot_id":")") + bad +
      R"(","username":"u","password":"p","expires_at":"e"})";
    EXPECT_EQ(parse_enroll_response(body).error, EnrollError::BAD_RESPONSE)
      << "robot_id='" << bad << "'";
  }
}

TEST(RobotEnrollTest, ParseAcceptsRobotIdNaturalForms)
{
  // 协议规定的可读递增形态（robot-001、超 999 进位 robot-1000）应放行
  for (const char * good : {"robot-001", "robot-999", "robot-1000", "robot-12345"}) {
    const std::string body = std::string(R"({"robot_id":")") + good +
      R"(","username":")" + good +
      R"(","password":"p","expires_at":"e"})";
    const auto result = parse_enroll_response(body);
    ASSERT_EQ(result.error, EnrollError::NONE) << "robot_id='" << good << "'";
    EXPECT_EQ(result.credential.robot_id, good);
  }
}

TEST(RobotEnrollTest, ParseRejectsEmptyCredential)
{
  const std::string body = R"({"robot_id":"robot-001","username":"",)"
    R"("password":"p","expires_at":"e"})";
  EXPECT_EQ(parse_enroll_response(body).error, EnrollError::BAD_RESPONSE);
}

TEST(RobotEnrollTest, ErrorCodeMapping)
{
  const std::string body = R"({"code":"robot_revoked","message":"revoked by ops",)"
    R"("request_id":"01234567-89ab-7cde-8f01-23456789abcd"})";
  EXPECT_EQ(parse_enroll_error(403, body).error, EnrollError::REVOKED);
  EXPECT_EQ(
    parse_enroll_error(403, R"({"code":"enroll_rejected"})").error,
    EnrollError::REJECTED);
  EXPECT_EQ(
    parse_enroll_error(400, R"({"code":"invalid_request"})").error,
    EnrollError::INVALID_REQUEST);
  EXPECT_EQ(
    parse_enroll_error(429, R"({"code":"rate_limited"})").error,
    EnrollError::RATE_LIMITED);
  // 无法识别的错误体归入 BAD_RESPONSE（不误判为可重试网络错误以外的类别）
  EXPECT_EQ(parse_enroll_error(500, "oops").error, EnrollError::BAD_RESPONSE);
  EXPECT_EQ(parse_enroll_error(502, R"({"code":"other"})").error, EnrollError::BAD_RESPONSE);
}

TEST(RobotEnrollTest, SerialNumberValidation)
{
  EXPECT_TRUE(valid_serial_number("SIM-tts-demo-01"));
  EXPECT_TRUE(valid_serial_number("SN123456"));
  EXPECT_TRUE(valid_serial_number("A.b_c:d-e"));  // 全部合法特殊字符
  // 长度：5 位太短、64 位封顶
  EXPECT_FALSE(valid_serial_number("abcde"));
  EXPECT_TRUE(valid_serial_number(std::string(64, 'a')));
  EXPECT_FALSE(valid_serial_number(std::string(65, 'a')));
  // 首字符必须字母数字；非法字符与空串
  EXPECT_FALSE(valid_serial_number("-abcde"));
  EXPECT_FALSE(valid_serial_number(".abcde"));
  EXPECT_FALSE(valid_serial_number("abc de"));
  EXPECT_FALSE(valid_serial_number("abc/de"));
  EXPECT_FALSE(valid_serial_number("abc+de"));
  EXPECT_FALSE(valid_serial_number(""));
}

TEST(RobotEnrollTest, SerialNumberFileReading)
{
  // 临时文件：内容带换行/回车/首尾空白，读取应只得到 trim 后的首行
  const std::string path = "/tmp/guide_pulse_sn_test_XXXXXX";
  std::vector<char> tmpl(path.begin(), path.end());
  tmpl.push_back('\0');
  const int fd = ::mkstemp(tmpl.data());
  ASSERT_GE(fd, 0);
  const std::string real_path(tmpl.data());
  ::close(fd);
  {
    std::ofstream out(real_path, std::ios::binary);
    out << "  T1100P0B900612\r\nOTHER-LINE-IGNORED\n";
  }

  const auto sn = read_serial_number_file(real_path);
  ASSERT_TRUE(sn.has_value());
  EXPECT_EQ(*sn, "T1100P0B900612");
  EXPECT_TRUE(valid_serial_number(*sn));  // 真机 SN 形态合法性闭环
  ::remove(real_path.c_str());
}

TEST(RobotEnrollTest, SerialNumberFileMissingOrEmpty)
{
  // 文件不存在 / 路径为空 / 文件为空行：都返回 nullopt（交由上层回退）
  EXPECT_FALSE(read_serial_number_file("/nonexistent/sn").has_value());
  EXPECT_FALSE(read_serial_number_file("").has_value());

  const std::string path = "/tmp/guide_pulse_sn_empty_XXXXXX";
  std::vector<char> tmpl(path.begin(), path.end());
  tmpl.push_back('\0');
  const int fd = ::mkstemp(tmpl.data());
  ASSERT_GE(fd, 0);
  const std::string real_path(tmpl.data());
  ::close(fd);
  {
    std::ofstream out(real_path, std::ios::binary);
    out << "   \r\n";
  }
  EXPECT_FALSE(read_serial_number_file(real_path).has_value());
  ::remove(real_path.c_str());
}

}  // namespace
