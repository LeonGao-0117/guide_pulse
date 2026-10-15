// ============================================================================
// Robot enroll 客户端实现：libcurl 同步 POST + 响应解析。
// ============================================================================
#include "guide_pulse/robot_enroll.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace guide_pulse
{

using nlohmann::json;

namespace
{

// 响应体读取回调：追加到调用方字符串；超限视为异常响应主动中止
std::size_t append_body(char * ptr, std::size_t size, std::size_t nmemb, void * userdata)
{
  auto * out = static_cast<std::string *>(userdata);
  const std::size_t bytes = size * nmemb;
  if (out->size() + bytes > 65536) {
    return 0;  // 非预期长度（协议响应仅几百字节），中止传输
  }
  out->append(ptr, bytes);
  return bytes;
}

std::string snippet(const std::string & text, std::size_t max_len = 160)
{
  if (text.size() <= max_len) {
    return text;
  }
  return text.substr(0, max_len) + "...";
}

// robot_id 会被拼进 MQTT 主题，拒绝通配符/分隔符/空白等危险字符
bool topic_safe_id(const std::string & id)
{
  if (id.empty() || id.size() > 128) {
    return false;
  }
  return std::all_of(id.begin(), id.end(), [](unsigned char ch) {
    return std::isalnum(ch) || ch == '-' || ch == '_' || ch == '.';
  });
}

}  // namespace

bool valid_serial_number(const std::string & serial_number)
{
  const std::size_t len = serial_number.size();
  if (len < 6 || len > 64) {
    return false;
  }
  const auto allowed = [](unsigned char ch) {
      return std::isalnum(ch) || ch == '.' || ch == '_' || ch == ':' || ch == '-';
    };
  if (!std::isalnum(static_cast<unsigned char>(serial_number[0]))) {
    return false;
  }
  return std::all_of(serial_number.begin() + 1, serial_number.end(), allowed);
}

std::optional<std::string> read_serial_number_file(const std::string & path)
{
  if (path.empty()) {
    return std::nullopt;
  }
  std::ifstream file(path);
  if (!file.is_open()) {
    return std::nullopt;  // 不存在/不可读（如 Jetson 上固件未落 SN 文件）
  }
  std::string line;
  std::getline(file, line);
  // 去首尾空白（含 \r：文件可能来自其他平台的换行风格）
  const auto not_space = [](unsigned char ch) {return !std::isspace(ch);};
  line.erase(line.begin(), std::find_if(line.begin(), line.end(), not_space));
  line.erase(std::find_if(line.rbegin(), line.rend(), not_space).base(), line.end());
  if (line.empty()) {
    return std::nullopt;
  }
  return line;
}

EnrollResult parse_enroll_response(const std::string & body)
{
  EnrollResult result;
  json value;
  try {
    value = json::parse(body);
  } catch (const json::exception & e) {
    result.error = EnrollError::BAD_RESPONSE;
    result.detail = std::string("200 响应不是合法 JSON: ") + e.what();
    return result;
  }
  if (!value.is_object()) {
    result.error = EnrollError::BAD_RESPONSE;
    result.detail = "200 响应不是 JSON 对象";
    return result;
  }

  // 逐字段取出并校验（协议四字段均必填字符串）
  const char * fields[] = {"robot_id", "username", "password", "expires_at"};
  for (const char * field : fields) {
    if (!value.contains(field) || !value.at(field).is_string()) {
      result.error = EnrollError::BAD_RESPONSE;
      result.detail = std::string("200 响应缺少字符串字段: ") + field;
      return result;
    }
  }
  result.credential.robot_id = value.at("robot_id").get<std::string>();
  result.credential.username = value.at("username").get<std::string>();
  result.credential.password = value.at("password").get<std::string>();
  result.credential.expires_at = value.at("expires_at").get<std::string>();

  if (!topic_safe_id(result.credential.robot_id)) {
    result.error = EnrollError::BAD_RESPONSE;
    result.detail = "robot_id 含主题非法字符或超长: '" + snippet(result.credential.robot_id) + "'";
    result.credential = RobotCredential{};
    return result;
  }
  if (result.credential.username.empty() || result.credential.username.size() > 128) {
    result.error = EnrollError::BAD_RESPONSE;
    result.detail = "username 为空或超长";
    result.credential = RobotCredential{};
    return result;
  }
  if (result.credential.password.empty() || result.credential.password.size() > 256) {
    // 协议约定 32..128，客户端只做存在性/上限校验，不硬卡下限
    result.error = EnrollError::BAD_RESPONSE;
    result.detail = "password 为空或超长";
    result.credential = RobotCredential{};
    return result;
  }
  return result;
}

EnrollResult parse_enroll_error(long http_code, const std::string & body)
{
  EnrollResult result;
  std::string code;
  std::string message;
  try {
    const auto value = json::parse(body);
    if (value.is_object()) {
      if (value.contains("code") && value.at("code").is_string()) {
        code = value.at("code").get<std::string>();
      }
      if (value.contains("message") && value.at("message").is_string()) {
        message = value.at("message").get<std::string>();
      }
    }
  } catch (const json::exception &) {
  }

  if (code == "invalid_request") {
    result.error = EnrollError::INVALID_REQUEST;
  } else if (code == "enroll_rejected") {
    result.error = EnrollError::REJECTED;
  } else if (code == "robot_revoked") {
    result.error = EnrollError::REVOKED;
  } else if (code == "rate_limited") {
    result.error = EnrollError::RATE_LIMITED;
  } else {
    result.error = EnrollError::BAD_RESPONSE;
  }
  result.detail = "HTTP " + std::to_string(http_code) + (code.empty() ? "" : " code=" + code) +
    (message.empty() ? "" : " message=" + snippet(message));
  return result;
}

RobotEnroller::RobotEnroller(std::string origin, int timeout_seconds)
: origin_(std::move(origin)), timeout_seconds_(timeout_seconds)
{
  while (!origin_.empty() && origin_.back() == '/') {
    origin_.pop_back();
  }
  if (timeout_seconds_ <= 0) {
    timeout_seconds_ = 5;
  }
}

EnrollResult RobotEnroller::enroll(const std::string & serial_number)
{
  EnrollResult result;
  if (!valid_serial_number(serial_number)) {
    result.error = EnrollError::INVALID_REQUEST;
    result.detail = "serial_number 本地格式校验失败（需 6..64 位，首字符字母数字，"
      "其后仅限 [A-Za-z0-9._:-]）";
    return result;
  }

  const std::string url = origin_ + "/v1/robots/enroll";
  const std::string request_body = json{{"serial_number", serial_number}}.dump();
  std::string response_body;

  CURL * curl = curl_easy_init();
  if (curl == nullptr) {
    result.error = EnrollError::NETWORK;
    result.detail = "curl_easy_init 失败";
    return result;
  }

  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers = curl_slist_append(headers, "Accept: application/json");

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, request_body.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(timeout_seconds_));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, static_cast<long>(timeout_seconds_));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_body);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);

  const CURLcode rc = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (rc != CURLE_OK) {
    result.error = EnrollError::NETWORK;
    result.detail = std::string("enroll 请求失败: ") + curl_easy_strerror(rc);
    return result;
  }
  if (http_code == 200) {
    return parse_enroll_response(response_body);
  }
  return parse_enroll_error(http_code, response_body);
}

}  // namespace guide_pulse
