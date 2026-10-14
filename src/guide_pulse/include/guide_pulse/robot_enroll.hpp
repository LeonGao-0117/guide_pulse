// ============================================================================
// Robot enroll 注册面客户端（RobotCar 协议 v0.9.0 / v0.10.0，D-019/P-025）。
//
// Robot 凭预登记的 serial_number 调 Server HTTP 注册面
//   POST {enroll_origin}/v1/robots/enroll   {"serial_number": "..."}
// 换取当次运行的 MQTT 凭据：
//   200 -> {robot_id, username, password, expires_at}
//   400 invalid_request / 403 enroll_rejected|robot_revoked / 429 rate_limited
//
// 语义要点（见 Protocol/contracts/server-robot/http/v1/robot-enroll-protocol.md）：
//   - 注册表以 SN 为唯一锚点；robot_id 由 Server 首次 enroll 时分配
//     （robot-001 起可读递增），同 SN 恒返回同一 robot_id，只轮换密码；
//   - username 即 robot_id；凭据只在内存中当次运行使用，不落盘、不打印；
//   - 密码有 TTL（expires_at），到期由 Server 作废，Robot 侧表现为
//     MQTT CONNECT 被拒（0x86）后重新 enroll。
//
// 本层只依赖 nlohmann/json 与 libcurl，不依赖 ROS，便于独立单元测试。
// ============================================================================
#pragma once

#include <optional>
#include <string>

namespace guide_pulse
{

// 当次 enroll 换得的凭据（仅内存持有）
struct RobotCredential
{
  std::string robot_id;     // Server 分配（或复用）的机器人 ID，同时是主题与身份锚点
  std::string username;     // MQTT username（协议规定等于 robot_id，但以响应为准）
  std::string password;     // 当次运行的 MQTT 密码（不落盘、不进日志）
  std::string expires_at;   // 本次密码作废时刻（RFC3339，仅用于日志展示）
};

// enroll 失败分类（决定桥接节点的重试策略）
enum class EnrollError
{
  NONE = 0,
  NETWORK,         // 请求未达 / 超时 / 响应无法读取 -> 退避后重试
  INVALID_REQUEST, // 400：serial_number 格式非法（本地配置问题）
  REJECTED,        // 403 enroll_rejected：SN 未预登记（运维登记后可恢复，退避重试）
  REVOKED,         // 403 robot_revoked：被吊销 -> 停止重试，等待运维处理
  RATE_LIMITED,    // 429：失败次数超限，退避后重试
  BAD_RESPONSE,    // 200 但字段非法，或错误响应体无法识别 -> 退避重试
};

struct EnrollResult
{
  EnrollError error{EnrollError::NONE};
  std::string detail;             // 诊断信息（绝不包含密码）
  RobotCredential credential;     // error==NONE 时有效
};

// serial_number 格式校验（OpenAPI EnrollRequest：长度 6..64，
// 首字符字母数字，其后 [A-Za-z0-9._:-]）。本地先拦，避免发出必 400 的请求。
bool valid_serial_number(const std::string & serial_number);

// 从文件读取序列号（批量部署自动获取）：取第一行并去除首尾空白
// （含 \r\n）。文件不存在/不可读/读后为空返回 std::nullopt；
// 格式校验由调用方用 valid_serial_number 完成。
// 板端标准位置 /robot/info/sn（Jetson 上为指向 /robot/protected/sn 的
// 符号链接，是否落地取决于固件版本；运控板上恒存在）。
std::optional<std::string> read_serial_number_file(const std::string & path);

// 解析 enroll 200 响应体（纯函数，便于单测）。
// 逐字段校验非空字符串；robot_id 额外拒绝 MQTT 主题通配符（+ # / 与空白），
// 因为它会被拼进订阅/发布主题，防止错订或越权。
EnrollResult parse_enroll_response(const std::string & body);

// 解析 enroll 错误响应体 {"code","message","request_id"}（纯函数，便于单测）。
// 无法识别时归入 BAD_RESPONSE。
EnrollResult parse_enroll_error(long http_code, const std::string & body);

// enroll HTTP 客户端。线程不安全（由桥接节点的连接线程独占调用）。
class RobotEnroller
{
public:
  RobotEnroller(std::string origin, int timeout_seconds);

  // 阻塞发起一次 enroll，至完成或超时。绝不抛异常，结果见 EnrollResult。
  EnrollResult enroll(const std::string & serial_number);

private:
  std::string origin_;        // 形如 http://host:port（构造时去掉尾部 '/'）
  int timeout_seconds_;
};

}  // namespace guide_pulse
