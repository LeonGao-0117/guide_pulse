// ============================================================================
// guide_bridge 入口：创建桥接节点，连接 MQTT，spin 等待退出。
// ============================================================================
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <string>

#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "guide_pulse/mqtt_guide_bridge_node.hpp"

namespace
{

// 日志前缀时间改为 RFC3339 UTC 毫秒（如 2026-09-30T05:15:10.776Z）。
// Humble 的 RCUTILS_CONSOLE_OUTPUT_FORMAT 没有日历时间 token（{time} 仅
// epoch 秒），故安装进程级自定义 output handler 直接格式化。行为与默认
// console handler 对齐：ERROR 及以上走 stderr，单次 fprintf 整行输出
// 避免多线程日志交错。
void rfc3339_log_output_handler(
  const rcutils_log_location_t *, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  // 先测长度再格式化，避免长 payload 日志（播报列表文本可达 ~4KB）被截断
  std::string message(1, '\0');
  {
    va_list ap;
    va_copy(ap, *args);
    const int needed = std::vsnprintf(nullptr, 0, format, ap);
    va_end(ap);
    if (needed > 0) {
      message.resize(static_cast<std::size_t>(needed) + 1, '\0');
      va_copy(ap, *args);
      std::vsnprintf(message.data(), message.size(), format, ap);
      va_end(ap);
    }
  }

  // epoch 纳秒 -> UTC 年月日时分秒.毫秒
  static constexpr int64_t kNsPerSec = 1000000000LL;
  const std::time_t secs = static_cast<std::time_t>(timestamp / kNsPerSec);
  const int millis = static_cast<int>((timestamp % kNsPerSec) / 1000000LL);
  std::tm tm_buf{};
  gmtime_r(&secs, &tm_buf);
  char time_buf[64];  // gcc 无法推断 tm 字段宽度，给足空间避免 -Wformat-truncation
  std::snprintf(
    time_buf, sizeof(time_buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
    tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
    tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, millis);

  // severity 枚举步长 10（DEBUG=10..FATAL=50），映射为默认前缀同名大写
  static constexpr const char * kNames[] = {"UNKNOWN", "DEBUG", "INFO", "WARN", "ERROR",
    "FATAL"};
  const int idx = severity / 10;
  const char * severity_name =
    (idx >= 1 && idx <= 5) ? kNames[idx] : kNames[0];

  std::fprintf(
    severity >= RCUTILS_LOG_SEVERITY_ERROR ? stderr : stdout,
    "[%s] [%s] [%s]: %s\n", severity_name, time_buf,
    (name != nullptr) ? name : "", message.c_str());
}

}  // namespace

int main(int argc, char ** argv)
{
  // stdout 行缓冲：重定向到文件时默认 4KB 块缓冲，逐行日志会积压滞后
  //（集成测试轮询桥接日志与运维 tail 排障都依赖实时输出）
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  rclcpp::init(argc, argv);
  rcutils_logging_set_output_handler(rfc3339_log_output_handler);

  auto node = std::make_shared<guide_pulse::MqttGuideBridgeNode>(
    rclcpp::NodeOptions());

  // shared_from_this 依赖：先加入 executor 再 start()
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  if (!node->start()) {
    RCLCPP_ERROR(node->get_logger(),
      "guide_bridge 启动失败（enroll 配置缺失或 serial_number 非法），退出");
    rclcpp::shutdown();
    return 1;
  }

  executor.spin();

  node->shutdown();
  rclcpp::shutdown();
  return 0;
}
