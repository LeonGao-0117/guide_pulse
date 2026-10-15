// ============================================================================
// ROS 服务同步调用封装（250ms 粒度检查节点关停，shutdown 时不被慢应答拖住）。
// 桥接节点（播报伴随动作）与迎宾变形模块共用。
// ============================================================================
#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace guide_pulse
{

// 变形/动作线程内的同步服务调用封装：future 由 executor 线程投递（调用
// 线程独立于 worker/executor，可安全阻塞等待）。timeout 秒无应答返回空指针
template<typename T>
typename T::Response::SharedPtr call_with_timeout(
  rclcpp::Client<T> & client, std::shared_ptr<typename T::Request> request,
  int timeout_seconds)
{
  auto future = client.async_send_request(request);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::seconds(timeout_seconds);
  // 250ms 粒度检查节点关停：shutdown 时不被一次慢应答拖住退出
  while (future.wait_for(std::chrono::milliseconds(250)) !=
    std::future_status::ready)
  {
    if (!rclcpp::ok() || std::chrono::steady_clock::now() >= deadline) {
      client.remove_pending_request(future);
      return nullptr;
    }
  }
  return future.get();
}

}  // namespace guide_pulse
