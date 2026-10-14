// ============================================================================
// SDK TTS 客户端封装：仅调用 SDK v0.9.4.4 (aimdk_msgs，已内置
// src/aimdk_msgs) 提供的 /aimdk_5Fmsgs/srv/PlayTts 服务完成语音合成播报
// 与"停止"。
//
// 说明：SDK v0.9.4.4 未提供独立的 StopTts 服务。依据《接口说明.md》的
// 优先级仲裁矩阵，使用“最高优先级 (SAFETY_L10=10) + 最大加权 (99) +
// is_interrupted=true + 近静音文本”的 PlayTts 请求即可立即掐断任意进行中
// 的播报（新任务权重 1099 必然 >= 当前任务），从而实现停止效果。
// ============================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <aimdk_msgs/srv/play_tts.hpp>

#include "guide_pulse/guide_protocol.hpp"

namespace guide_pulse
{

class TtsClient
{
public:
  // PlayTts 服务调用结果
  struct Result
  {
    bool ok{false};                 // 服务调用成功且回执判定通过
    bool service_failed{false};     // 服务不可达/调用超时（区别于 SDK 业务/仲裁拒绝）：
                                    // true 时上层按 tts.result(playback_unavailable) 回执
    int64_t code{0};                // SDK CommonResponse.header.code（0 成功）
    std::string message;            // SDK CommonResponse.message
    bool is_success{false};         // SDK PlayTtsResponse.is_success
    std::string error_message;      // SDK PlayTtsResponse.error_message
    std::string detail;             // 本地失败描述（超时/服务不可达等），ok=false 时非空
    std::int64_t elapsed_ms{0};     // 请求发出→回执耗时（成功日志用）
    std::uint32_t estimated_duration_ms{0};  // SDK 回执的预计播报耗时（当前 SDK 固定返回 0）
  };

  // node: 所属节点（通常是桥接节点本身）；服务客户端挂独立回调组，
  // 保证节点被 executor 正常 spin 时，工作线程可以阻塞等待回执。
  TtsClient(
    rclcpp::Node::SharedPtr node, std::string service_name, std::string domain,
    int request_timeout_seconds, std::string stop_text);

  // 阻塞等待服务上线（timeout 秒）。返回是否可用。
  bool wait_for_service(std::chrono::seconds timeout);

  // 播报：将下行指令映射为 SDK PlayTts 请求并调用。
  Result play(const TtsCommand & command);

  // 停止：最高优先级抢占播报（见文件头说明）。
  Result stop(const std::string & trace_id = {});

private:
  Result call_play_tts(
    const std::string & text, int priority_level, int priority_weight,
    bool is_interrupted, const std::string & trace_id_suffix);

  rclcpp::Node::SharedPtr node_;
  std::string service_name_;
  std::string domain_;
  std::chrono::seconds request_timeout_;
  std::string stop_text_;
  rclcpp::Client<aimdk_msgs::srv::PlayTts>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace guide_pulse
