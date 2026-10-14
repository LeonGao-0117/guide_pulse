// ============================================================================
// SDK TTS 客户端实现：调用 aimdk_msgs/srv/PlayTts（服务由机器人本体提供）。
// 请求/回执字段映射依据 SDK v0.9.4.4《接口说明.md》“语音合成服务”章节。
// ============================================================================
#include "guide_pulse/tts_client.hpp"

#include <utility>

#include <aimdk_msgs/msg/common_request.hpp>
#include <aimdk_msgs/msg/common_state.hpp>
#include <aimdk_msgs/msg/tts_priority_level.hpp>
#include <aimdk_msgs/srv/play_tts.hpp>

namespace guide_pulse
{

using aimdk_msgs::msg::CommonRequest;
using aimdk_msgs::msg::CommonState;
using aimdk_msgs::msg::TtsPriorityLevel;
using aimdk_msgs::srv::PlayTts;

TtsClient::TtsClient(
  rclcpp::Node::SharedPtr node, std::string service_name, std::string domain,
  int request_timeout_seconds, std::string stop_text)
: node_(std::move(node)),
  service_name_(std::move(service_name)),
  domain_(std::move(domain)),
  request_timeout_(std::chrono::seconds(std::max(1, request_timeout_seconds))),
  stop_text_(std::move(stop_text))
{
  // 独立回调组：executor 线程处理回执，工作线程只阻塞在 future 上，互不干扰
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  client_ = node_->create_client<PlayTts>(service_name_, rmw_qos_profile_services_default,
      callback_group_);
}

bool TtsClient::wait_for_service(std::chrono::seconds timeout)
{
  return client_->wait_for_service(timeout);
}

TtsClient::Result TtsClient::call_play_tts(
  const std::string & text, int priority_level, int priority_weight,
  bool is_interrupted, const std::string & trace_id_suffix)
{
  Result result;

  if (!client_->service_is_ready()) {
    result.service_failed = true;  // 播放子系统不可达：上层回 playback_unavailable
    result.detail = "TTS service not available: " + service_name_;
    return result;
  }

  auto request = std::make_shared<PlayTts::Request>();
  // CommonRequest.header.stamp
  request->header.header.stamp = node_->now();
  request->tts_req.text = text;
  request->tts_req.priority_level.value =
    static_cast<std::uint8_t>(priority_level);
  request->tts_req.priority_weight = static_cast<std::uint32_t>(priority_weight);
  request->tts_req.domain = domain_;
  request->tts_req.trace_id = trace_id_suffix;
  request->tts_req.is_interrupted = is_interrupted;

  // 受理耗时可暴露长文本逼近超时的情况；成败与耗时由调用方日志输出
  const auto sent_at = std::chrono::steady_clock::now();
  auto future = client_->async_send_request(request);
  if (future.wait_for(request_timeout_) != std::future_status::ready) {
    // 超时后移除挂起请求，避免迟到的回执干扰后续调用
    client_->remove_pending_request(future);
    result.service_failed = true;  // 调用超时：上层回 playback_unavailable
    result.detail =
      "PlayTts timeout after " + std::to_string(request_timeout_.count()) + "s";
    return result;
  }

  const auto response = future.get();
  const auto round_trip_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - sent_at).count();
  if (!response) {
    result.detail = "PlayTts returned empty response";
    return result;
  }

  result.code = response->header.header.code;
  result.message = response->header.message;
  result.is_success = response->tts_resp.is_success;
  result.error_message = response->tts_resp.error_message;
  result.estimated_duration_ms = response->tts_resp.estimated_duration;

  // 回执判定与 SDK 示例一致：code==0 或 status==SUCCESS，且业务 is_success
  const auto status = response->header.status.value;
  const bool framework_ok = (result.code == 0) || (status == CommonState::SUCCESS);
  result.ok = framework_ok && result.is_success;

  if (result.ok) {
    result.elapsed_ms = round_trip_ms;
  } else if (result.detail.empty()) {
    result.detail = "PlayTts rejected by robot: " +
      (!result.error_message.empty() ? result.error_message : result.message);
  }
  return result;
}

TtsClient::Result TtsClient::play(const TtsCommand & command)
{
  return call_play_tts(
    command.text, command.priority_level, command.priority_weight,
    command.is_interrupted, command.trace_id);
}

TtsClient::Result TtsClient::stop(const std::string & trace_id)
{
  // 停止 = 最高优先级抢占：SAFETY_L10(10)*100+99=1099 必然不小于任何进行中任务，
  // 配合 is_interrupted=true 依据仲裁矩阵立即掐断当前播报。
  // stop_text 为 SSML 静音段（默认见 config/guide_pulse.yaml，勿用纯标点——任务
  // 无法完结会把仲裁通道卡住 30-60 秒）。
  std::string id = trace_id.empty() ? generate_uuid_v7() : trace_id;
  return call_play_tts(
    stop_text_,
    TtsPriorityLevel::SAFETY_L10,
    99,
    true,
    "stop-" + id);
}

}  // namespace guide_pulse
