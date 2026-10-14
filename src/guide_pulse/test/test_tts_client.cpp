// ============================================================================
// guide_pulse SDK TTS 客户端单元测试：
// 进程内拉起 Mock PlayTts 服务端（真实 aimdk_msgs 服务），
// 校验请求字段映射、回执判定与停止策略（SAFETY_L10 抢占）。
// ============================================================================
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <aimdk_msgs/msg/common_state.hpp>
#include <aimdk_msgs/msg/tts_priority_level.hpp>
#include <aimdk_msgs/srv/play_tts.hpp>

#include "guide_pulse/tts_client.hpp"

namespace
{

using aimdk_msgs::srv::PlayTts;

// ---------------------------------------------------------------------------
// Mock 服务端：记录收到的请求，按预设返回回执
// ---------------------------------------------------------------------------
class MockTtsServer final : public rclcpp::Node
{
public:
  MockTtsServer()
  : Node("mock_tts_server")
  {
    service_ = create_service<PlayTts>(
      "/test/mock_play_tts",
      [this](
        const std::shared_ptr<PlayTts::Request> request,
        std::shared_ptr<PlayTts::Response> response) {
        last_request_ = *request;
        request_count_++;

        response->header.header.stamp = now();
        response->header.header.code = next_code_.load();
        response->header.status.value = next_code_.load() == 0
          ? aimdk_msgs::msg::CommonState::SUCCESS
          : aimdk_msgs::msg::CommonState::FAILURE;
        response->header.message = next_code_.load() == 0 ? "ok" : "rejected";
        response->tts_resp.text = request->tts_req.text;
        response->tts_resp.is_success = next_is_success_.load();
        response->tts_resp.error_message =
          next_is_success_.load() ? "" : "synthesis failed";
      });
  }

  PlayTts::Request last_request_;
  std::atomic<int> request_count_{0};
  std::atomic<bool> next_is_success_{true};
  std::atomic<int64_t> next_code_{0};

private:
  rclcpp::Service<PlayTts>::SharedPtr service_;
};

class TtsClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    server_ = std::make_shared<MockTtsServer>();
    client_node_ = std::make_shared<rclcpp::Node>("tts_client_test_node");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(server_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    client_ = std::make_unique<guide_pulse::TtsClient>(
      client_node_, "/test/mock_play_tts", "unit_test", 2, "。");
    ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override
  {
    // 必须先停 executor 再销毁客户端：客户端的 guard condition 仍在
    // executor 的 wait set 中，先销毁会让 spin 线程触发
    // "guard condition implementation is invalid" 并中止进程。
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    client_.reset();
    executor_.reset();
    server_.reset();
    client_node_.reset();
  }

  guide_pulse::TtsCommand make_play_command()
  {
    guide_pulse::TtsCommand cmd;
    cmd.text = "测试播报";
    cmd.priority_level = 6;
    cmd.priority_weight = 10;
    cmd.is_interrupted = true;
    cmd.trace_id = "trace-001";
    return cmd;
  }

  // 注意：Node::SharedPtr 是 shared_ptr<Node>，这里必须显式指定派生类型
  std::shared_ptr<MockTtsServer> server_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::unique_ptr<guide_pulse::TtsClient> client_;
};

// play(): 请求字段完整映射到 SDK PlayTts 结构
TEST_F(TtsClientTest, PlayMapsRequestFields)
{
  const auto result = client_->play(make_play_command());
  ASSERT_TRUE(result.ok) << result.detail;

  EXPECT_EQ(server_->last_request_.tts_req.text, "测试播报");
  EXPECT_EQ(
    server_->last_request_.tts_req.priority_level.value,
    aimdk_msgs::msg::TtsPriorityLevel::INTERACTION_L6);
  EXPECT_EQ(server_->last_request_.tts_req.priority_weight, 10u);
  EXPECT_TRUE(server_->last_request_.tts_req.is_interrupted);
  EXPECT_EQ(server_->last_request_.tts_req.domain, "unit_test");
  EXPECT_EQ(server_->last_request_.tts_req.trace_id, "trace-001");
}

// play(): 服务返回 is_success=false 时判定失败（业务拒绝，非服务不可达：
// 上层按本地重试策略处理，仍回 tts.result accepted）
TEST_F(TtsClientTest, PlayFailureWhenBusinessRejects)
{
  server_->next_is_success_ = false;
  const auto result = client_->play(make_play_command());
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.service_failed);
  EXPECT_FALSE(result.is_success);
  EXPECT_EQ(result.error_message, "synthesis failed");
  EXPECT_FALSE(result.detail.empty());
}

// play(): 框架层 code!=0 时判定失败（业务拒绝，非服务不可达）
TEST_F(TtsClientTest, PlayFailureOnFrameworkCode)
{
  server_->next_code_ = 500;
  const auto result = client_->play(make_play_command());
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.service_failed);
  EXPECT_EQ(result.code, 500);
}

// play(): 回执成功时携带 code/message
TEST_F(TtsClientTest, PlaySuccessCarriesReceipt)
{
  const auto result = client_->play(make_play_command());
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.code, 0);
  EXPECT_EQ(result.message, "ok");
  EXPECT_TRUE(result.error_message.empty());
}

// stop(): 必须以最高优先级 SAFETY_L10 + 最大加权 + 抢占方式调用
TEST_F(TtsClientTest, StopUsesHighestPriorityPreemption)
{
  const auto result = client_->stop("trace-stop");
  ASSERT_TRUE(result.ok) << result.detail;

  EXPECT_EQ(server_->last_request_.tts_req.text, "。");
  EXPECT_EQ(
    server_->last_request_.tts_req.priority_level.value,
    aimdk_msgs::msg::TtsPriorityLevel::SAFETY_L10);
  EXPECT_EQ(server_->last_request_.tts_req.priority_weight, 99u);
  EXPECT_TRUE(server_->last_request_.tts_req.is_interrupted);
  EXPECT_EQ(server_->last_request_.tts_req.trace_id, "stop-trace-stop");
}

// 服务不可达：超时返回失败并给出明确描述
TEST(TtsClientNoServerTest, PlayTimesOutWhenServiceMissing)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("tts_client_noserver_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  guide_pulse::TtsClient client(
    node, "/test/no_such_tts_service", "unit_test", 1, "。");
  guide_pulse::TtsCommand cmd;
  cmd.text = "hi";
  cmd.trace_id = "m";
  const auto result = client.play(cmd);
  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.service_failed);  // 服务不可达：上层回 playback_unavailable
  EXPECT_FALSE(result.detail.empty());  // 应包含服务不可用/超时描述

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
}

}  // namespace
