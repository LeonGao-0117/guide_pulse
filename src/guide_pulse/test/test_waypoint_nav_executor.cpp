// ============================================================================
// WaypointNavExecutor 单元测试：进程内拉起 Mock 航点巡航服务端
//（go_to_waypoints / get_status / cancel 三个真实服务），校验
// 受理拒绝、终态判定（SUCCEEDED/FAILED）、RUNNING->SUCCEEDED 转移与取消语义。
// 夹具模式沿用 test_tts_client.cpp（进程内服务 + MultiThreadedExecutor）。
// ============================================================================
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <waypoint_nav_msgs/srv/get_waypoint_status.hpp>
#include <waypoint_nav_msgs/srv/go_to_waypoints.hpp>

#include "guide_pulse/waypoint_nav_executor.hpp"

namespace
{

using GoToWaypoints = waypoint_nav_msgs::srv::GoToWaypoints;
using GetWaypointStatus = waypoint_nav_msgs::srv::GetWaypointStatus;
using Trigger = std_srvs::srv::Trigger;

// ---------------------------------------------------------------------------
// Mock 航点巡航服务端：记录 go_to_waypoints 请求，按预设返回受理结果；
// get_status 返回可动态修改的状态；cancel 置 CANCELLED
// ---------------------------------------------------------------------------
class MockWaypointServer final : public rclcpp::Node
{
public:
  MockWaypointServer()
  : Node("mock_waypoint_server")
  {
    goto_service_ = create_service<GoToWaypoints>(
      "/test/waypoint_nav/go_to_waypoints",
      [this](
        const std::shared_ptr<GoToWaypoints::Request> request,
        std::shared_ptr<GoToWaypoints::Response> response) {
        last_waypoints_ = request->waypoints;
        last_return_to_quadruped_ = request->return_to_quadruped;
        goto_count_++;
        response->accepted = next_accepted_.load();
        response->message = next_accepted_.load() ? "ok" : "任务冲突：已有任务在跑";
        response->plan = request->waypoints;
        if (next_accepted_.load()) {
          // 受理即开始任务：置 RUNNING；auto_terminal 非空则直接进入该终态
          //（模拟"任务很快完成/失败"；持续 RUNNING 由测试切状态）
          std::lock_guard<std::mutex> lock(mutex_);
          set_state_no_lock(auto_terminal_.empty() ? "RUNNING" : auto_terminal_);
        }
      });
    status_service_ = create_service<GetWaypointStatus>(
      "/test/waypoint_nav/get_status",
      [this](
        const std::shared_ptr<GetWaypointStatus::Request>,
        std::shared_ptr<GetWaypointStatus::Response> response) {
        response->state = state();
        response->reason = reason();
        status_count_++;
      });
    cancel_service_ = create_service<Trigger>(
      "/test/waypoint_nav/cancel",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {
        cancel_count_++;
        set_state("CANCELLED");
        response->success = true;
        response->message = "cancelled";
      });
  }

  // 状态/原因由 executor 线程（服务回调）写、测试线程读：mutex 保护
  //（std::atomic<std::string> 非平凡可拷贝，不可用）
  void set_state(const std::string & s)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = s;
  }
  std::string state()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }
  void set_reason(const std::string & r)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reason_ = r;
  }
  std::string reason()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return reason_;
  }
  void set_state_no_lock(const std::string & s) {state_ = s;}

  std::atomic<bool> next_accepted_{true};
  // 受理后的目标状态：默认直接 SUCCEEDED（快速到达）；"" = 停在 RUNNING
  //（测试随后切终态）；"FAILED" = 受理即失败
  void set_auto_terminal(const std::string & t)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_terminal_ = t;
  }
  std::atomic<int> goto_count_{0};
  std::atomic<int> status_count_{0};
  std::atomic<int> cancel_count_{0};
  std::atomic<bool> last_return_to_quadruped_{false};
  std::vector<std::string> last_waypoints_;

private:
  std::mutex mutex_;
  std::string state_{"IDLE"};
  std::string reason_;
  std::string auto_terminal_{"SUCCEEDED"};
  rclcpp::Service<GoToWaypoints>::SharedPtr goto_service_;
  rclcpp::Service<GetWaypointStatus>::SharedPtr status_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
};

class WaypointNavExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    server_ = std::make_shared<MockWaypointServer>();
    client_node_ = std::make_shared<rclcpp::Node>("waypoint_nav_executor_test_node");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(server_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    // 轮询 50ms 加速测试；超时 5s
    executor_nav_ = std::make_unique<guide_pulse::WaypointNavExecutor>(
      client_node_, "/test/waypoint_nav/go_to_waypoints",
      "/test/waypoint_nav/get_status", "/test/waypoint_nav/cancel",
      std::chrono::milliseconds(50), std::chrono::seconds(5));
  }

  void TearDown() override
  {
    // 先停 executor 再销毁客户端（guard condition 仍在 executor wait set 中）
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_nav_.reset();
    executor_.reset();
    server_.reset();
    client_node_.reset();
  }

  // 等待回调落地（最多 seconds 秒）；返回是否收到
  bool wait_callback(int seconds = 3)
  {
    for (int i = 0; i < seconds * 20; ++i) {
      if (called_.load()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return called_.load();
  }

  std::shared_ptr<MockWaypointServer> server_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::unique_ptr<guide_pulse::WaypointNavExecutor> executor_nav_;

  std::atomic_bool called_{false};
  guide_pulse::NavOutcome outcome_{};
};

TEST_F(WaypointNavExecutorTest, ArriveOnSucceeded)
{
  // 夹具默认：受理后直接 SUCCEEDED（任务很快完成）
  executor_nav_->goto_point(
    "3号展台", [this](guide_pulse::NavOutcome o) {
      outcome_ = o;
      called_.store(true);
    });
  ASSERT_TRUE(wait_callback()) << "未收到到达回调";
  EXPECT_TRUE(outcome_.ok);
  EXPECT_NE(outcome_.detail.find("3号展台"), std::string::npos);
  // 请求字段：单航点 + 不变狗形（讲解机器人保持人形）
  EXPECT_EQ(server_->last_waypoints_, (std::vector<std::string>{"3号展台"}));
  EXPECT_FALSE(server_->last_return_to_quadruped_.load());
}

TEST_F(WaypointNavExecutorTest, FailOnFailedWithReason)
{
  server_->set_auto_terminal("FAILED");
  server_->set_reason("Nav2 goal aborted");
  executor_nav_->goto_point(
    "东门", [this](guide_pulse::NavOutcome o) {
      outcome_ = o;
      called_.store(true);
    });
  ASSERT_TRUE(wait_callback());
  EXPECT_FALSE(outcome_.ok);
  EXPECT_NE(outcome_.detail.find("FAILED"), std::string::npos);
  EXPECT_NE(outcome_.detail.find("Nav2 goal aborted"), std::string::npos);
}

TEST_F(WaypointNavExecutorTest, RejectMapsToFailure)
{
  server_->next_accepted_.store(false);
  server_->set_state("IDLE");
  executor_nav_->goto_point(
    "不存在的点", [this](guide_pulse::NavOutcome o) {
      outcome_ = o;
      called_.store(true);
    });
  ASSERT_TRUE(wait_callback());
  EXPECT_FALSE(outcome_.ok);
  EXPECT_NE(outcome_.detail.find("航点任务被拒"), std::string::npos);
  EXPECT_EQ(server_->status_count_.load(), 0);  // 被拒不进入轮询
}

TEST_F(WaypointNavExecutorTest, RunningThenSucceeded)
{
  // 受理后保持 RUNNING（等位姿/行走中）→ 测试切 SUCCEEDED：转移仍正确判达
  server_->set_auto_terminal("");
  executor_nav_->goto_point(
    "充电桩A", [this](guide_pulse::NavOutcome o) {
      outcome_ = o;
      called_.store(true);
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(called_.load());  // RUNNING 期间不回调
  server_->set_state("SUCCEEDED");
  ASSERT_TRUE(wait_callback());
  EXPECT_TRUE(outcome_.ok);
}

TEST_F(WaypointNavExecutorTest, CancelDropsCallback)
{
  server_->set_auto_terminal("");
  executor_nav_->goto_point(
    "A展台", [this](guide_pulse::NavOutcome) {called_.store(true);});
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  executor_nav_->cancel();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_FALSE(called_.load());  // 取消后不回调（语义由 cancel 定义）
  EXPECT_GE(server_->cancel_count_.load(), 1);  // cancel Trigger 已下发
}

}  // namespace
