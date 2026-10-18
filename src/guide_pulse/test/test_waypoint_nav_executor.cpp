// ============================================================================
// WaypointNavExecutor 单元测试：进程内拉起 Mock Navigate action 服务端
//（liwei_nav_msgs/action/Navigate + /query 服务），校验受理/被拒试探、
// 终态判定（SUCCEEDED/FAILED/REJECTED）、set_pose 受理与验收轮询、取消语义。
// 夹具模式沿用 test_tts_client.cpp（进程内服务 + MultiThreadedExecutor）。
// 对齐真机语义（NavigateAction.md）：被拒的 goal 也先「已接受」，拒因走
// result 的 state=REJECTED（毫秒级）；set_pose 的 result 即验收终态。
// ============================================================================
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <liwei_nav_msgs/action/navigate.hpp>
#include <liwei_nav_msgs/srv/query.hpp>

#include "guide_pulse/waypoint_nav_executor.hpp"

namespace
{

using Navigate = liwei_nav_msgs::action::Navigate;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;
using Query = liwei_nav_msgs::srv::Query;

// ---------------------------------------------------------------------------
// Mock 导航 action 服务端：记录 goal 字段，execute 按预设立即给终态
//（REJECTED/FAILED/SUCCEEDED）或挂起等测试触发；cancel 终结挂起 goal；
// /query 回带合法航点清单（set_pose 拒因 point_not_found 的结构化判定依据）
// ---------------------------------------------------------------------------
class MockNavigateServer final : public rclcpp::Node
{
public:
  MockNavigateServer()
  : Node("mock_navigate_server")
  {
    action_server_ = rclcpp_action::create_server<Navigate>(
      this, "/test/navigate",
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Navigate::Goal> goal) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_command_ = goal->command;
          last_waypoints_ = goal->waypoints;
          last_return_to_quadruped_ = goal->return_to_quadruped;
          goal_count_++;
        }
        // 真机语义：被拒也 accept（拒因走 result 的 REJECTED）
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandleNavigate> goal_handle) {
        cancel_count_++;
        // Humble：本回调返回 ACCEPT 后 rcl 才把 goal 转 CANCELING——
        // 直接在 EXECUTING 上调 canceled() 是非法转移。终结交给
        // 一次性 timer（回调返回后触发，goal 已在 CANCELING）
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle != pending_gh_) {
          return rclcpp_action::CancelResponse::ACCEPT;  // 已终结：忽略
        }
        cancel_timer_ = create_wall_timer(
          std::chrono::milliseconds(10), [this, goal_handle]() {
            cancel_timer_->cancel();  // 一次性
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_gh_ != goal_handle) {
              return;
            }
            pending_gh_.reset();
            auto result = std::make_shared<Navigate::Result>();
            result->state = "CANCELLED";
            result->message = "cancelled";
            goal_handle->canceled(result);
          });
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandleNavigate> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Navigate::Result>();
        std::lock_guard<std::mutex> lock(mutex_);
        const auto auto_result = auto_result_;
        result->message = (auto_result == "REJECTED") ? reject_message_
          : (auto_result == "FAILED" ? fail_reason_ : "ok");
        result->state = auto_result.empty() ? "" : auto_result;
        result->success = (auto_result == "SUCCEEDED");
        if (auto_result.empty()) {
          pending_gh_ = goal_handle;  // 挂起：等测试触发/取消后终结
          return;
        }
        finish_no_lock(goal_handle, result, auto_result);
      });
    // /query：只读状态快照（available 清单供拒因分类）
    query_service_ = create_service<Query>(
      "/test/navigate/query",
      [this](
        const std::shared_ptr<Query::Request>,
        std::shared_ptr<Query::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        response->available = available_;
        response->state = "IDLE";
      });
  }

  // 预设 execute 结果：SUCCEEDED（默认）/ FAILED / REJECTED / ""（挂起）
  void set_auto_result(const std::string & r, const std::string & message = "")
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_result_ = r;
    if (r == "REJECTED") {
      reject_message_ = message;
    } else if (r == "FAILED") {
      fail_reason_ = message;
    }
  }

  // 挂起 goal 的测试侧终结（真机「走到/失败」对应面）
  void finish_succeed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_gh_) {
      return;
    }
    auto result = std::make_shared<Navigate::Result>();
    result->state = "SUCCEEDED";
    result->success = true;
    result->message = "ok";
    finish_no_lock(pending_gh_, result, "SUCCEEDED");
    pending_gh_.reset();
  }
  void finish_fail(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_gh_) {
      return;
    }
    auto result = std::make_shared<Navigate::Result>();
    result->state = "FAILED";
    result->success = false;
    result->message = reason;
    finish_no_lock(pending_gh_, result, "FAILED");
    pending_gh_.reset();
  }

  // 服务端合法航点名清单（query 回带；默认含 A/B/home 三点）
  void set_available(const std::vector<std::string> & names)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    available_ = names;
  }

  // goal 字段快照（goal 回调线程写、测试线程读：mutex 保护）
  std::string last_command_;               // 最近 goal 的 command（go/set_pose）
  std::vector<std::string> last_waypoints_;
  bool last_return_to_quadruped_{false};
  std::atomic<int> goal_count_{0};
  std::atomic<int> cancel_count_{0};

private:
  // 持锁终结挂起/即时 goal（status 码与 state 对齐：REJECTED/FAILED -> ABORTED）
  void finish_no_lock(
    const std::shared_ptr<GoalHandleNavigate> & gh,
    const std::shared_ptr<Navigate::Result> & result, const std::string & auto_result)
  {
    if (auto_result == "SUCCEEDED") {
      gh->succeed(result);
    } else {
      gh->abort(result);  // REJECTED / FAILED（真机两者都是 goal ABORTED）
    }
  }

  std::mutex mutex_;
  std::string auto_result_{"SUCCEEDED"};
  std::string reject_message_{"任务冲突：已有任务在跑"};
  std::string fail_reason_{"Nav2 goal aborted"};
  std::vector<std::string> available_{"A", "B", "home"};
  std::shared_ptr<GoalHandleNavigate> pending_gh_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Service<Query>::SharedPtr query_service_;
};

class WaypointNavExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    server_ = std::make_shared<MockNavigateServer>();
    client_node_ = std::make_shared<rclcpp::Node>("waypoint_nav_executor_test_node");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(server_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    // 轮询分片 250ms；超时 5s
    executor_nav_ = std::make_unique<guide_pulse::WaypointNavExecutor>(
      client_node_, "/test/navigate", "/test/navigate/query",
      std::chrono::seconds(5));
  }

  void TearDown() override
  {
    // 先停 executor 再销毁客户端（guard condition 仍在 executor wait set 中）；
    // 挂起 goal 的导航线程至多等一个 result 分片超时后自行退出
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

  std::shared_ptr<MockNavigateServer> server_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::unique_ptr<guide_pulse::WaypointNavExecutor> executor_nav_;

  std::atomic_bool called_{false};
  guide_pulse::NavOutcome outcome_{};
};

TEST_F(WaypointNavExecutorTest, ArriveOnSucceeded)
{
  // 夹具默认：goal 受理后直接 SUCCEEDED（任务很快完成）
  executor_nav_->goto_point(
    "3号展台", [this](guide_pulse::NavOutcome o) {
      outcome_ = o;
      called_.store(true);
    });
  ASSERT_TRUE(wait_callback()) << "未收到到达回调";
  EXPECT_TRUE(outcome_.ok);
  EXPECT_NE(outcome_.detail.find("3号展台"), std::string::npos);
  // goal 字段：command=go + 单航点 + 不变狗形（讲解机器人保持人形）
  EXPECT_EQ(server_->last_command_, "go");
  EXPECT_EQ(server_->last_waypoints_, (std::vector<std::string>{"3号展台"}));
  EXPECT_FALSE(server_->last_return_to_quadruped_);
}

TEST_F(WaypointNavExecutorTest, FailOnFailedWithReason)
{
  server_->set_auto_result("FAILED", "Nav2 goal aborted");
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

// 被拒的 goal 也先「已接受」（真机语义），拒因毫秒级走 result REJECTED：
// 执行器在受理试探窗口内现形为失败，不误判为在途
TEST_F(WaypointNavExecutorTest, RejectMapsToFailure)
{
  server_->set_auto_result("REJECTED", "路线里没有叫 '不存在的点' 的航点");
  executor_nav_->goto_point(
    "不存在的点", [this](guide_pulse::NavOutcome o) {
      outcome_ = o;
      called_.store(true);
    });
  ASSERT_TRUE(wait_callback());
  EXPECT_FALSE(outcome_.ok);
  EXPECT_NE(outcome_.detail.find("导航任务被拒"), std::string::npos);
}

TEST_F(WaypointNavExecutorTest, RunningThenSucceeded)
{
  // goal 挂起（RUNNING：出发前 posture/nav2_wake 等阶段或行走中）
  // → 测试触发 SUCCEEDED：终态判定正确
  server_->set_auto_result("");
  executor_nav_->goto_point(
    "充电桩A", [this](guide_pulse::NavOutcome o) {
      outcome_ = o;
      called_.store(true);
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(called_.load());  // 在途期间不回调
  server_->finish_succeed();
  ASSERT_TRUE(wait_callback());
  EXPECT_TRUE(outcome_.ok);
}

TEST_F(WaypointNavExecutorTest, CancelDropsCallback)
{
  server_->set_auto_result("");
  executor_nav_->goto_point(
    "A展台", [this](guide_pulse::NavOutcome) {called_.store(true);});
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  executor_nav_->cancel();
  // 取消链路：250ms 分片检测 -> cancel_goal -> server 转 CANCELING ->
  // mock 终结 CANCELLED -> client 收 result（DDS 往返数跳，留足余量）
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  EXPECT_FALSE(called_.load());  // 取消后不回调（语义由 cancel 定义）
  EXPECT_GE(server_->cancel_count_.load(), 1);  // action cancel 已下发
}

// 重定位：result（验收完成才回）在受理试探窗口内回来 = set_pose 直接
// 带回终态；poll 首调即回缓存 true
TEST_F(WaypointNavExecutorTest, SetPoseAcceptedIsTerminal)
{
  const auto outcome = executor_nav_->set_pose("A");
  EXPECT_TRUE(outcome.ok);
  EXPECT_NE(outcome.detail.find("A"), std::string::npos);
  // goal 字段：command=set_pose + 单航点 + 保持人形（服务端硬约束）
  EXPECT_EQ(server_->last_command_, "set_pose");
  EXPECT_EQ(server_->last_waypoints_, (std::vector<std::string>{"A"}));
  EXPECT_FALSE(server_->last_return_to_quadruped_);
  const auto verified = executor_nav_->poll_set_pose_verified("A");
  ASSERT_TRUE(verified.has_value());
  EXPECT_TRUE(*verified);
}

// 重定位被拒（导航在途/非站立人形/定位未起）：失败 + 服务端 message 透传；
// 点名在 available 清单中 = 非 point_not_found（状态冲突类拒因）
TEST_F(WaypointNavExecutorTest, SetPoseRejectedMapsToFailure)
{
  server_->set_auto_result("REJECTED", "任务冲突：已有任务在跑");
  const auto outcome = executor_nav_->set_pose("A");
  EXPECT_FALSE(outcome.ok);
  EXPECT_NE(outcome.detail.find("set_pose 被拒"), std::string::npos);
  EXPECT_NE(outcome.detail.find("任务冲突"), std::string::npos);  // message 透传
  EXPECT_FALSE(outcome.point_not_found);  // A 在 available 清单：非点位名错
}

// 重定位被拒且点名不在 available 清单：point_not_found 置位（query 结构化
// 分类，桥接层据此映射 relocate.result 的 point_not_found）
TEST_F(WaypointNavExecutorTest, SetPoseRejectedUnknownWaypointClassified)
{
  server_->set_auto_result("REJECTED", "路线里没有叫 'X9' 的航点");
  const auto outcome = executor_nav_->set_pose("X9");
  EXPECT_FALSE(outcome.ok);
  EXPECT_TRUE(outcome.point_not_found);  // X9 不在 {A, B, home} 清单
}

// 重定位受理后验收结果异步回（真机 ~1.5s 窗口）：set_pose 受理即返回，
// poll 三态——nullopt（未出）-> true（验收通过）；再一轮 -> false（验收失败）
TEST_F(WaypointNavExecutorTest, SetPosePendingThenVerified)
{
  using R = std::optional<bool>;
  // mock 终结 -> client 收 result 要经 DDS 往返 + executor 调度，
  // 每次触发后留 settle 余量再 poll
  server_->set_auto_result("");  // 挂起：验收未出
  ASSERT_TRUE(executor_nav_->set_pose("A").ok);
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(std::nullopt));
  server_->finish_succeed();  // 验收通过
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(true));

  // 第二轮：验收失败（注入后位姿仍不匹配）
  ASSERT_TRUE(executor_nav_->set_pose("A").ok);
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(std::nullopt));
  server_->finish_fail("注入后仍差 0.42 m。");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(false));
}

}  // namespace
