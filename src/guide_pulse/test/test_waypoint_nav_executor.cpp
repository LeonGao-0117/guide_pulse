// ============================================================================
// WaypointNavExecutor 单元测试：进程内拉起 Mock 航点巡航服务端
//（go_to_waypoints / cancel 两个服务 + status 话题推送），校验
// 受理拒绝、终态判定（SUCCEEDED/FAILED）、RUNNING->SUCCEEDED 转移与取消语义。
// 夹具模式沿用 test_tts_client.cpp（进程内服务 + MultiThreadedExecutor）。
// 状态通道对齐真机 follower_node：不再提供 get_status 服务，每次状态变化
// 向 ~/status（std_msgs/String，latched JSON 快照）推一帧。
// ============================================================================
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <waypoint_nav_msgs/srv/go_to_waypoints.hpp>

#include "guide_pulse/waypoint_nav_executor.hpp"

namespace
{

using GoToWaypoints = waypoint_nav_msgs::srv::GoToWaypoints;
using Trigger = std_srvs::srv::Trigger;

// ---------------------------------------------------------------------------
// Mock 航点巡航服务端：记录 go_to_waypoints 请求，按预设返回受理结果；
// 状态变化即向 status 话题推 JSON 快照帧；cancel 置 CANCELLED
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
        last_action_ = request->action;
        goto_count_++;
        response->accepted = next_accepted_.load();
        response->message = next_accepted_.load() ? "ok" : "任务冲突：已有任务在跑";
        // 真机语义（follower_node._srv_goto）：available 无论受理与否都回带
        // 全部合法航点名（set_pose 拒因 point_not_found 的结构化判定依据）
        response->available = available();
        response->plan = request->waypoints;
        if (next_accepted_.load()) {
          // 受理即开始任务：置 RUNNING；auto_terminal 非空则直接进入该终态
          //（模拟"任务很快完成/失败"；持续 RUNNING 由测试切状态）。
          // 受理帧先于服务应答发布（真机 _begin_set_pose 同款时序）
          std::lock_guard<std::mutex> lock(mutex_);
          const auto terminal = auto_terminal_;
          snapshot_["state"] = terminal.empty() ? "RUNNING" : terminal;
          if (request->action == "set_pose") {
            // 真机 follower_node._begin_set_pose：受理时同步覆盖 set_pose
            // 记录（verified 重置 null），应答前先推这帧
            snapshot_["set_pose"] = {
              {"waypoint", request->waypoints.empty() ? "" : request->waypoints[0]},
              {"sent", true}, {"verified", nullptr}};
          }
          publish_no_lock();
        }
      });
    cancel_service_ = create_service<Trigger>(
      "/test/waypoint_nav/cancel",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {
        cancel_count_++;
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_["state"] = "CANCELLED";
        publish_no_lock();
        response->success = true;
        response->message = "cancelled";
      });
    // status 话题：QoS 与真机 _LATCHED 对齐（执行器订阅 transient_local，
    // 这里 volatile 发布会 QoS 不兼容、安静连不上）
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/test/waypoint_nav/status",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  // 状态/原因由 executor 线程（服务回调）写、测试线程读：mutex 保护
  //（std::atomic<std::string> 非平凡可拷贝，不可用）
  void set_state(const std::string & s)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_["state"] = s;
    publish_no_lock();
  }
  void set_reason(const std::string & r)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_["reason"] = r;
    publish_no_lock();
  }

  // 服务端合法航点名清单（available 回带；默认含 A/B/home 三点）
  void set_available(const std::vector<std::string> & names)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    available_ = names;
  }
  std::vector<std::string> available()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return available_;
  }

  // 注入任意快照字段（set_pose.verified 等）：合法 JSON 合并进快照后
  // 推帧；非法 JSON 原样推坏帧（测执行器解析容错——真机不会发，防御在）
  void set_status_json(const std::string & j)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      snapshot_.update(nlohmann::json::parse(j));
      publish_no_lock();
    } catch (const nlohmann::json::exception &) {
      raw_publish_no_lock(j);
    }
  }

  std::atomic<bool> next_accepted_{true};
  // 受理后的目标状态：默认直接 SUCCEEDED（快速到达）；"" = 停在 RUNNING
  //（测试随后切终态）；"FAILED" = 受理即失败
  void set_auto_terminal(const std::string & t)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_terminal_ = t;
  }
  std::atomic<int> goto_count_{0};
  std::atomic<int> cancel_count_{0};
  std::atomic<bool> last_return_to_quadruped_{false};
  std::string last_action_;  // 最近一次 go_to_waypoints 请求的 action（"" = go）
  std::vector<std::string> last_waypoints_;

private:
  // 持锁推帧（真机 _publish_status 同款：状态一变就发全量快照）
  void publish_no_lock()
  {
    std_msgs::msg::String msg;
    msg.data = snapshot_.dump();
    status_pub_->publish(msg);
  }
  void raw_publish_no_lock(const std::string & raw)
  {
    std_msgs::msg::String msg;
    msg.data = raw;
    status_pub_->publish(msg);
  }

  std::mutex mutex_;
  nlohmann::json snapshot_{{"state", "IDLE"}, {"reason", ""}};
  std::string auto_terminal_{"SUCCEEDED"};
  std::vector<std::string> available_{"A", "B", "home"};
  rclcpp::Service<GoToWaypoints>::SharedPtr goto_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
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
      "/test/waypoint_nav/status", "/test/waypoint_nav/cancel",
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

  // 等订阅端消化刚推送的帧（进程内 DDS 传递 + executor 调度余量）
  void settle() const
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
}

// 重定位：受理即返回（不进任务状态机）；请求字段 action=set_pose
TEST_F(WaypointNavExecutorTest, SetPoseAcceptedIsTerminal)
{
  const auto outcome = executor_nav_->set_pose("A");
  EXPECT_TRUE(outcome.ok);
  EXPECT_NE(outcome.detail.find("A"), std::string::npos);
  // 请求字段：单航点 + set_pose 动作 + 保持人形（服务端硬约束）
  EXPECT_EQ(server_->last_waypoints_, (std::vector<std::string>{"A"}));
  EXPECT_EQ(server_->last_action_, "set_pose");
  EXPECT_FALSE(server_->last_return_to_quadruped_.load());
}

// 重定位被拒（导航在途/非站立人形/定位未起）：失败 + 服务端 message 透传；
// 点名在 available 清单中 = 非 point_not_found（状态冲突类拒因）
TEST_F(WaypointNavExecutorTest, SetPoseRejectedMapsToFailure)
{
  server_->next_accepted_.store(false);
  const auto outcome = executor_nav_->set_pose("A");
  EXPECT_FALSE(outcome.ok);
  EXPECT_NE(outcome.detail.find("set_pose 被拒"), std::string::npos);
  EXPECT_NE(outcome.detail.find("任务冲突"), std::string::npos);  // message 透传
  EXPECT_FALSE(outcome.point_not_found);  // A 在 available 清单：非点位名错
}

// 重定位被拒且点名不在 available 清单：point_not_found 置位（结构化分类，
// 桥接层据此映射 relocate.result 的 point_not_found）
TEST_F(WaypointNavExecutorTest, SetPoseRejectedUnknownWaypointClassified)
{
  server_->next_accepted_.store(false);
  const auto outcome = executor_nav_->set_pose("X9");
  EXPECT_FALSE(outcome.ok);
  EXPECT_TRUE(outcome.point_not_found);  // X9 不在 {A, B, home} 清单
}

// 验收查询（2026-10-17：重定位终态 = set_pose.verified）：status 快照的
// set_pose 记录三态——无记录/航点名不符/verified 仍 null 均为「继续等」
//（nullopt），verified 出 bool 才是终态。注：本用例直接推帧（无受理），
// watermark=0，凡到达的帧都算新帧；受理-防串窗口由下面用例覆盖
TEST_F(WaypointNavExecutorTest, PollSetPoseVerifiedReadsStatusJson)
{
  using R = std::optional<bool>;
  // 1. 无 set_pose 记录（受理前/其他航点残留）-> 继续等
  server_->set_status_json(R"({"state":"IDLE"})");
  settle();
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(std::nullopt));
  // 2. 航点名不匹配（上次别的航点的记录）-> 继续等
  server_->set_status_json(
    R"({"set_pose":{"waypoint":"B","sent":true,"verified":true}})");
  settle();
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(std::nullopt));
  // 3. 航点名匹配但验收未出（真机受理后 ~1.5s 窗口）-> 继续等
  server_->set_status_json(
    R"({"set_pose":{"waypoint":"A","sent":true,"verified":null,"note":""}})");
  settle();
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(std::nullopt));
  // 4. 验收通过 -> true
  server_->set_status_json(
    R"({"set_pose":{"waypoint":"A","sent":true,"verified":true,"note":""}})");
  settle();
  const auto ok = executor_nav_->poll_set_pose_verified("A");
  ASSERT_TRUE(ok.has_value());
  EXPECT_TRUE(*ok);
  // 5. 验收失败（注入后位姿仍不匹配）-> false
  server_->set_status_json(
    R"({"set_pose":{"waypoint":"A","sent":true,"verified":false,)"
    R"("note":"注入后仍差 0.42 m。"}})");
  settle();
  const auto bad = executor_nav_->poll_set_pose_verified("A");
  ASSERT_TRUE(bad.has_value());
  EXPECT_FALSE(*bad);
  // 6. 快照非法（空串/坏 JSON）-> 继续等（超时兜底由调用方负责）
  server_->set_status_json("");
  settle();
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(std::nullopt));
  server_->set_status_json("not-json");
  settle();
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"), R(std::nullopt));
}

// 验收防串：受理前的旧帧（上次同航点 verified=true）不得误判为本次结果。
// 真机语义（follower_node._begin_set_pose）：受理时同步重置 set_pose.
// verified=null 并先于服务应答推帧——Mock 的 goto 回调在 action=set_pose
// 时同样重置。本用例预置旧帧后受理，验证「继续等 -> 本次验收结果」
TEST_F(WaypointNavExecutorTest, PollIgnoresStaleFramesBeforeAccept)
{
  // 预置旧帧：上次 A 航点验收通过（latched 语义 = 受理前最后一帧）
  server_->set_status_json(
    R"({"set_pose":{"waypoint":"A","sent":true,"verified":true}})");
  settle();
  // 本次重定位受理：服务端重置 verified=null 并推帧
  server_->set_auto_terminal("");
  ASSERT_TRUE(executor_nav_->set_pose("A").ok);
  settle();
  // 本次记录 verified 未出：继续等（watermark 保证受理前旧帧的
  // verified=true 不越过受理窗口被当作本次结果）
  EXPECT_EQ(executor_nav_->poll_set_pose_verified("A"),
    std::optional<bool>(std::nullopt));
  // 本次验收失败：结果是 false，不是旧帧的 true
  server_->set_status_json(
    R"({"set_pose":{"waypoint":"A","sent":true,"verified":false}})");
  settle();
  const auto bad = executor_nav_->poll_set_pose_verified("A");
  ASSERT_TRUE(bad.has_value());
  EXPECT_FALSE(*bad);
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
