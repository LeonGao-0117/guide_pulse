// ============================================================================
// 真机导航执行器：对接 t1_slam_nav_ws 的航点巡航节点（waypoint_follower）。
//
// 服务面（follower_node.py，节点名 waypoint_follower）：
//   - /waypoint_follower/go_to_waypoints（GoToWaypoints）：下发航点任务。
//     accepted 只表示「已开始执行」；waypoints=[point]（协议 point_name
//     直接映射航点名，航点文件即本地点位表）、return_to_quadruped=false
//     （讲解机器人保持人形）、action 缺省 = go。
//   - /waypoint_follower/get_status（GetWaypointStatus，空请求）：轮询状态。
//     终态停在 TERMINAL=(SUCCEEDED, FAILED, CANCELLED) 不回落，轮询判定安全；
//     awaiting_pose 时 RUNNING 但机器人未动（等新鲜位姿），继续等即可。
//   - /waypoint_follower/cancel（std_srvs/Trigger）：取消当前任务，就地停。
//     语义与 GoToWaypoints(["cancel"]) 一致，取有应答的服务面。
//
// 线程模型：goto_point 的「受理+轮询」在独立 detached 线程串行完成
//（服务调用 = async_send_request + future 等待，回执由节点 executor 处理，
// 与 TtsClient 同款模式）；cancel 只作废当前 job 的取消标志并下发 Trigger。
// ============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <waypoint_nav_msgs/srv/get_waypoint_status.hpp>
#include <waypoint_nav_msgs/srv/go_to_waypoints.hpp>

#include "guide_pulse/nav_executor.hpp"

namespace guide_pulse
{

// 状态机终态（与 follower_node.py TERMINAL 一致）
inline constexpr const char * WAYPOINT_STATE_SUCCEEDED = "SUCCEEDED";
inline constexpr const char * WAYPOINT_STATE_FAILED = "FAILED";
inline constexpr const char * WAYPOINT_STATE_CANCELLED = "CANCELLED";

class WaypointNavExecutor final : public NavExecutor
{
public:
  // 服务名按 t1_slam_nav_ws 默认节点名（waypoint_follower）给出；
  // 测试注入进程内 Mock 服务时传自定义名
  WaypointNavExecutor(
    rclcpp::Node::SharedPtr node,
    std::string goto_service = "/waypoint_follower/go_to_waypoints",
    std::string status_service = "/waypoint_follower/get_status",
    std::string cancel_service = "/waypoint_follower/cancel",
    std::chrono::milliseconds poll_interval = std::chrono::milliseconds(1000),
    std::chrono::seconds poll_timeout = std::chrono::seconds(600))
  : node_(std::move(node)),
    goto_service_(std::move(goto_service)),
    status_service_(std::move(status_service)),
    cancel_service_(std::move(cancel_service)),
    poll_interval_(poll_interval),
    poll_timeout_(poll_timeout)
  {
    // 独立回调组：goto_point 的等待线程阻塞在 future 上，
    // 服务回执由 executor 线程处理，互不干扰（TtsClient 同款）
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto qos = rmw_qos_profile_services_default;
    goto_client_ = node_->create_client<GoToWaypoints>(goto_service_, qos, callback_group_);
    status_client_ = node_->create_client<GetWaypointStatus>(status_service_, qos,
        callback_group_);
    cancel_client_ = node_->create_client<Trigger>(cancel_service_, qos, callback_group_);
  }

  ~WaypointNavExecutor() override
  {
    // 先作废任务再 join：轮询线程在下个 poll 周期内检查取消标志退出，
    // 保证服务客户端（随节点销毁）不再被使用
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (current_job_) {
        current_job_->store(true);
      }
    }
    if (nav_thread_.joinable()) {
      nav_thread_.join();
    }
  }

  void goto_point(
    const std::string & point_name, std::function<void(NavOutcome)> on_done) override
  {
    if (!on_done) {
      return;
    }
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (current_job_) {
        current_job_->store(true);  // 作废旧任务（新导航替换语义）
      }
      current_job_ = cancelled;
    }
    // join 旧线程（已置取消，最多一个 poll 周期内退出），保证任一时刻
    // 至多一个导航线程——析构时 join 才能根治线程越界访问客户端
    if (nav_thread_.joinable()) {
      nav_thread_.join();
    }
    nav_thread_ = std::thread(
      [this, cancelled, point_name, cb = std::move(on_done)]() mutable {
        run_navigation(cancelled, point_name, cb);
      });
  }

  void cancel() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (current_job_) {
        current_job_->store(true);
      }
    }
    // 下发 Trigger 取消（就地停）；fire-and-forget，等待由轮询线程的
    // 终态/CANCELLED 观察收敛，不阻塞 worker 线程
    if (cancel_client_->service_is_ready()) {
      cancel_client_->async_send_request(std::make_shared<Trigger::Request>());
    }
  }

private:
  using GoToWaypoints = waypoint_nav_msgs::srv::GoToWaypoints;
  using GetWaypointStatus = waypoint_nav_msgs::srv::GetWaypointStatus;
  using Trigger = std_srvs::srv::Trigger;

  // 「受理 -> 轮询终态」串行流程（detached 线程）
  void run_navigation(
    std::shared_ptr<std::atomic_bool> cancelled, const std::string & point_name,
    std::function<void(NavOutcome)> & cb)
  {
    NavOutcome outcome;
    outcome.detail = "navigate to " + point_name;

    // ---- 1. 下发航点任务 ----
    if (cancelled->load()) {
      return;
    }
    if (!goto_client_->service_is_ready()) {
      outcome.detail = "go_to_waypoints 服务不可用: " + goto_service_;
      cb(outcome);
      return;
    }
    auto request = std::make_shared<GoToWaypoints::Request>();
    request->waypoints = {point_name};
    request->return_to_quadruped = false;  // 讲解机器人保持人形
    request->action.clear();               // 缺省 = go
    auto goto_future = goto_client_->async_send_request(request);
    if (goto_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      goto_client_->remove_pending_request(goto_future);
      outcome.detail = "go_to_waypoints 受理超时";
      cb(outcome);
      return;
    }
    const auto accepted = goto_future.get();
    if (!accepted->accepted) {
      // 被拒原因（状态冲突/航点名错并附近似匹配提示）服务端已写清
      outcome.detail = "航点任务被拒: " + accepted->message;
      if (!accepted->available.empty()) {
        outcome.detail += "（可用航点见服务端日志）";
      }
      cb(outcome);
      return;
    }

    // ---- 2. 轮询状态直到终态（终态停住不回落，判定安全）----
    const auto deadline = std::chrono::steady_clock::now() + poll_timeout_;
    while (!cancelled->load()) {
      std::this_thread::sleep_for(poll_interval_);
      if (cancelled->load()) {
        return;  // 已取消：不回调（语义由 cancel 定义）
      }
      if (!status_client_->service_is_ready()) {
        continue;  // get_status 短暂不可用：继续轮询（导航本身不受影响）
      }
      auto status = std::make_shared<GetWaypointStatus::Request>();
      auto future = status_client_->async_send_request(status);
      if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        status_client_->remove_pending_request(future);
        continue;
      }
      const auto response = future.get();
      const auto & state = response->state;
      if (state == WAYPOINT_STATE_SUCCEEDED) {
        outcome.ok = true;
        outcome.detail = "航点到达: " + point_name;
        cb(outcome);
        return;
      }
      if (state == WAYPOINT_STATE_FAILED || state == WAYPOINT_STATE_CANCELLED) {
        outcome.detail = "航点任务" + state + ": " + response->reason;
        cb(outcome);
        return;
      }
      // IDLE/RUNNING/PAUSED（含 awaiting_pose 等位姿）：继续轮询
      if (std::chrono::steady_clock::now() >= deadline) {
        outcome.detail = "航点任务轮询超时（" +
          std::to_string(poll_timeout_.count()) + "s）最后状态: " + state;
        cb(outcome);
        return;
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string goto_service_;
  std::string status_service_;
  std::string cancel_service_;
  std::chrono::milliseconds poll_interval_;
  std::chrono::seconds poll_timeout_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<GoToWaypoints>::SharedPtr goto_client_;
  rclcpp::Client<GetWaypointStatus>::SharedPtr status_client_;
  rclcpp::Client<Trigger>::SharedPtr cancel_client_;

  std::mutex mutex_;
  std::shared_ptr<std::atomic_bool> current_job_;
  std::thread nav_thread_;  // 当前导航线程（至多一个；替换/析构时 join）
};

}  // namespace guide_pulse
