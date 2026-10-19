// ============================================================================
// 真机导航执行器：对接 t1_slam_nav_ws 的导航 action（liwei_navigation）。
//
// 接口面（docs：t1_slam_nav_ws/docs/NavigateAction.md）：
//   - action /liwei_navigation/navigate（liwei_nav_msgs/action/Navigate）：
//     去航点 / 取消 / 充电 / 重定位都是发一个 goal，用 command 字段区分；
//     任务结束时结果直接回给发 goal 的客户端（Result），不再需要订阅
//     status 话题 / 轮询服务。
//       go        本执行器 goto_point：waypoints=[point]（协议 point_name
//                 直接映射航点名）、return_to_quadruped=false（讲解机器
//                 人保持人形）。
//       set_pose  本执行器 set_pose：机器人已被人工摆到该航点上，服务端
//                 发一次初始位姿并验收，**验收完才给 result**（通常零点
//                 几秒，最多约 3s）——result 即验收终态。
//       cancel    guide_pulse 的取消走 action 自带的 cancel_goal（停在
//                 原地、保持人形），不另发 cancel goal。
//   - 服务 /liwei_navigation/query（liwei_nav_msgs/srv/Query）：只读状态
//     快照。仅在 set_pose 被拒（state=REJECTED）时取 available 清单做
//     point_not_found 的结构化判定（点名不在清单 = point_not_found，其余
//     拒因归并 localization_failed；不猜自由文本 message）。
//
// 接口契约对齐（相对旧 GoToWaypoints 服务面的行为差异）：
//   - **被拒的 goal 也会显示「已接受」**，随后马上以 ABORTED + Result
//     state=REJECTED 结束（action 的拒绝应答带不了文字，服务端刻意如此）。
//     因此受理判定不看 goal 应答，统一在 goal 应答后先做 500ms（go）/
//     1s（set_pose）的 result 试探：REJECTED 毫秒级紧随 goal 应答，试探
//     窗口内出现即按「被拒」处理——go 不播受理引导语（notifier(false)）、
//     set_pose 同步返回 ok=false；窗口内无 result = 正常受理。
//   - set_pose 受理后 result（验收结果）不再立即回来：future 挂起由
//     poll_set_pose_verified 消费（调用方 250ms 周期轮询 + 超时兜底，
//     语义与旧 status_json 的 set_pose.verified 完全一致）。
//
// 线程模型（与旧实现同构）：goto_point 的「发 goal -> 等结果」在独立
// 线程串行完成（future 阻塞等待，回执由节点 executor 线程处理，与
// TtsClient 同款模式）；cancel 只置取消标志，由导航线程在 ≤250ms 的
// 分片等待里看到后下发 cancel_goal。set_pose / poll_set_pose_verified
// 由桥接节点 worker 线程串行调用（handle_relocate 与 worker_loop），
// 免锁。current_job_/nav_thread_ 仅 worker 线程（及关停路径，worker 已
// join 后）访问，满足 TourRunner 单写者槽的前提。
// ============================================================================
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <liwei_nav_msgs/action/navigate.hpp>
#include <liwei_nav_msgs/srv/query.hpp>

#include "guide_pulse/nav_executor.hpp"

namespace guide_pulse
{

// Result.state 终态字（与导航栈 Navigate.action 一致）
inline constexpr const char * NAV_STATE_SUCCEEDED = "SUCCEEDED";
inline constexpr const char * NAV_STATE_FAILED = "FAILED";
inline constexpr const char * NAV_STATE_CANCELLED = "CANCELLED";
inline constexpr const char * NAV_STATE_REJECTED = "REJECTED";

class WaypointNavExecutor final : public NavExecutor
{
public:
  // action/服务名按 t1_slam_nav_ws 默认给出；测试注入进程内 Mock 时传自定义名。
  // poll_timeout：go goal 受理后等 result 的兜底超时（对应 nav_timeout_seconds）
  WaypointNavExecutor(
    rclcpp::Node::SharedPtr node,
    std::string action_name = "/liwei_navigation/navigate",
    std::string query_service = "/liwei_navigation/query",
    std::chrono::seconds poll_timeout = std::chrono::seconds(600))
  : node_(std::move(node)),
    action_name_(std::move(action_name)),
    query_service_(std::move(query_service)),
    poll_timeout_(poll_timeout)
  {
    // 独立回调组：等待线程阻塞在 future 上，goal/result 回执由 executor
    // 线程处理，互不干扰（TtsClient 同款）
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    action_client_ = rclcpp_action::create_client<Navigate>(
      node_, action_name_, callback_group_);
    const auto qos = rmw_qos_profile_services_default;
    query_client_ = node_->create_client<Query>(query_service_, qos, callback_group_);
  }

  ~WaypointNavExecutor() override
  {
    // 先作废任务再 join：导航线程在下个 250ms 分片内检查取消标志退出，
    // 保证 action 客户端（随节点销毁）不再被使用
    if (current_job_) {
      current_job_->store(true);
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
    if (current_job_) {
      current_job_->store(true);  // 作废旧任务（新导航替换语义）
    }
    current_job_ = cancelled;  // 仅 worker 线程访问（免锁）
    // join 旧线程（已置取消，最多一个分片内退出），保证任一时刻
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
    if (current_job_) {
      current_job_->store(true);
    }
    // 取消请求由导航线程在分片等待中看到后下发（cancel_goal：停在原地、
    // 保持人形）；旧 goal 以 CANCELED 结束时导航线程静默退出（不回调，
    // 语义由 cancel 定义）。无导航在途 = 无导航线程 = no-op
  }

  // 受理通知安装（TourRunner 每次 start 时在 goto_point 前重装；
  // 回调仅在导航线程的 result 试探点被调）
  void set_accepted_notifier(
    std::function<void(bool, const std::string &)> notifier) override
  {
    accepted_notifier_ = std::move(notifier);
  }

  // 重定位（同步，正常受理 ~1s 内返回）：发 set_pose goal 并等 result
  // 试探 1s——REJECTED（毫秒级紧随 goal 应答）当场返回被拒 + 结构化
  // 拒因分类；1s 内 result（验收完成，正常零点几秒、最多约 3s）直接
  // 带回验收终态（poll_set_pose_verified 首调即回缓存）；仍无 result =
  // 正常受理挂起，验收结果由 poll_set_pose_verified 消费
  //（relocate_timeout_seconds 超时兜底由调用方负责，与旧语义一致）。
  NavOutcome set_pose(const std::string & point_name) override
  {
    NavOutcome outcome;
    outcome.detail = "set_pose " + point_name;
    set_pose_cached_.reset();
    set_pose_pending_.reset();
    auto goal_handle = send_goal(Navigate::Goal::COMMAND_SET_POSE, point_name);
    if (!goal_handle) {
      outcome.detail = goal_error_;  // 服务不可用/受理超时/action 层拒绝
      RCLCPP_WARN(
        node_->get_logger(), "[重定位] %s 请求未成功: %s",
        action_name_.c_str(), outcome.detail.c_str());
      return outcome;
    }
    auto result_future = action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      return finish_set_pose(result_future.get().result, point_name);
    }
    // 正常受理：初值已发，验收结果稍后回来（future 本身就是本次任务的
    // 结果，天然防串，无需旧实现的帧序 watermark）
    set_pose_pending_ = std::move(result_future);
    outcome.ok = true;
    outcome.detail = "重定位初值已发（验收结果约 1.5~3s 后回）: 航点 " + point_name;
    return outcome;
  }

  // 重定位验收查询（调用方周期调用，仅桥接 worker 线程）：
  //   true/false = 验收结果；nullopt = 尚无结果（继续等，超时兜底由调用方负责）
  std::optional<bool> poll_set_pose_verified(const std::string & /*point_name*/) override
  {
    if (set_pose_cached_.has_value()) {
      return set_pose_cached_;  // set_pose 同步段已拿到终态（验收极快）
    }
    if (!set_pose_pending_.has_value()) {
      return std::nullopt;  // 无在途 set_pose：继续等（调用方超时兜底）
    }
    if ((*set_pose_pending_).wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return std::nullopt;  // 验收未出（真机 ~1.5s 窗口）
    }
    const bool verified = finish_set_pose((*set_pose_pending_).get().result, {}).ok;
    set_pose_pending_.reset();
    return verified;
  }

  // 变形后重定位（接口语义见 NavExecutor::set_pose_latest，2026-10-18
  // 现场需求）：发**不带航点**的 set_pose goal——服务端用最新停留点
  //（最近一次导航结束实际停的位置）重设定位初值，把变形转身带偏的
  // AMCL 定位拉回。同步等 result 至终态：REJECTED 毫秒级；正常验收
  // 零点几秒、定位确认上限 10s（NavigateAction.md），兜底 12s。独立
  // future 等待，不占 set_pose/poll_set_pose_verified 的 relocate 轮询槽
  //（写到的 set_pose_cached_ 会被下一次 set_pose() 开头 reset，串行
  // worker 下无交错）
  NavOutcome set_pose_latest() override
  {
    NavOutcome outcome;
    outcome.detail = "set_pose（最新停留点）";
    auto goal_handle = send_goal(Navigate::Goal::COMMAND_SET_POSE, "");
    if (!goal_handle) {
      outcome.detail = goal_error_;  // 服务不可用/受理超时/action 层拒绝
      RCLCPP_WARN(
        node_->get_logger(), "[变形后重定位] 请求未成功: %s", outcome.detail.c_str());
      return outcome;
    }
    auto result_future = action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(12)) != std::future_status::ready) {
      outcome.detail = "set_pose 结果等待超时（12s；验收通常零点几秒，上限 10s）";
      RCLCPP_WARN(node_->get_logger(), "[变形后重定位] %s", outcome.detail.c_str());
      return outcome;
    }
    return finish_set_pose(result_future.get().result, "");
  }

private:
  using Navigate = liwei_nav_msgs::action::Navigate;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
  using ResultFuture = std::shared_future<typename GoalHandle::WrappedResult>;
  using Query = liwei_nav_msgs::srv::Query;

  // 发 goal 并等受理应答（5s 兜底）。成功回 goal_handle；失败回 nullptr
  // 且 goal_error_ 写明原因（仅调用线程使用）
  typename GoalHandle::SharedPtr send_goal(
    const std::string & command, const std::string & point_name)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      goal_error_ = "导航 action 不可用: " + action_name_ +
        "（t1_slam_nav_ws 导航栈未起？）";
      return nullptr;
    }
    Navigate::Goal goal;
    goal.command = command;
    // point_name 为空 = set_pose 留空航点（用最新停留点，NavigateAction.md
    // §3「重定位到最近一次导航结束时停的位置」）；go 恒非空调用，空
    // 列表 go 会被服务端拒——不额外防御
    if (!point_name.empty()) {
      goal.waypoints = {point_name};
    }
    goal.return_to_quadruped = false;  // 讲解机器人保持人形（set_pose 硬约束）
    auto goal_future = action_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      goal_error_ = "导航 goal 受理超时（5s）: " + command + " " + point_name;
      // ponytail: goal 应答迟到时 goal 可能仍被服务端受理（无人等待其
      // result）；仅 DDS 断连才会发生，重启导览/重发即可恢复，不特判
      return nullptr;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      goal_error_ = "导航 goal 被 action 层拒绝: " + command + " " + point_name;
    }
    return goal_handle;
  }

  // set_pose 的 result 终态归类（受理被拒 / 验收成败），返回值 ok 与
  // NavOutcome 语义对齐；point_name 非空时做拒因分类
  NavOutcome finish_set_pose(
    const typename Navigate::Result::SharedPtr result, const std::string & point_name)
  {
    NavOutcome outcome;
    if (result->state == NAV_STATE_REJECTED) {
      outcome.ok = false;
      outcome.detail = "set_pose 被拒: " + result->message;
      if (!point_name.empty()) {
        // 拒因结构化分类：点名不在 query 的 available 清单 =
        // point_not_found；其余（RUNNING/验收窗口/变形中/非人形/定位
        // 未起/查询失败）归并 localization_failed
        outcome.point_not_found = !is_known_waypoint(point_name);
      }
      RCLCPP_WARN(
        node_->get_logger(), "[重定位] 被拒: %s", outcome.detail.c_str());
      return outcome;
    }
    // 验收终态：success = 验收通过（定位已生效）
    outcome.ok = result->success;
    outcome.detail = std::string("set_pose ") +
      (result->success ? "验收通过" : "验收失败") + ": 航点 " +
      (point_name.empty() ? "(本次)" : point_name) + "（" + result->message + "）";
    set_pose_cached_ = result->success;
    return outcome;
  }

  // 查 query 服务的 available 清单（结构化拒因判定）；查询失败回 false
  //（保守：归并 localization_failed，不误报 point_not_found）
  bool is_known_waypoint(const std::string & point_name)
  {
    // wait_for_service 阻塞等 DDS 双向发现（service_is_ready 单次查询在
    // 发现完成前会误判）；由调用线程阻塞，回执由 executor 线程处理
    if (!query_client_->wait_for_service(std::chrono::seconds(1))) {
      return false;
    }
    auto future = query_client_->async_send_request(std::make_shared<Query::Request>());
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      query_client_->remove_pending_request(future);
      RCLCPP_WARN(node_->get_logger(), "[重定位] 拒因分类: query 应答超时");
      return false;
    }
    const auto response = future.get();
    const auto & available = response->available;
    const bool known =
      std::find(available.begin(), available.end(), point_name) != available.end();
    RCLCPP_INFO(
      node_->get_logger(), "[重定位] 拒因分类: 航点「%s」%s available 清单（%zu 个）",
      point_name.c_str(), known ? "在" : "不在", available.size());
    return known;
  }

  // Result -> NavOutcome 终态归类（go 任务）
  NavOutcome classify_result(
    const typename Navigate::Result::SharedPtr result, const std::string & point_name)
  {
    NavOutcome outcome;
    if (result->state == NAV_STATE_REJECTED) {
      outcome.detail = "导航任务被拒: " + result->message;
      return outcome;
    }
    if (result->state == NAV_STATE_SUCCEEDED && result->success) {
      outcome.ok = true;
      outcome.detail = "航点到达: " + point_name;
      return outcome;
    }
    // 其余：FAILED / CANCELLED / SUCCEEDED+success=false（跳过航点或
    // 变形失败——本执行器恒不带变形，成功路径不会出现）
    outcome.detail = "航点任务" + result->state + ": " + result->message;
    if (!result->skipped.empty()) {
      outcome.detail += "（跳过: ";
      for (const auto & w : result->skipped) {
        outcome.detail += w + " ";
      }
      outcome.detail += "）";
    }
    return outcome;
  }

  // 「发 goal -> 等 result」串行流程（独立线程）
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
    RCLCPP_INFO(
      node_->get_logger(), "[导航] 发送请求: %s 航点=%s",
      action_name_.c_str(), point_name.c_str());
    auto goal_handle = send_goal(Navigate::Goal::COMMAND_GO, point_name);
    if (!goal_handle) {
      outcome.detail = goal_error_;
      RCLCPP_WARN(
        node_->get_logger(), "[导航] 请求未发出: %s", outcome.detail.c_str());
      cb(outcome);
      return;
    }
    auto result_future = action_client_->async_get_result(goal_handle);

    // ---- 2. REJECTED 试探（500ms）：被拒的 goal 也会先「已接受」，拒因
    // 走 result（毫秒级）。窗口内 REJECTED -> 不播受理引导语直接失败；
    // 无 result = 正常受理（出发前 posture/nav2_wake 等阶段要十几秒）
    // ----
    auto wait_status = result_future.wait_for(std::chrono::milliseconds(500));
    if (wait_status == std::future_status::ready) {
      outcome = classify_result(result_future.get().result, point_name);
      if (accepted_notifier_) {
        accepted_notifier_(outcome.ok, outcome.detail);
      }
      if (!outcome.ok) {
        RCLCPP_WARN(node_->get_logger(), "[导航] 失败: %s", outcome.detail.c_str());
      }
      cb(outcome);
      return;
    }
    // 受理通知（引导语等）：真机被拒不播（REJECTED 已在试探窗口现形）
    if (accepted_notifier_) {
      accepted_notifier_(true, "导航任务已受理");
    }

    // ---- 3. 等终态 result（分片等待 + cancel 响应 + 超时兜底）----
    const auto deadline = std::chrono::steady_clock::now() + poll_timeout_;
    while (!cancelled->load()) {
      if (result_future.wait_for(std::chrono::milliseconds(250)) ==
        std::future_status::ready)
      {
        outcome = classify_result(result_future.get().result, point_name);
        if (!outcome.ok) {
          RCLCPP_WARN(node_->get_logger(), "[导航] 失败: %s", outcome.detail.c_str());
        } else {
          RCLCPP_INFO(node_->get_logger(), "[导航] 成功: %s", outcome.detail.c_str());
        }
        cb(outcome);
        return;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        outcome.detail = "航点任务等待 result 超时（" +
          std::to_string(poll_timeout_.count()) + "s）";
        RCLCPP_WARN(node_->get_logger(), "[导航] 失败: %s", outcome.detail.c_str());
        cb(outcome);
        return;
      }
    }
    // 本地取消：下发 cancel_goal（停在原地、保持人形），吃完 CANCELED
    // result 后静默退出（不回调，语义由 cancel 定义）；等待上限 2s——
    // 正常取消链路（停稳/变形）数秒内完成，此等待仅为防 client 析构
    // 竞态，超时退出亦安全（shared_future 析构不阻塞）。打印与
    // 「[导航] 发送请求」「[重定位]」同风格（2026-10-18 现场需求：
    // 停止导航发给 t1_slam_nav_ws 的消息要有日志）
    RCLCPP_INFO(
      node_->get_logger(), "[导航] 发送取消请求: %s 航点=%s（cancel_goal，停在原地保持人形）",
      action_name_.c_str(), point_name.c_str());
    action_client_->async_cancel_goal(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      const auto result = result_future.get().result;
      RCLCPP_INFO(
        node_->get_logger(), "[导航] 取消终态: state=%s message=%s",
        result->state.c_str(), result->message.c_str());
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "[导航] 取消终态未在 2s 内回流（本地业务已释放，不影响）");
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  std::string query_service_;
  std::chrono::seconds poll_timeout_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<rclcpp_action::Client<Navigate>> action_client_;
  rclcpp::Client<Query>::SharedPtr query_client_;

  // 受理通知（仅导航线程调用）：TourRunner 在每次 start 时重装
  std::function<void(bool, const std::string &)> accepted_notifier_;

  // set_pose 在途结果（仅桥接 worker 线程访问：set_pose/poll 串行调用，免锁）
  std::optional<ResultFuture> set_pose_pending_;
  std::optional<bool> set_pose_cached_;

  // send_goal 的失败原因（仅调用线程使用，无共享）
  std::string goal_error_;

  // 仅 worker 线程（及关停路径，worker 已 join 后）访问，免锁
  std::shared_ptr<std::atomic_bool> current_job_;
  std::thread nav_thread_;  // 当前导航线程（至多一个；替换/析构时 join）
};

}  // namespace guide_pulse
