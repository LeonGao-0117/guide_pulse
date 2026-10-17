// ============================================================================
// 真机导航执行器：对接 t1_slam_nav_ws 的航点巡航节点（waypoint_follower）。
//
// 服务面（follower_node.py，节点名 waypoint_follower）：
//   - /waypoint_follower/go_to_waypoints（GoToWaypoints）：下发航点任务。
//     accepted 只表示「已开始执行」；waypoints=[point]（协议 point_name
//     直接映射航点名，航点文件即本地点位表）、return_to_quadruped=false
//     （讲解机器人保持人形）、action 缺省 = go。
//     action=set_pose 时为重定位：机器人已被人工
//     摆到该航点上，服务端只发一次 /initialpose 重设定位、机器人不动。
//     受理不是终态（2026-10-17）：生效与否约 1.5s 后由服务端验收并写
//     ~/status 的 set_pose.verified，本执行器订阅 ~/status 话题推送读取
//     （poll_set_pose_verified），终态以验收结果为准。
//   - /waypoint_follower/status（std_msgs/String，JSON 快照，latched）：
//     状态通道。服务端每次状态变化主动推一帧（与 get_status 服务读的是
//     同一个快照），QoS = RELIABLE + TRANSIENT_LOCAL + KeepLast(1)——
//     订阅必须同配 transient_local，否则 QoS 对不上**不会报错**、只会
//     安静地收不到 latched 帧（服务端代码原话警告）。本执行器全部状态
//     等待（navigate 终态 / relocate 验收）都走这条推送流，不再轮询
//     get_status 服务：栈没起 = 收不到帧，由 poll_timeout 兜底，等价。
//   - /waypoint_follower/cancel（std_srvs/Trigger）：取消当前任务，就地停。
//     语义与 GoToWaypoints(["cancel"]) 一致，取有应答的服务面。
//
// 帧序防串（watermark）：下发服务请求**前**记录当前已收帧数，等待期间
// 只认此后的新帧。为什么记在 send 前而不是受理应答后：服务端在服务
// 应答之前就已 publish 本次任务的首帧（goto=RUNNING / set_pose 的
// verified=null），应答后记 watermark 会把这帧滤掉；send 前记则受理
// 回调里的帧无论与应答谁先到达都有效。旧帧（上次任务终态、订阅建立时
// latched 补发的最后一帧）不会误判：订阅在构造时建立，请求下发时缓存
// 已稳定，之后到达的帧必属本次任务（navigate/relocate 业务互斥，一次
// 只有一个等待者，共用一个 watermark）。
//
// 线程模型：goto_point 的「受理+等推送」在独立线程串行完成
//（服务调用 = async_send_request + future 等待，回执由节点 executor 处理，
// 与 TtsClient 同款模式）；cancel 只作废当前 job 的取消标志并下发 Trigger。
// 订阅回调仅持锁拷贝快照字符串 + 帧计数，JSON 解析留在 worker 线程
//（poll_set_pose_verified / run_navigation），回调不做重活。
// current_job_/nav_thread_ 仅 worker 线程（及关停路径，worker 已 join 后）
// 访问，免锁；goto_point join 旧线程保证任一时刻至多一个导航线程，
// 满足 TourRunner 单写者槽的前提。
// ============================================================================
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
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
  // status 是 ~/status 话题（JSON 快照推送），不是服务；
  // 测试注入进程内 Mock 服务时传自定义名
  WaypointNavExecutor(
    rclcpp::Node::SharedPtr node,
    std::string goto_service = "/waypoint_follower/go_to_waypoints",
    std::string status_topic = "/waypoint_follower/status",
    std::string cancel_service = "/waypoint_follower/cancel",
    std::chrono::milliseconds poll_interval = std::chrono::milliseconds(1000),
    std::chrono::seconds poll_timeout = std::chrono::seconds(600))
  : node_(std::move(node)),
    goto_service_(std::move(goto_service)),
    status_topic_(std::move(status_topic)),
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
    cancel_client_ = node_->create_client<Trigger>(cancel_service_, qos, callback_group_);
    // ~/status 订阅：QoS 必须与服务端 _LATCHED 逐项对齐
    //（RELIABLE + TRANSIENT_LOCAL + KeepLast(1)），durability 低配会安静
    // 丢失 latched 帧。回调组用默认组：回调只拷贝字符串，不与任何
    // 等待线程争用
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::String::ConstSharedPtr & msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_json_ = msg->data;
        ++status_frames_;
        // status 帧不打印（约 5s 一帧心跳 + 状态变化帧，量大）：终态判定
        // 的结果在 run_navigation 出口统一打一条
      });
  }

  ~WaypointNavExecutor() override
  {
    // 先作废任务再 join：轮询线程在下个 poll 周期内检查取消标志退出，
    // 保证服务客户端/订阅（随节点销毁）不再被使用
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
    if (current_job_) {
      current_job_->store(true);
    }
    // 下发 Trigger 取消（就地停）；fire-and-forget，等待由轮询线程的
    // 终态/CANCELLED 观察收敛，不阻塞 worker 线程
    if (cancel_client_->service_is_ready()) {
      RCLCPP_INFO(
        node_->get_logger(), "[导航] 发送取消请求: %s", cancel_service_.c_str());
      cancel_client_->async_send_request(std::make_shared<Trigger::Request>());
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "[导航] 取消请求未发出（服务不可用）: %s",
        cancel_service_.c_str());
    }
  }

  // 受理通知安装（TourRunner 每次 start 时在 goto_point 前重装；
  // 回调仅在导航线程的应答点被调）
  void set_accepted_notifier(
    std::function<void(bool, const std::string &)> notifier) override
  {
    accepted_notifier_ = std::move(notifier);
  }

  // 重定位（受理，毫秒级）：go_to_waypoints(action=set_pose)，真机服务端
  // 语义（follower_node._begin_set_pose）：只在无任务在跑且已是站立人形时
  // 受理——导航在途/狗形/定位未起均被拒且 message 写明原因；accepted=true
  // 表示初值已发给定位并**同步写入 status 的 set_pose 记录（verified=null）**，
  // 生效与否约 1.5s 后由服务端验收（poll_set_pose_verified 读 ~/status 推送）。
  // 与 trigger_speech_motion 同款同步短调用（受理超时兜底 5s）
  NavOutcome set_pose(const std::string & point_name) override
  {
    NavOutcome outcome;
    outcome.detail = "set_pose " + point_name;
    if (!goto_client_->service_is_ready()) {
      outcome.detail = "go_to_waypoints 服务不可用: " + goto_service_ +
        "（t1_slam_nav_ws 航点巡航未起？）";
      RCLCPP_WARN(
        node_->get_logger(), "[重定位] %s 请求未发出（服务不可用）: %s",
        goto_service_.c_str(), outcome.detail.c_str());
      return outcome;
    }
    auto request = std::make_shared<GoToWaypoints::Request>();
    request->waypoints = {point_name};  // set_pose 只认单航点
    request->return_to_quadruped = false;  // 重定位要求保持人形（服务端硬约束）
    request->action = "set_pose";
    mark_wait_start();  // 帧序 watermark：send 前记（见类注释）
    auto future = goto_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      goto_client_->remove_pending_request(future);
      outcome.detail = "set_pose 受理超时";
      RCLCPP_WARN(
        node_->get_logger(), "[重定位] %s 应答超时（5s）: 航点 %s",
        goto_service_.c_str(), point_name.c_str());
      return outcome;
    }
    const auto accepted = future.get();
    outcome.ok = accepted->accepted;
    if (accepted->accepted) {
      outcome.detail = "重定位初值已发（生效与否约 1.5s 后见 ~/status 推送）: 航点 " +
        point_name + "（" + accepted->message + "）";
    } else {
      // 拒因结构化分类（唯一可靠信号）：available 恒回带全部合法航点名
      //（服务端语义：名字打错的调用方无需再查一次状态），点名不在其中
      // = point_not_found；其余拒因（RUNNING/验收窗口/变形中/非人形/
      // 定位未起）的自由文本 message 不猜，归并 localization_failed
      outcome.point_not_found = std::find(
        accepted->available.begin(), accepted->available.end(),
        point_name) == accepted->available.end();
      outcome.detail = "set_pose 被拒: " + accepted->message;
    }
    return outcome;
  }

  // 重定位验收查询（调用方周期调用，worker 线程）：读 ~/status 推送流的
  // 最新快照（受理后新帧），解析 set_pose = {waypoint, verified(null|bool)}：
  //   waypoint 匹配且 verified 非 null -> 验收结果（true/false）
  //   其余（尚无新帧 / 无 set_pose / 航点名不符 / verified 仍 null /
  //   JSON 非法）-> nullopt 继续等（超时兜底由调用方负责）
  // 新旧记录无歧义：受理成功时服务端同步覆盖 set_pose 记录（verified
  // 重置为 null 并先于服务应答 publish），watermark 之后的新帧必是本次
  // 记录（follower_node._begin_set_pose 在应答前写 _set_pose）。
  std::optional<bool> poll_set_pose_verified(const std::string & point_name) override
  {
    const auto snapshot = take_fresh_snapshot();
    if (!snapshot.has_value()) {
      return std::nullopt;  // 受理后还没收到新帧：继续等（超时兜底）
    }
    try {
      const auto status = nlohmann::json::parse(*snapshot);
      if (!status.contains("set_pose") || !status.at("set_pose").is_object()) {
        return std::nullopt;
      }
      const auto & record = status.at("set_pose");
      if (!record.contains("waypoint") || !record.at("waypoint").is_string() ||
        record.at("waypoint").get<std::string>() != point_name)
      {
        return std::nullopt;  // 不是本次的记录（受理前旧值等）
      }
      if (!record.contains("verified") || record.at("verified").is_null()) {
        return std::nullopt;  // 初值已发、验收未出（真机 ~1.5s 窗口）
      }
      return record.at("verified").get<bool>();
    } catch (const nlohmann::json::exception &) {
      return std::nullopt;  // 快照非法/缺字段：继续等（超时兜底）
    }
  }

private:
  using GoToWaypoints = waypoint_nav_msgs::srv::GoToWaypoints;
  using Trigger = std_srvs::srv::Trigger;

  // ---- ~/status 推送流缓存（订阅回调线程写，worker 线程读）----
  // 下发服务请求前调用（worker 线程）：把「当前已收帧数」记为等待起点，
  // 之后 take_fresh_snapshot 只回放这之后到达的新帧
  void mark_wait_start()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    wait_watermark_ = status_frames_;
  }

  // 取等待起点之后的新帧（worker 线程）；旧帧/nullopt = 尚无新帧
  std::optional<std::string> take_fresh_snapshot()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_frames_ <= wait_watermark_) {
      return std::nullopt;
    }
    return status_json_;
  }

  // 「受理 -> 等推送终态」串行流程（detached 线程）
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
      RCLCPP_WARN(
        node_->get_logger(), "[导航] 请求未发出（服务不可用）: %s",
        outcome.detail.c_str());
      cb(outcome);
      return;
    }
    auto request = std::make_shared<GoToWaypoints::Request>();
    request->waypoints = {point_name};
    request->return_to_quadruped = false;  // 讲解机器人保持人形
    request->action.clear();               // 缺省 = go
    RCLCPP_INFO(
      node_->get_logger(), "[导航] 发送请求: %s 航点=%s",
      goto_service_.c_str(), point_name.c_str());
    mark_wait_start();  // 帧序 watermark：send 前记（见类注释）
    auto goto_future = goto_client_->async_send_request(request);
    if (goto_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      goto_client_->remove_pending_request(goto_future);
      outcome.detail = "go_to_waypoints 受理超时";
      RCLCPP_WARN(
        node_->get_logger(), "[导航] 应答超时（5s）: 航点 %s",
        point_name.c_str());
      cb(outcome);
      return;
    }
    const auto accepted = goto_future.get();
    RCLCPP_INFO(
      node_->get_logger(), "[导航] 收到应答: accepted=%s, message=%s",
      accepted->accepted ? "true" : "false", accepted->message.c_str());
    // 受理通知（引导语等）：收到应答即回调一次；超时/不可用路径不回调
    if (accepted_notifier_) {
      accepted_notifier_(accepted->accepted, accepted->message);
    }
    if (!accepted->accepted) {
      // 被拒原因（状态冲突/航点名错并附近似匹配提示）服务端已写清
      outcome.detail = "航点任务被拒: " + accepted->message;
      if (!accepted->available.empty()) {
        outcome.detail += "（可用航点见服务端日志）";
      }
      cb(outcome);
      return;
    }

    // ---- 2. 等推送终态（终态停住不回落，判定安全）----
    const auto deadline = std::chrono::steady_clock::now() + poll_timeout_;
    while (!cancelled->load()) {
      std::this_thread::sleep_for(poll_interval_);
      if (cancelled->load()) {
        return;  // 已取消：不回调（语义由 cancel 定义）
      }
      const auto snapshot = take_fresh_snapshot();
      if (snapshot.has_value()) {
        try {
          const auto status = nlohmann::json::parse(*snapshot);
          const auto state = status.value<std::string>("state", "");
          if (state == WAYPOINT_STATE_SUCCEEDED) {
            outcome.ok = true;
            outcome.detail = "航点到达: " + point_name;
            RCLCPP_INFO(
              node_->get_logger(), "[导航] 成功: %s", outcome.detail.c_str());
            cb(outcome);
            return;
          }
          if (state == WAYPOINT_STATE_FAILED || state == WAYPOINT_STATE_CANCELLED) {
            outcome.detail = "航点任务" + state + ": " +
              status.value<std::string>("reason", "");
            RCLCPP_WARN(
              node_->get_logger(), "[导航] 失败: %s", outcome.detail.c_str());
            cb(outcome);
            return;
          }
          // IDLE/RUNNING/PAUSED（含 awaiting_pose 等位姿）：继续等
        } catch (const nlohmann::json::exception &) {
          // 坏帧当无帧处理：继续等（超时兜底）
        }
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        outcome.detail = "航点任务等待推送超时（" +
          std::to_string(poll_timeout_.count()) + "s）最后帧: " +
          snapshot.value_or("(无)");
        RCLCPP_WARN(
          node_->get_logger(), "[导航] 失败: %s", outcome.detail.c_str());
        cb(outcome);
        return;
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string goto_service_;
  std::string status_topic_;
  std::string cancel_service_;
  std::chrono::milliseconds poll_interval_;
  std::chrono::seconds poll_timeout_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<GoToWaypoints>::SharedPtr goto_client_;
  rclcpp::Client<Trigger>::SharedPtr cancel_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

  // 受理通知（仅导航线程调用）：TourRunner 在每次 start 时重装
  //（goto_point 前旧线程仍在跑的理论窗口见基类注释）
  std::function<void(bool, const std::string &)> accepted_notifier_;

  // ~/status 推送流缓存：订阅回调线程写（持锁），worker 线程读（持锁）。
  // status_frames_ 单调递增 = 已收帧数；wait_watermark_ = 等待起点
  //（mark_wait_start 记录，take_fresh_snapshot 只认新帧，防旧帧串场）
  std::mutex status_mutex_;
  std::string status_json_;
  uint64_t status_frames_{0};
  uint64_t wait_watermark_{0};

  // 仅 worker 线程（及关停路径，worker 已 join 后）访问，免锁
  std::shared_ptr<std::atomic_bool> current_job_;
  std::thread nav_thread_;  // 当前导航线程（至多一个；替换/析构时 join）
};

}  // namespace guide_pulse
