// ============================================================================
// 导航执行器：point_name -> 本地导航执行的唯一挂载点。
//
// 背景与边界（docs/design.md §4）：SDK sdk_t1-v0.9.4.4 没有点位导航服务
//（pnc 是运控内置输入源，不对外暴露 goto-point 接口）；RobotCar 协议把
//「point_name -> 物理位置映射与导航执行」划给 Robot 本地。本工程以窄接口
// 隔离该硬件边界，默认实现 MockNavExecutor（联调/测试）。
//
// // ponytail: 真机导航栈可用后新增一个 NavExecutor 子类并在
// // nav_executor 参数注册选择；TourRunner 与桥接节点无需改动。
// ============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace guide_pulse
{

struct NavOutcome
{
  bool ok{false};
  std::string detail;  // 失败原因（日志用）
  // set_pose 被拒且拒因是「航点名不在航点表」的结构化标记（2026-10-17
  // MQTT relocate 链路）：由服务端回带的 available 清单判定（结构化字段，
  // 不猜自由文本 message），桥接层据此映射 relocate.result 的
  // point_not_found；其余拒因（导航在途/非人形/定位未起等）归并
  // localization_failed。goto 导航链路不置位（失败统一 navigation_failed）
  bool point_not_found{false};
};

class NavExecutor
{
public:
  virtual ~NavExecutor() = default;

  // 异步导航到点位。on_done 在任意线程回调一次（成功或失败）；
  // cancel() 之后不得再回调。点位名 -> 位置/坐标的映射归实现持有。
  virtual void goto_point(
    const std::string & point_name, std::function<void(NavOutcome)> on_done) = 0;

  // 取消当前导航（无导航时 no-op）；语义：取消后 on_done 不触发。
  virtual void cancel() = 0;

  // 受理通知（可选支持，2026-10-17 导览引导语需求）：导航请求收到
  // go_to_waypoints 服务应答时回调一次（accepted + 服务端 message；
  // 无应答/受理超时/服务不可用不回调）。回调在实现内部的导航线程，
  // 调用方须自行保证线程安全（如仅原子写单槽）。
  // ⚠️ 安装时机窗口：TourRunner 在 goto_point（内部 join 旧导航线程）
  // 之前重装——若旧任务恰在「已发请求、等应答」的 ≤5s 窗口内被取消
  // 并立刻下发新任务，旧应答可能写进新任务的槽；后果仅引导语播报
  // 时机异常（到达/终态槽独立不受影响），业务互斥下正常路径无此竞态
  virtual void set_accepted_notifier(
    std::function<void(bool, const std::string &)> /*notifier*/)
  {
  }

  // 重定位：机器人已被人工摆到 point_name 对应航点上（位置与朝向对准），
  // 把定位初值重设到该航点——**机器人不动**，只改定位认知。同步受理（真机
  // waypoint_follower 只发一次 /initialpose，毫秒级返回；受理超时兜底 5s）。
  // 受理成功**不是终态**：生效与否由服务端异步验收（真机约 1.5s 后写
  // ~/status 的 set_pose.verified），调用方受理成功后应周期调
  // poll_set_pose_verified(point) 直到拿到验收结果（2026-10-17 需求：
  // 重定位业务以导航模块的验收结果为终态，超时由调用方兜底）。
  // 默认实现：当前执行器不支持重定位
  virtual NavOutcome set_pose(const std::string & point_name)
  {
    NavOutcome outcome;
    outcome.detail = "导航执行器不支持重定位: " + point_name;
    return outcome;
  }

  // set_pose 受理成功后轮询验收结果（调用方周期调用，建议 250ms）：
  //   true     = 验收通过（定位已生效）
  //   false    = 验收失败（注入后位姿仍不匹配，如特征匹配被拒）
  //   nullopt  = 尚无结果（继续轮询；超时兜底由调用方负责）
  // 默认实现返回 true（受理即验收通过：Mock 及无验收面的执行器）
  virtual std::optional<bool> poll_set_pose_verified(const std::string & /*point_name*/)
  {
    return true;
  }
};

// ---------------------------------------------------------------------------
// Mock 导航执行器：mock_nav_seconds 秒后回调到达（0=立即，但仍异步）。
// 联调与集成测试用；cancel 只作废当前这一次 goto（每次 goto 独立取消
// 标志，旧导航线程不受新导航 goto 重置影响）。
// 线程模型（无锁，仅 worker 线程调用 goto/cancel/析构）：goto 先作废
// 并 join 旧线程再启动新线程——任一时刻至多一个模拟线程存活，满足
// TourRunner 单写者槽的前提。模拟行走分片睡眠（250ms 检查取消），
// 被作废后 join 最多等一个分片。
// ---------------------------------------------------------------------------
class MockNavExecutor final : public NavExecutor
{
public:
  // mock_relocate_hold_polls：重定位受理后 poll_set_pose_verified 前 N 次
  // 返回 nullopt（模拟真机 ~1.5s 验收等待窗口，集成测试用它打开「重定位
  // 进行中」的拒绝窗口）；0 = 受理即验收通过
  explicit MockNavExecutor(
    int mock_nav_seconds = 0, int mock_relocate_hold_polls = 0)
  : mock_nav_seconds_(mock_nav_seconds),
    relocate_hold_polls_(mock_relocate_hold_polls) {}

  ~MockNavExecutor() override
  {
    if (current_) {
      current_->store(true);
    }
    if (walker_.joinable()) {
      walker_.join();
    }
  }

  void goto_point(
    const std::string & point_name, std::function<void(NavOutcome)> on_done) override
  {
    if (!on_done) {
      return;
    }
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    if (current_) {
      current_->store(true);  // 作废旧任务
    }
    current_ = cancelled;  // 仅 worker 线程访问（免锁）
    if (walker_.joinable()) {
      walker_.join();  // 旧线程已在分片睡眠中看到取消，至多 250ms 退出
    }
    walker_ = std::thread(
      [this, cancelled, point_name, cb = std::move(on_done)]() {
        if (mock_notifier_) {
          mock_notifier_(true, "Mock 导航受理");  // 即发即受理
        }
        // 分片模拟行走：被作废后 250ms 内退出（保证上层 join 不久等）
        for (int slept = 0; slept < mock_nav_seconds_ * 4 && !cancelled->load();
          ++slept)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        if (cancelled->load()) {
          return;  // 已取消：不回调（语义由 cancel 定义）
        }
        NavOutcome outcome;
        outcome.ok = true;
        outcome.detail = "Mock导航到达: " + point_name;
        cb(outcome);
      });
  }

  void cancel() override
  {
    if (current_) {
      current_->store(true);
    }
  }

  // Mock 受理通知：goto 即受理（联调/集成测试与真机行为对齐）
  void set_accepted_notifier(
    std::function<void(bool, const std::string &)> notifier) override
  {
    mock_notifier_ = std::move(notifier);
  }

  // Mock 重定位：受理即成功（联调/集成测试；真机语义见 NavExecutor 基类）。
  // 每次受理重置验收等待窗口（hold 计数），保证连续多次重定位都有完整窗口
  NavOutcome set_pose(const std::string & point_name) override
  {
    relocate_hold_left_ = relocate_hold_polls_;
    NavOutcome outcome;
    outcome.ok = true;
    outcome.detail = "Mock重定位受理（初值已发）: " + point_name;
    return outcome;
  }

  // Mock 验收轮询：本次受理的 hold 计数耗尽前返回 nullopt（模拟验收
  // 等待），之后 true。仅 worker 线程调用，免锁
  std::optional<bool> poll_set_pose_verified(const std::string & /*point_name*/) override
  {
    if (relocate_hold_left_ > 0) {
      --relocate_hold_left_;
      return std::nullopt;
    }
    return true;
  }

private:
  int mock_nav_seconds_;
  int relocate_hold_polls_;   // 配置的等待窗口（每次受理重新生效）
  int relocate_hold_left_{0}; // 本次受理剩余的等待 poll 次数
  std::function<void(bool, const std::string &)> mock_notifier_;  // 仅 walker 线程调用
  std::shared_ptr<std::atomic_bool> current_;  // 仅 worker 线程访问（免锁）
  std::thread walker_;  // 当前模拟线程（至多一个；替换/析构时 join）
};

}  // namespace guide_pulse
