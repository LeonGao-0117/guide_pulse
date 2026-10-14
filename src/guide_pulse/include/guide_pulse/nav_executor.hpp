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
#include <mutex>
#include <string>
#include <thread>

namespace guide_pulse
{

struct NavOutcome
{
  bool ok{false};
  std::string detail;  // 失败原因（日志用）
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
};

// ---------------------------------------------------------------------------
// Mock 导航执行器：mock_nav_seconds 秒后回调到达（0=立即，但仍异步）。
// 联调与集成测试用；cancel 只作废当前这一次 goto（每次 goto 独立取消
// 标志，旧导航线程不受新导航 goto 重置影响）。
// ---------------------------------------------------------------------------
class MockNavExecutor final : public NavExecutor
{
public:
  explicit MockNavExecutor(int mock_nav_seconds = 0)
  : mock_nav_seconds_(mock_nav_seconds) {}

  void goto_point(
    const std::string & point_name, std::function<void(NavOutcome)> on_done) override
  {
    if (!on_done) {
      return;
    }
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_ = cancelled;  // cancel() 只作废最新一次 goto
    }
    // 独立线程模拟行走；真机替换为实现内部的导航调用
    std::thread(
      [this, cancelled, point_name, cb = std::move(on_done)]() {
        std::this_thread::sleep_for(std::chrono::seconds(mock_nav_seconds_));
        if (cancelled->load()) {
          return;  // 已取消：不回调（语义由 cancel 定义）
        }
        NavOutcome outcome;
        outcome.ok = true;
        outcome.detail = "Mock导航到达: " + point_name;
        cb(outcome);
      })
    .detach();
  }

  void cancel() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_) {
      current_->store(true);
    }
  }

private:
  int mock_nav_seconds_;
  std::mutex mutex_;
  std::shared_ptr<std::atomic_bool> current_;
};

}  // namespace guide_pulse
