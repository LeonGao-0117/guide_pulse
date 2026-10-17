// ============================================================================
// 讲解模块状态机（navigate 指令，协议 v0.16.0 + v0.20.0 可选 texts）：
// 导航 -> 到达 -> 桥接节点开一轮讲解 PlaylistSession。
//
// 线程模型（无锁）：业务状态（seq_/active_/point_name_）仅 worker 线程
// 访问；NavExecutor 的 on_done 回调线程只做一件事——把结果原子发布到
// 单槽（shared_ptr 原子交换），不碰任何业务状态。单写者成立的前提由
// NavExecutor 实现保证：启动新任务前旧任务不再触发回调（Waypoint/Mock
// 均 join 旧线程后启动；测试 Fake 为同步回调）。worker 消费时按 seq
// 丢弃过期回调，语义与旧版「锁内判 seq」完全一致。
// 纯 C++（仅 STL），独立单测（test_tour.cpp 注入 FakeNavExecutor）。
// ============================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "guide_pulse/nav_executor.hpp"

namespace guide_pulse
{

class TourRunner
{
public:
  // nav: 导航执行器（调用方持有，生命周期覆盖 TourRunner）；
  // point 不存在等失败由 NavOutcome.detail 表达
  explicit TourRunner(NavExecutor & nav)
  : nav_(nav) {}

  // 发起新导航。texts 为到达后的讲解词（可为空 = 到达后不播报）。
  // 仅 worker 线程调用（无并发，成员免锁）
  void start(const std::string & point_name, std::vector<std::string> texts)
  {
    const auto seq = ++seq_;
    active_ = true;  // 在途；到达落槽后仍算活动（待 worker 消费）
    point_name_ = point_name;
    // 旧槽作废：即使旧回调已发布，消费时也会因 seq 失配被丢弃
    std::atomic_store(&arrived_, std::shared_ptr<const ArrivalSlot>());
    // 受理通知重装（捕获当次 seq；窗口说明见 NavExecutor 基类注释）
    nav_.set_accepted_notifier(
      [this, seq](bool accepted, const std::string & message) {
        std::atomic_store(
          &accepted_,
          std::make_shared<const AcceptedSlot>(
            AcceptedSlot{seq, accepted, message}));
      });
    // 回调线程只原子发布结果单槽；seq 判废在消费侧（poll_arrival）做
    nav_.goto_point(
      point_name, [this, seq, texts = std::move(texts)](NavOutcome outcome) mutable {
        std::atomic_store(
          &arrived_,
          std::make_shared<const ArrivalSlot>(
            ArrivalSlot{seq, outcome.ok, std::move(texts), outcome.detail}));
      });
  }

  // 取消当前导航并清空到达槽（tts.stop / 程序停止时调用）
  void cancel()
  {
    ++seq_;  // 使在途回调失效（消费侧 seq 失配丢弃）
    active_ = false;
    std::atomic_store(&arrived_, std::shared_ptr<const ArrivalSlot>());
    nav_.cancel();
  }

  // worker 线程消费到达事件（取走即清除）：ok=true 时 texts 交由桥接节点
  // 开讲解播报；ok=false 仅日志（协议无导航回执面）
  struct Arrival
  {
    bool ok{false};
    std::vector<std::string> texts;
    std::string detail;
  };
  std::optional<Arrival> poll_arrival()
  {
    auto slot = std::atomic_exchange(&arrived_, std::shared_ptr<const ArrivalSlot>());
    if (!slot || slot->seq != seq_) {
      // 无事件，或过期回调（新导航仍在途）：active_ 不动，继续等新结果
      return std::nullopt;
    }
    active_ = false;  // 到达事件已消费：活动结束
    Arrival event;
    event.ok = slot->ok;
    event.texts = std::move(slot->texts);
    event.detail = std::move(slot->detail);
    return event;
  }

  // 是否有活动（在途导航或待消费的到达事件）：worker 据此钳制等待上限。
  // 仅 worker 线程访问（成员免锁）
  bool navigating() const
  {
    return active_;
  }

  // worker 线程消费受理事件（取走即清除）：返回 (accepted, 服务端
  // message)；无事件或已过期（cancel/被新任务替换，seq 失配）返回
  // nullopt。到达前的受理先于到达事件到达此槽，两槽独立
  std::optional<std::pair<bool, std::string>> poll_accepted()
  {
    auto slot = std::atomic_exchange(
      &accepted_, std::shared_ptr<const AcceptedSlot>());
    if (!slot || slot->seq != seq_) {
      return std::nullopt;
    }
    return std::make_pair(slot->accepted, std::move(slot->message));
  }

  // 仅供日志展示（仅 worker 线程访问）
  const std::string & current_point() const
  {
    return point_name_;
  }

private:
  // 到达结果单槽：导航回调线程一次原子发布，worker 原子取走（SPSC）
  struct ArrivalSlot
  {
    std::uint64_t seq;       // 发布该结果的导航代次（消费侧判废）
    bool ok;
    std::vector<std::string> texts;
    std::string detail;
  };

  // 受理结果单槽（SPSC 同上）：go_to_waypoints 应答到达时发布
  struct AcceptedSlot
  {
    std::uint64_t seq;
    bool accepted;
    std::string message;
  };

  NavExecutor & nav_;
  // ---- 以下成员仅 worker 线程访问（业务串行，免锁）----
  std::uint64_t seq_{0};
  bool active_{false};                 // 在途导航或到达待消费（worker 轮询依据）
  std::string point_name_;
  // ---- 导航回调线程写、worker 读（原子单槽，无锁）----
  std::shared_ptr<const ArrivalSlot> arrived_;
  std::shared_ptr<const AcceptedSlot> accepted_;
};

}  // namespace guide_pulse
