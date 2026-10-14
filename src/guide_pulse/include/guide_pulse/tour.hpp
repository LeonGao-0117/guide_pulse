// ============================================================================
// 讲解模块状态机（navigate 指令，协议 v0.16.0 + v0.20.0 可选 texts）：
// 导航 -> 到达 -> 桥接节点开一轮讲解 PlaylistSession。
//
// 线程模型：NavExecutor 的 on_done 回调可能发生在任意线程（Mock 为独立
// 线程，真机为导航栈回调线程）。TourRunner 把异步到达规约为「单槽事件」：
// 回调只落槽，worker 线程经 poll_arrival() 消费（与 tts_demo condition
// 待播 queue 同款模式）。导航代次 seq 防止被取消的旧回调污染新导航。
// 纯 C++（仅 STL），独立单测（test_tour.cpp 注入 FakeNavExecutor）。
// ============================================================================
#pragma once

#include <cstdint>
#include <mutex>
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

  // 发起新导航（替换进行中的导航：先作废旧导航回调）。texts 为到达后的
  // 讲解词（可为空 = 到达后不播报）。协议允许本地决定替换策略。
  void start(const std::string & point_name, std::vector<std::string> texts)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++seq_;
    arrived_.reset();
    active_ = true;  // 在途；到达落槽后仍算活动（待 worker 消费）
    point_name_ = point_name;
    const auto seq = seq_;
    nav_.goto_point(
      point_name, [this, seq, texts = std::move(texts)](NavOutcome outcome) mutable {
        std::lock_guard<std::mutex> cb_lock(mutex_);
        if (seq != seq_) {
          return;  // 已被新导航/取消取代：丢弃旧回调
        }
        arrived_ = Arrival{outcome.ok, std::move(texts), outcome.detail};
      });
  }

  // 取消当前导航并清空到达槽（tts.stop / 程序停止时调用）
  void cancel()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++seq_;  // 使在途回调失效
    arrived_.reset();
    active_ = false;
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
    std::lock_guard<std::mutex> lock(mutex_);
    if (!arrived_.has_value()) {
      return std::nullopt;
    }
    auto event = std::move(*arrived_);
    arrived_.reset();
    active_ = false;  // 到达事件已消费：活动结束
    return event;
  }

  // 是否有活动（在途导航或待消费的到达事件）：worker 据此钳制等待上限
  bool navigating() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }

  // 仅供日志展示
  std::string current_point() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return point_name_;
  }

private:
  NavExecutor & nav_;
  mutable std::mutex mutex_;
  std::uint64_t seq_{0};
  bool active_{false};                 // 在途导航或到达待消费（worker 轮询依据）
  std::string point_name_;
  std::optional<Arrival> arrived_;  // 到达槽（容量 1）
};

}  // namespace guide_pulse
