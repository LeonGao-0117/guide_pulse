// ============================================================================
// PlaylistSession：共享的播报列表会话状态机（纯逻辑，无 ROS/MQTT 依赖）。
//
// tts_demo 的 cycle 状态机抽象（sequence/shuffle 轮次 + 语音时长估算间隔），
// 单轮模式：loop_forever=false 时一轮播完会话自行结束——tts/迎宾词/
// 讲解词（scene/navigate 的 texts）均单轮播完。驱动方（桥接节点
// worker 线程）负责调用 SDK。
// ============================================================================
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "guide_pulse/guide_protocol.hpp"

namespace guide_pulse
{

class PlaylistSession
{
public:
  // texts 播报列表（1..20 条）；mode sequence|shuffle；
  // interval_seconds 每两条之间的间隔；loop_forever false=单轮结束
  PlaylistSession(
    std::vector<std::string> texts, const std::string & mode, int interval_seconds,
    bool loop_forever)
  : source_(std::move(texts)), round_(make_round_order(source_, mode)),
    mode_(mode), interval_seconds_(interval_seconds), loop_forever_(loop_forever) {}

  // 当前应播的文本（未推进状态）
  const std::string & current() const {return round_.at(index_);}

  // 单次播报成功后推进；返回 false 表示会话已播完（仅单轮模式会出现），
  // 驱动方应结束会话。失败重试不调用本函数（索引不动）。
  bool advance()
  {
    ++index_;
    if (index_ < round_.size()) {
      return true;
    }
    // 一轮播完
    index_ = 0;
    ++round_no_;
    if (!loop_forever_) {
      finished_ = true;
      return false;
    }
    if (mode_ == TTS_MODE_SHUFFLE) {
      round_ = make_round_order(source_, mode_);  // 契约建议每轮重洗
    }
    return true;
  }

  bool finished() const {return finished_;}

  // 下一次播报时刻的等待秒数：语音时长估算 + 每两条间隔（驱动方拼接用）
  int next_delay_seconds() const
  {
    return estimate_speech_seconds(current()) + interval_seconds_;
  }

  std::size_t round_no() const {return round_no_;}
  std::size_t index() const {return index_;}
  std::size_t size() const {return round_.size();}

private:
  std::vector<std::string> source_;  // 母本（shuffle 重洗来源）
  std::vector<std::string> round_;   // 当前轮播放序列
  std::size_t index_{0};
  std::size_t round_no_{1};
  std::string mode_;
  int interval_seconds_{0};
  bool loop_forever_{true};
  bool finished_{false};
};

}  // namespace guide_pulse
