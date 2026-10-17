// ============================================================================
// 原厂 agent（运控板 aima-agent-app，DDS 域 /aimrt_agent_node）事件监控组件。
// 来源：agent_bridge_runtime 调研工程（2026-10-15 真机验证通过），
// 结论与话题清单见 docs/agent_bridge_notes.md。
//
// 架构：与 TtsClient/McTransform 同款——非独立节点，订阅挂在所属节点
//（桥接节点）上，由其 executor 驱动（launch 注入的 __node remap 会把
// 同进程手动创建的节点全部改名导致 rosout 同名冲突，故不做子节点）。
//
// 订阅轻量话题（QoS BEST_EFFORT，与发布端匹配）：
//   /aima/agent/interaction_state  AgentConversationState  对话状态机
//   /interaction/tts_status        TtsStatus               TTS 播报事件
//   /aima/agent/asr_result         AsrResult               ASR 识别结果
//（ASR 仅作观测）
// 并转为业务回调（构造后注入，在内部工作线程串行执行，不阻塞收包）：
//   on_wakeup          唤醒触发（IDLE -> LISTENING）
//   on_user_voice      用户说话中（sub_state=VOICE_ACTIVE）
//   on_speaking        机器人开始播报（SPEAKING + TTS BEGIN）
//   on_speaking_end    播报结束（TTS END/STOP/ERROR）
//   on_tts_event       原始 TTS 事件（状态码 + trace_id）
//   on_idle            整轮对话结束（回 IDLE）
//   on_user_utterance  用户一句完整识别文本（is_final，观测）
//   on_phrase          语料命中（action + 航点名 + 命中时的 agent 话术）
//   on_tts_text        TTS 话术原文查得（trace_id, 话术）
//
// 语料检测：agent 云端 TTS 应答话术（按 trace_id 回查运控板日志所得
// 原文，TtsStatus.text 恒空）按规则表做归一化子串匹配（normalize_
// phrase_text：去空白与中英文标点、ASCII 转小写），命中触发 on_phrase。
// 同一动作冷却 cooldown_seconds 内不重复触发（agent 可能重复复读）。
// navigate/relocate 动作延迟触发（2026-10-17 现场需求）：命中时 agent
// 正在说这句话（「请稍等」/「开始重定位」），立即执行体验差——挂起等
// 该 trace 的 TTS 终态（END/STOP/ERROR）再触发；兜底：对话状态机回
// IDLE（肯定说完）或超时（15s，TtsStatus 丢失时）。ASR 订阅仅作观测
//（on_user_utterance），不参与语料匹配。
//
// 资源设计：订阅回调轻量（只入队），业务逻辑在工作线程串行执行；
// 不订音频流等大流量话题。
// ============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <aimdk_msgs/msg/agent_conversation_state.hpp>
#include <aimdk_msgs/msg/asr_result.hpp>
#include <aimdk_msgs/msg/tts_status.hpp>

namespace guide_pulse
{

// 语料归一化：去空白与中英文标点（含全角），ASCII 字母转小写，保留
// 文字与数字（CJK 表意文字/扩展）。ASR 的标点与分词不稳定，语料匹配
// 前两侧都过本函数，配置语料无需关心标点写法
std::string normalize_phrase_text(const std::string & text);

// 语料匹配：归一化后的识别文本包含归一化后的语料即命中（用户在语料
// 前后夹语气词属正常；识别文本是语料子串不算命中，防短词误触发）
bool phrase_matches(const std::string & asr_text, const std::string & phrase);

class AgentMonitor final
{
public:
  // 语料规则：agent TTS 应答话术命中任一语料 -> 触发 action
  struct PhraseRule
  {
    std::string action;                // 动作名："welcome" -> scene=welcome 迎宾；
                                       // "show_qr" -> 屏幕显示默认二维码 3 分钟；
                                       // "cancel_qr" -> 取消二维码显示（撤屏，
                                       //   未显示 no-op，2026-10-17 现场需求；
                                       //   与 show_qr 同为互斥业务占 20s 窗口）；
                                       // "relocate" -> 在 point 航点上重定位；
                                       // "navigate" -> point 经点位映射表解析取
                                       //   code 后导航（2026-10-17 车型导览）
    std::string point;                 // relocate 专用：触发后重定位的航点名
                                       //（t1_slam_nav_ws 航点文件点位名，如 A）；
                                       // 其他动作空。冷却按 action+point 独立计时
    std::vector<std::string> phrases;  // 触发语料表（匹配 agent 说的 TTS 话术）
  };

  // 语料检测配置（main.cpp 从 yaml guide_bridge 段读取注入）
  struct Config
  {
    std::string asr_topic{"/aima/agent/asr_result"};  // ASR 识别结果话题
    int cooldown_seconds{10};          // 同一动作触发冷却秒数（防一轮对话重复触发）
    std::vector<PhraseRule> rules;     // 语料规则表（空 = 语料检测关闭）
    // 按 trace_id 回查 TTS 话术原文：TtsStatus.text 恒为空（原厂发布前
    // 丢弃，docs/agent_bridge_notes.md §6），话术只在运控板 agent.log；
    // SSH 目标/密码复用 display_remote_* 配置
    bool fetch_tts_text{true};
    std::string ssh_target{"run@10.1.1.100"};  // 运控板 SSH 目标
    std::string ssh_password{"1"};             // SSH 密码（shell 前缀环境变量传递）
    // 自产播报过滤：domain 等于本值的 TTS 事件不回查话术（自己发的
    // PlayTts 回流事件，话术在请求侧已知且运控板日志无对应记录）；
    // 空 = 不过滤
    std::string self_domain;
  };

  // ---- 业务回调（构造后注入，在工作线程串行执行；未设置则忽略）----
  std::function<void()> on_wakeup;
  std::function<void(const std::string &)> on_user_voice;      // event_id
  std::function<void(const std::string &)> on_speaking;        // trace_id
  std::function<void(const std::string &)> on_speaking_end;    // trace_id
  std::function<void(std::uint8_t, const std::string &)> on_tts_event;  // 状态码, trace_id
  std::function<void()> on_idle;
  std::function<void(const std::string &)> on_user_utterance;  // final 识别原文
  std::function<void(const std::string &, const std::string &, const std::string &)> on_phrase;
    // action, point（relocate 航点名，其他动作空）, 命中时的 agent 话术
  std::function<void(const std::string &, const std::string &)> on_tts_text;  // trace_id, 话术

  // node: 所属节点（桥接节点），订阅挂其默认回调组
  AgentMonitor(rclcpp::Node::SharedPtr node, const Config & config);
  ~AgentMonitor();

private:
  // 回调投递到工作线程（队列上限 64，防积压）
  void emit(std::function<void()> task);
  void on_conv_msg(const aimdk_msgs::msg::AgentConversationState::SharedPtr msg);
  void on_tts_msg(const aimdk_msgs::msg::TtsStatus::SharedPtr msg);
  void on_asr_msg(const aimdk_msgs::msg::AsrResult::SharedPtr msg);
  // 按 trace_id 回查运控板日志取 TTS 话术原文（工作线程执行，串行免锁）。
  // 覆盖两种 TTS 路径：本地 TTS（received request item_id=<trace> 行）
  // 与云端流式（item_id 匹配的 text.done JSON 行）；查到后触发 on_tts_text
  // 并做语料匹配
  void fetch_tts_text(const std::string & trace_id);
  // agent TTS 话术的语料匹配（含冷却），工作线程执行。trace_id = 话术
  // 来源 TTS（navigate/relocate 延迟触发挂起时等其终态用）
  void match_phrases(const std::string & text, const std::string & trace_id);
  // TTS 终态（END/STOP/ERROR）落地（工作线程）：记录已结束 trace；
  // 挂起中的延迟语料 trace 匹配则触发
  void on_tts_finished(const std::string & trace_id);
  // 触发挂起的延迟语料（无挂起 no-op）。why = 触发路径（日志用）。
  // 在工作线程直接调 on_phrase（不经队列，调用点已串行）
  void fire_deferred(const char * why);
  // 延迟语料超时兜底（worker 空转周期调用）：挂起到期仍未等来终态
  //（TtsStatus 丢失）强制触发
  void check_deferred_deadline();

  rclcpp::Node::SharedPtr node_;  // 所属节点（订阅与日志）
  Config config_;                 // 语料规则（构造后不变）

  std::atomic<std::uint32_t> last_main_{0};  // AgentConversationMainState::IDLE
  std::atomic<bool> speaking_{false};
  std::atomic<bool> idle_fired_{true};      // 抑制启动时初始 IDLE 状态的误报
  // 动作 -> 冷却截止时刻。仅工作线程（TTS 话术匹配）访问，无需加锁
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> cooldown_until_;
  // 已回查过话术的 trace_id（去重，每条 TTS 只查一次）。仅工作线程访问
  std::set<std::string> fetched_traces_;
  // ---- navigate/relocate 语料延迟触发挂起（2026-10-17 现场需求：等
  // agent 把命中语料的这句话说完——导航动身 / 重定位发定位。仅工作
  // 线程访问，免锁 ----
  struct DeferredPhrase
  {
    std::string action;               // "navigate" 或 "relocate"
    std::string point;                // 目标点位名（车型名/航点名）
    std::string text;                 // 命中时的 agent 话术（日志/回调透传）
    std::string trace_id;             // 话术来源 TTS 的 trace（终态比对用）
    std::chrono::steady_clock::time_point deadline;  // 超时兜底到期时刻
  };
  std::optional<DeferredPhrase> deferred_;
  // 已到终态的 TTS trace（话术回查慢于 END 的乱序判定：挂起前该 trace
  // 已结束则立即触发）。有界防增长：超限整体清空（丢的只是乱序判定，
  // 兜底由 deadline 承担）。仅工作线程访问
  std::unordered_set<std::string> finished_traces_;

  rclcpp::Subscription<aimdk_msgs::msg::AgentConversationState>::SharedPtr sub_conv_;
  rclcpp::Subscription<aimdk_msgs::msg::TtsStatus>::SharedPtr sub_tts_;
  rclcpp::Subscription<aimdk_msgs::msg::AsrResult>::SharedPtr sub_asr_;

  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<std::function<void()>> queue_;
  std::atomic<bool> running_{true};
};

}  // namespace guide_pulse
