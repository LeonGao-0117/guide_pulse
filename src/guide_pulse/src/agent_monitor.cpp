// ============================================================================
// AgentMonitor 实现：话题订阅、状态机判定与语料匹配，见 agent_monitor.hpp
// 文件头。
// ============================================================================
#include "guide_pulse/agent_monitor.hpp"

#include <cctype>
#include <utility>

namespace guide_pulse
{

using aimdk_msgs::msg::AgentConversationMainState;
using aimdk_msgs::msg::AgentConversationState;
using aimdk_msgs::msg::AgentConversationSubState;
using aimdk_msgs::msg::AsrResult;
using aimdk_msgs::msg::TtsStatus;
using aimdk_msgs::msg::TtsStatusType;

// ---------------------------------------------------------------------------
// 语料归一化与匹配（纯函数，单测覆盖：test_agent_monitor.cpp）
// ---------------------------------------------------------------------------
namespace
{

// UTF-8 解码一个码点；失败返回 false（非法序列，调用方终止处理）
bool decode_utf8(const std::string & s, std::size_t & i, std::uint32_t & cp)
{
  const auto b0 = static_cast<unsigned char>(s[i]);
  std::size_t len = 0;
  std::uint32_t val = 0;
  if ((b0 & 0xE0) == 0xC0) { len = 2; val = b0 & 0x1F; }
  else if ((b0 & 0xF0) == 0xE0) { len = 3; val = b0 & 0x0F; }
  else if ((b0 & 0xF8) == 0xF0) { len = 4; val = b0 & 0x07; }
  else { return false; }
  if (i + len > s.size()) { return false; }
  for (std::size_t k = 1; k < len; ++k) {
    const auto bk = static_cast<unsigned char>(s[i + k]);
    if ((bk & 0xC0) != 0x80) { return false; }
    val = (val << 6) | (bk & 0x3F);
  }
  i += len;
  cp = val;
  return true;
}

void append_utf8(std::string & out, std::uint32_t cp)
{
  if (cp < 0x80) {
    out.push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp < 0x10000) {
    out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

}  // namespace

std::string normalize_phrase_text(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  std::size_t i = 0;
  while (i < text.size()) {
    const auto b = static_cast<unsigned char>(text[i]);
    if (b < 0x80) {
      // ASCII：仅保留字母数字（转小写），空白与标点丢弃
      if (std::isalnum(b) != 0) {
        out.push_back(static_cast<char>(std::tolower(b)));
      }
      ++i;
      continue;
    }
    std::uint32_t cp = 0;
    if (!decode_utf8(text, i, cp)) {
      break;  // 非法 UTF-8：保守终止（防御，ASR 文本应为合法 UTF-8）
    }
    // 保留 CJK 表意文字（含扩展 A/兼容表意）；CJK 标点/全角符号/
    // 各类空白外的其他符号一律丢弃
    if ((cp >= 0x3400 && cp <= 0x4DBF) || (cp >= 0x4E00 && cp <= 0x9FFF) ||
      (cp >= 0xF900 && cp <= 0xFAFF))
    {
      append_utf8(out, cp);
    }
  }
  return out;
}

bool phrase_matches(const std::string & asr_text, const std::string & phrase)
{
  const auto normalized_asr = normalize_phrase_text(asr_text);
  const auto normalized_phrase = normalize_phrase_text(phrase);
  if (normalized_phrase.empty()) {
    return false;  // 语料归一化后为空（纯标点）不作为有效触发词
  }
  return normalized_asr.find(normalized_phrase) != std::string::npos;
}

// ---------------------------------------------------------------------------
// 节点实现
// ---------------------------------------------------------------------------
AgentMonitor::AgentMonitor(rclcpp::Node::SharedPtr node, const Config & config)
: node_(std::move(node)), config_(config)
{
  // QoS 与发布端匹配：BEST_EFFORT
  rclcpp::QoS qos(10);
  qos.best_effort();
  sub_conv_ = node_->create_subscription<AgentConversationState>(
    "/aima/agent/interaction_state", qos,
    [this](AgentConversationState::SharedPtr msg) { on_conv_msg(msg); });
  sub_tts_ = node_->create_subscription<TtsStatus>(
    "/interaction/tts_status", qos,
    [this](TtsStatus::SharedPtr msg) { on_tts_msg(msg); });
  if (!config_.rules.empty()) {
    RCLCPP_INFO(
      node_->get_logger(), "语料检测开启: 规则 %zu 组 冷却 %ds（匹配 agent TTS 应答话术）",
      config_.rules.size(), config_.cooldown_seconds);
  }
  // ASR 订阅仅作观测（打印用户原话，排查用），不参与语料匹配
  sub_asr_ = node_->create_subscription<AsrResult>(
    config_.asr_topic, qos,
    [this](AsrResult::SharedPtr msg) { on_asr_msg(msg); });
  // 回调工作线程：慢逻辑不阻塞 DDS 收包。空转唤醒周期顺带做延迟
  // 语料的超时兜底检查（锁外执行，fire_deferred 不碰队列锁）
  worker_ = std::thread([this] {
    while (running_.load()) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait_for(lock, std::chrono::milliseconds(200), [this] { return !queue_.empty(); });
        if (!queue_.empty()) {
          task = std::move(queue_.front());
          queue_.pop();
        }
      }
      if (task) {
        task();
      } else {
        check_deferred_deadline();
      }
    }
  });
}

AgentMonitor::~AgentMonitor()
{
  running_.store(false);
  queue_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

void AgentMonitor::emit(std::function<void()> task)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (queue_.size() < 64) queue_.push(std::move(task));
  queue_cv_.notify_one();
}

void AgentMonitor::on_conv_msg(const AgentConversationState::SharedPtr msg)
{
  const std::uint32_t main = msg->main_state.value;
  const std::uint32_t sub = msg->sub_state.value;
  const std::uint32_t prev = last_main_.exchange(main);
  idle_fired_.store(main == AgentConversationMainState::IDLE);  // 离开 IDLE 重置结束标志

  // 唤醒：空闲 -> 聆听（区别于播报完回聆听）
  if (prev == AgentConversationMainState::IDLE && main == AgentConversationMainState::LISTENING) {
    auto cb = on_wakeup;
    emit([cb] { if (cb) cb(); });
  }
  // 用户开口
  if (main == AgentConversationMainState::LISTENING && sub == AgentConversationSubState::VOICE_ACTIVE) {
    std::string event_id = msg->event_id;
    auto cb = on_user_voice;
    emit([cb, event_id] { if (cb) cb(event_id); });
  }
  // 开始播报（SPEAKING 状态携带 trace_id，比 TtsStatus 终止态早整个
  // 播报时长——云端链路话术回查从这里提前触发；text.done 日志在开播
  // 前已写入，与 TtsStatus 触发点靠 fetched_traces_ 去重。注：对话状态
  // 无 domain 字段，自产播报也会触发一次回查，落空即止）
  if (main == AgentConversationMainState::SPEAKING && !speaking_.load()) {
    speaking_.store(true);
    std::string trace_id = msg->trace_id;
    auto cb = on_speaking;
    emit([cb, trace_id] { if (cb) cb(trace_id); });
    if (config_.fetch_tts_text && !trace_id.empty()) {
      emit([this, trace_id] { fetch_tts_text(trace_id); });
    }
  }
  // 整轮对话结束（回 IDLE 且不在播报）
  if (main == AgentConversationMainState::IDLE && !speaking_.load() && !idle_fired_.exchange(true)) {
    auto cb = on_idle;
    emit([cb] { if (cb) cb(); });
    // 延迟语料兜底：整轮对话结束 = 话肯定说完了（TtsStatus 终态丢失
    // 场景；正常路径该 trace 终态早已触发，此处 no-op）
    emit([this] { fire_deferred("对话结束"); });
  }
}

void AgentMonitor::on_tts_msg(const TtsStatus::SharedPtr msg)
{
  const std::uint8_t status = msg->tts_status.value;
  std::string trace_id = msg->trace_id;
  const bool finished = status == TtsStatusType::TTS_STATUE_TYPE_END ||
                        status == TtsStatusType::TTS_STATUS_TYPE_STOP ||
                        status == TtsStatusType::TTS_STATUS_TYPE_ERROR;

  // 原始事件（BEGIN/END/STOP/ERROR）
  if (status == TtsStatusType::TTS_STATUS_TYPE_BEGIN || finished) {
    auto cb = on_tts_event;
    emit([cb, status, trace_id] { if (cb) cb(status, trace_id); });
  }
  // 状态事件触发异步回查话术原文（一轮对话可含多条 TTS，trace 各异）。
  // 2026-10-15 22:51 真机实测：云端流式链路的 TtsStatus 只发 END(3)，
  // 不发 BEGIN(1)（本地 TTS 链路两者都发）——BEGIN 与终止态均触发，
  // 同一 trace 由 fetched_traces_ 去重。
  // 注：实测原厂/自产 TtsStatus.domain 恒为空，self_domain 过滤当前
  // 不生效，保留待原厂填充 domain 后自行起效
  if (config_.fetch_tts_text && !trace_id.empty() &&
    (status == TtsStatusType::TTS_STATUS_TYPE_BEGIN || finished) &&
    msg->domain != config_.self_domain)
  {
    emit([this, trace_id] { fetch_tts_text(trace_id); });
  }
  // 播报结束：END/STOP/ERROR
  if (finished) {
    if (speaking_.exchange(false)) {
      auto cb = on_speaking_end;
      emit([cb, trace_id] { if (cb) cb(trace_id); });
    }
    // 终态落地（无条件，不依赖 speaking_ 标志）：记录已结束 trace，
    // 挂起中的延迟语料（navigate 等说完再触发）trace 匹配则触发
    emit([this, trace_id] { on_tts_finished(trace_id); });
  }
}

void AgentMonitor::on_asr_msg(const AsrResult::SharedPtr msg)
{
  if (!msg->is_final) {
    return;  // 只上报最终结果（中间结果不稳定）
  }
  const std::string text = msg->text;
  if (text.empty()) {
    return;
  }
  // 每句最终识别文本上报（观测/排查：语料未命中时先看识别原文）
  auto utter_cb = on_user_utterance;
  emit([utter_cb, text] { if (utter_cb) utter_cb(text); });
}

// ---------------------------------------------------------------------------
// agent TTS 话术的语料匹配（工作线程串行执行，成员免锁）
// ---------------------------------------------------------------------------
bool AgentMonitor::match_phrases(const std::string & text, const std::string & trace_id)
{
  const auto now = std::chrono::steady_clock::now();
  for (const auto & rule : config_.rules) {
    bool hit = false;
    for (const auto & phrase : rule.phrases) {
      if (phrase_matches(text, phrase)) {
        hit = true;
        break;
      }
    }
    if (!hit) {
      continue;
    }
    // 冷却键 = action+point：relocate 的 A~E 五点互不挤占（各点话术不同，
    // 不存在互相复读；冷却只为防同一点被 agent 复读重复触发）
    auto & until = cooldown_until_[rule.action + "@" + rule.point];
    if (now < until) {
      RCLCPP_DEBUG(
        node_->get_logger(), "语料命中但冷却中(%lds): action=%s",
        std::chrono::duration_cast<std::chrono::seconds>(until - now).count(),
        rule.action.c_str());
      return true;  // 一条话术至多触发第一个命中的规则（已被消化）
    }
    until = now + std::chrono::seconds(config_.cooldown_seconds);
    RCLCPP_INFO(
      node_->get_logger(), "语料命中: action=%s point=%s agent话术「%s」",
      rule.action.c_str(), rule.point.c_str(), text.c_str());
    // navigate/relocate/welcome_prep 延迟触发（2026-10-17 现场需求；
    // welcome_prep 2026-10-18 迎宾准备）：命中时 agent
    // 正在说这句话（「请稍等」/「开始重定位」/「马上去迎宾点」），立即执行体验差——navigate 边说边动身、relocate 话音未落就发「重定位成功」自产播报
    // 与 agent 话音相撞——挂起等该 trace 的 TTS 终态再触发。该 trace
    // 已结束（话术回查慢于终态的乱序，如本地链路 BEGIN 触发的回查落空
    // 重试）则立即触发不等待。15s 为 TtsStatus 丢失的兜底上限（话术
    // ~20 字 ≈ 5s 播完，宽裕；到期由 check_deferred_deadline 强制触发）
    if ((rule.action == "navigate" || rule.action == "relocate" ||
         rule.action == "welcome_prep") && !trace_id.empty() &&
      finished_traces_.count(trace_id) == 0)
    {
      deferred_ = DeferredPhrase{
        rule.action, rule.point, text, trace_id,
        now + std::chrono::seconds(15)};
      RCLCPP_INFO(
        node_->get_logger(),
        "语料挂起等播报结束: action=%s point=%s trace=%s（说完再触发，15s 兜底）",
        rule.action.c_str(), rule.point.c_str(), trace_id.c_str());
      return true;
    }
    auto cb = on_phrase;
    emit([cb, action = rule.action, point = rule.point, text] {
      if (cb) {cb(action, point, text);}
    });
    return true;  // 一条话术至多触发第一个命中的规则
  }
  return false;  // 未命中任何规则
}

// ---------------------------------------------------------------------------
// navigate 延迟语料的挂起与触发（均工作线程串行执行，成员免锁）
// ---------------------------------------------------------------------------
void AgentMonitor::fire_deferred(const char * why)
{
  if (!deferred_.has_value()) {
    return;
  }
  const auto d = std::move(*deferred_);
  deferred_.reset();
  // 不打印触发行（2026-10-18 日志整理：挂起行已预告「说完再触发」，
  // 触发时刻由后续业务行体现）；why 仅调试期排查用
  (void)why;
  auto cb = on_phrase;
  if (cb) {
    cb(d.action, d.point, d.text);  // 调用点已在工作线程，直接调不经队列
  }
}

void AgentMonitor::on_tts_finished(const std::string & trace_id)
{
  // 有界：超限整体清空防长期运行无限增长；丢的只是「挂起前已结束」
  // 的乱序判定，兜底由挂起超时承担
  if (finished_traces_.size() >= 128) {
    finished_traces_.clear();
  }
  finished_traces_.insert(trace_id);
  if (deferred_ && deferred_->trace_id == trace_id) {
    fire_deferred("播报结束");
  }
}

void AgentMonitor::check_deferred_deadline()
{
  if (deferred_ && std::chrono::steady_clock::now() >= deferred_->deadline) {
    fire_deferred("超时兜底");
  }
}

// ---------------------------------------------------------------------------
// TTS 话术原文回查（工作线程串行执行，成员免锁）。
// TtsStatus.text 恒为空（原厂转发时丢弃），话术只在运控板 agent.log：
//   本地 TTS： 'TTSManager received request item_id=<trace> ... text=话术 domain='
//   云端流式： item_id 匹配的 active_chat.response.text.done JSON 行的 "text"
//   云端音频直推（2026-10-19 现场发现：腾势D9 介绍无 text.done、非本地
//     合成，两条旧路径均落空）：response.audio.delta 音频流直推 TTSManager
//     （item_id 带 _0_ 前缀、无 _S 后缀），文本在
//     'TTSManager dispatching status item_id=<trace> ... text="话术" domain="xxx"'
//     行。注意该行超长时 logger 会折行：主行带时间戳、~ 结尾，余下内容
//     在无时间戳续行（2026-10-19 现场：M9 固定语位于长话术尾部全在续行，
//     单行 grep 永远拿不到完整话术）——回查须 -A3 带续行拼接后提取
// ---------------------------------------------------------------------------
namespace
{

// trace_id 白名单：DDS 外部数据拼 shell 命令，仅放行安全字符防注入
bool safe_trace_id(const std::string & trace_id)
{
  if (trace_id.empty() || trace_id.size() > 128) {
    return false;
  }
  for (const char c : trace_id) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
    if (!ok) {
      return false;
    }
  }
  return true;
}

// 执行合并回查命令：一次 SSH 同时跑本地/云端/音频直推三条 grep（=1=/=2=
// /=3= 标记分段），返回各自首个非空输出行（空串 = 未查到/失败）
struct FetchLines
{
  std::string local_line;   // 路径1输出：形如 "text=话术 domain="
  std::string cloud_line;   // 路径2输出：纯话术文本
  std::string stream_line;  // 路径3输出：形如 "text=\"话术\" domain=\"omnissdk\""
};

FetchLines ssh_fetch_lines(const std::string & cmd)
{
  FetchLines out;
  FILE * p = ::popen(cmd.c_str(), "r");  // NOLINT：命令由白名单校验的片段拼装
  if (p == nullptr) {
    return out;
  }
  char buf[1024];
  int section = 0;
  while (fgets(buf, sizeof(buf), p) != nullptr) {
    std::string line = buf;
    line.erase(line.find_last_not_of(" \r\n\t") + 1);
    if (line == "=1=") {section = 1; continue;}
    if (line == "=2=") {section = 2; continue;}
    if (line == "=3=") {section = 3; continue;}
    if (line.empty()) {continue;}
    if (section == 1 && out.local_line.empty()) {
      out.local_line = line;
    } else if (section == 2 && out.cloud_line.empty()) {
      out.cloud_line = line;
    } else if (section == 3 && out.stream_line.empty()) {
      out.stream_line = line;
    }
  }
  pclose(p);
  return out;
}

// 简易 base64 编码（标准字母表，无换行）：远程命令编码后经单引号
// 传递，本地/远端两侧均零转义（多层引号转义在 dash/bash 下行为有
// 差异，2026-10-16 曾因此翻车——赋值路径被转义出非法字符导致快照
// 文件写失败、回查恒空）
std::string base64_encode(const std::string & in)
{
  static const char tbl[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((in.size() + 2) / 3 * 4);
  std::uint32_t buf = 0;
  int bits = 0;
  for (const unsigned char c : in) {
    buf = (buf << 8) | c;
    bits += 8;
    if (bits == 24) {
      out += tbl[(buf >> 18) & 63];
      out += tbl[(buf >> 12) & 63];
      out += tbl[(buf >> 6) & 63];
      out += tbl[buf & 63];
      bits = 0;
      buf = 0;
    }
  }
  if (bits == 8) {
    buf <<= 16;
    out += tbl[(buf >> 18) & 63];
    out += tbl[(buf >> 12) & 63];
    out += "==";
  } else if (bits == 16) {
    buf <<= 8;
    out += tbl[(buf >> 18) & 63];
    out += tbl[(buf >> 12) & 63];
    out += tbl[(buf >> 6) & 63];
    out += '=';
  }
  return out;
}

}  // namespace

void AgentMonitor::fetch_tts_text(const std::string & trace_id)
{
  if (!fetched_traces_.insert(trace_id).second) {
    return;  // 该 trace 已查过（SPEAKING 与 TtsStatus 双触发只查一次）
  }
  if (!safe_trace_id(trace_id) || config_.ssh_password.find('\'') != std::string::npos) {
    return;  // trace/密码含非法字符：静默跳过回查（防御，不打印）
  }
  // 一次 SSH 同时跑三条路径（省第二次连接 1~2s）。远程命令直白书写后
  // base64 编码、单引号传递解码执行——两侧 shell 均零转义。
  // 各条 grep 打当前档+最新滚动档（$F，ls -t 取前二；2026-10-19 现场
  // 亲历 agent.log 100MB 滚动把几分钟前的话术行滚进旧档）：日志含高频
  // 音频流（空闲 ~0.6MB/min），任何尾部窗口都可能把话术行挤出；全量
  // ~100MB 单模式 grep ~0.5s 可接受
  //   =1= 本地 TTS：'received request item_id=<trace> ' 行 -> text=话术 domain=
  //   =2= 云端流式：trace 匹配的 text.done JSON 行 -> "text":"话术"
  //   =3= 云端音频直推：'dispatching status item_id=<trace> ' 行 ->
  //       text="话术" domain="xxx"。该行超长会折行（主行 ~ 结尾、余下在
  //       无时间戳续行），-A3 带续行、剔 -- 分隔、tr 把换行与折行符 ~
  //       压成空格拼成整条记录，再按 text="[^"]*" domain="[^"]*" 逐段
  //       提取（拼接话术里的多余空格过归一化自然消解）；剔除 text=""
  //       空段后取最后一条（状态流转发多次，末尾终态行文本最全）。
  //       多文件 grep 用 -h 抑制文件名前缀（=1=/=2= 同）
  const std::string remote =
    "F=$(ls -t /robot/persist/log/agent/agent.log* | head -2); "
    "echo =1=; "
    "grep -ahm1 'received request item_id=" + trace_id + " ' $F"
    " | grep -ao 'text=.* domain='; "
    "echo =2=; "
    "grep -ah '" + trace_id + "' $F"
    " | grep -a text.done | head -1"
    " | grep -o '\"text\":\"[^\"]*\"' | cut -d'\"' -f4; "
    "echo =3=; "
    "grep -ah -A3 'dispatching status item_id=" + trace_id + " ' $F"
    " | grep -av '^--' | tr '\\n~' '  '"
    " | grep -ao 'text=\"[^\"]*\" domain=\"[^\"]*\"'"
    " | grep -av '^text=\"\"' | tail -1";
  const std::string cmd =
    "SSHPASS='" + config_.ssh_password + "' sshpass -e ssh "
    "-o StrictHostKeyChecking=accept-new -o ConnectTimeout=3 " + config_.ssh_target +
    " 'echo " + base64_encode(remote) + " | base64 -d | sh'";

  for (int attempt = 0; attempt < 2; ++attempt) {  // 日志写入延迟时重试一次
    const auto out = ssh_fetch_lines(cmd);
    // 路径 1：取 "text=... domain=" 的中间段
    if (!out.local_line.empty()) {
      const auto a = out.local_line.find("text=");
      const auto b = out.local_line.rfind(" domain=");
      if (a != std::string::npos) {
        std::string text = out.local_line.substr(
          a + 5, b != std::string::npos && b > a ? b - a - 5 : std::string::npos);
        // 自产播报过滤（2026-10-18 日志整理）：本地 TTS 日志行带
        // domain=<self_domain> 即本进程发的 PlayTts（话术请求侧已知，
        // 「稍等，马上出发」等提示回查再打一遍纯冗余），直接丢弃。
        // TtsStatus.domain 恒空的原有过滤不生效，此处按日志行解析兜底
        std::string domain;
        if (b != std::string::npos && b > a) {
          domain = out.local_line.substr(b + 8);
        }
        if (!text.empty()) {
          if (!domain.empty() && domain == config_.self_domain) {
            return;  // 自产播报：不打不匹配
          }
          // 先匹配语料：命中则「语料命中」行已含话术，跳过 on_tts_text
          //（免同句双打）；未命中才打「agent 事件: TTS 话术」观测行
          if (!match_phrases(text, trace_id)) {
            auto cb = on_tts_text;
            emit([cb, trace_id, text] { if (cb) cb(trace_id, text); });
          }
          return;
        }
      }
    }
    // 路径 2：输出已是纯文本
    if (!out.cloud_line.empty()) {
      const std::string text = out.cloud_line;
      if (!match_phrases(text, trace_id)) {
        auto cb = on_tts_text;
        emit([cb, trace_id, text] { if (cb) cb(trace_id, text); });
      }
      return;
    }
    // 路径 3：云端音频直推——输出形如 text="话术" domain="omnissdk"
    //（折行拼接后话术中可能夹折行空格，归一化匹配自然消解；引号内
    // 用 [^"]* 防跨段贪婪，domain 与 self_domain 比对过滤自产播报）
    if (!out.stream_line.empty()) {
      const auto a = out.stream_line.find("text=\"");
      const auto b = out.stream_line.rfind("\" domain=\"");
      if (a != std::string::npos && b != std::string::npos && b > a) {
        std::string text = out.stream_line.substr(a + 6, b - a - 6);
        std::string domain = out.stream_line.substr(b + 10);
        if (!domain.empty() && domain.back() == '"') {
          domain.pop_back();  // 去尾部引号
        }
        if (!text.empty()) {
          if (!domain.empty() && domain == config_.self_domain) {
            return;  // 自产播报：不打不匹配
          }
          if (!match_phrases(text, trace_id)) {
            auto cb = on_tts_text;
            emit([cb, trace_id, text] { if (cb) cb(trace_id, text); });
          }
          return;
        }
      }
    }
    if (attempt == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(600));
    }
  }
  // 落空：解除去重标记，允许后续触发点（TtsStatus END 等）重查——
  // 云端分句流式场景 SPEAKING 早于 text.done 写入，提前查会落空，
  // 不能挡住播完后的重查
  fetched_traces_.erase(trace_id);
  RCLCPP_DEBUG(
    node_->get_logger(), "TTS 话术未查到（日志可能未写入或自产播报）: trace=%s",
    trace_id.c_str());
}

}  // namespace guide_pulse
