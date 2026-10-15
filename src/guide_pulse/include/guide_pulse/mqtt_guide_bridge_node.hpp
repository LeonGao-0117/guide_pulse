// ============================================================================
// MQTT <-> 展厅讲解 桥接节点：enroll 换凭据 -> 订阅 down/commands
//（tts/tts.stop/scene/navigate）-> 播报/导航/迎宾/动作执行 -> 回执与
// presence/heartbeat 上行。详见 docs/design.md。
// MQTT：vendored paho（async_client + MQTT v5），下行先进队列由 worker
// 线程处理，重连由专属连接线程驱动（凭据轮换需应用层控制）。
// ============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <mqtt/async_client.h>
#include <rclcpp/rclcpp.hpp>

#include <aimdk_msgs/srv/set_agent_suppression.hpp>
#include <aimdk_msgs/srv/set_mc_preset_motion.hpp>

#include "guide_pulse/guide_protocol.hpp"
#include "guide_pulse/mc_transform.hpp"
#include "guide_pulse/mqtt_guide_bridge_node.hpp"
#include "guide_pulse/nav_executor.hpp"
#include "guide_pulse/playlist.hpp"
#include "guide_pulse/qr_code_cache.hpp"
#include "guide_pulse/robot_enroll.hpp"
#include "guide_pulse/screen_display.hpp"
#include "guide_pulse/tour.hpp"
#include "guide_pulse/tts_client.hpp"
#include "guide_pulse/waypoint_nav_executor.hpp"
#include "guide_pulse/welcome.hpp"

namespace guide_pulse
{

class MqttGuideBridgeNode final : public rclcpp::Node, private mqtt::callback
{
public:
  explicit MqttGuideBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MqttGuideBridgeNode() override;

  // 连接 MQTT broker（阻塞至连接建立或超时）；返回是否成功
  bool start();
  // 停止桥接：停工作线程、取消导航、发布 offline presence、断开连接
  void shutdown() noexcept;

private:
  // ---- mqtt::callback ----
  void connected(const std::string & cause) override;
  void connection_lost(const std::string & cause) override;
  void message_arrived(mqtt::const_message_ptr message) override;

  // ---- 内部流程 ----
  void declare_and_load_params();
  bool apply_credential(const RobotCredential & credential);
    // 应用当次 enroll 凭据；robot_id 变化时重建客户端与主题（防御性路径）
  void build_topics();                     // 由 robot_id 生成四个契约主题
  mqtt::connect_options build_connect_options();
    // 每次连接前重建（遗嘱须带本次 session_id，凭据取当次 enroll 结果）
  void connect_loop();                     // 连接管理线程：enroll -> 连接 -> 重试
  void worker_loop();
  void maintain_connection();   // 回调线程外的订阅恢复/上线通告（防 paho 自死锁）
  void handle_command(const CommandEvent & event);  // tts / tts.stop / scene / navigate
  void handle_scene(const CommandEvent & event);        // 迎宾模块入口
  void handle_navigate(const CommandEvent & event);     // 讲解模块入口
  // 迎宾变形（确保人形形态）由 McTransform 模块承担（先变形、后人声的
  // 同步点在 handle_scene 内 transform_->wait_done()）
  // 播报伴随动作：TTS 受理成功时触发预设动作（轮换池取下一个）。
  // 同步短超时（正常毫秒级返回）；失败仅告警绝不影响语音播报
  void trigger_speech_motion();
  // 开播一个音频会话（替换当前会话——音频单会话策略）
  void start_session(
    std::vector<std::string> texts, const std::string & mode, int interval_seconds,
    bool loop_forever, const std::string & source);
  TtsClient::Result play_current();  // 播会话当前条并推进/排程（worker 线程）
  void cancel_session();             // 终止当前播报会话（不掐语音）
  // 唤醒抑制原子调用：发送 SetAgentSuppression(WAKEUP, ttl_ms)。
  // ttl_ms=0 时服务端用默认 3s（用于会话结束后的快速恢复刷新）
  void suppress_wakeup(std::uint32_t ttl_ms);
  void publish_presence(const std::string & state, const std::string & reason);
  void publish_heartbeat();
  void send_tts_result(
    const std::string & operation_id, const std::string & outcome,
    const std::string & reason = "");  // 首次生成并缓存，重复原样重发
  bool publish_json(
    const std::string & topic, const std::string & payload, int qos, bool retained);
  bool seen_recently(const std::string & key);  // operation_id/event 幂等去重

  // ---- 参数 ----
  std::string broker_uri_;
  std::string enroll_origin_;           // Server HTTP 注册面，如 http://host:8180
  std::string serial_number_;           // 本机序列号（须运维预登记）
  int enroll_timeout_seconds_{5};
  std::string topic_prefix_;            // 默认 robotCar/v1/robots（Server 固定值）
  std::string client_version_;          // presence/heartbeat 契约字段
  std::string tts_service_name_;        // 默认 /aimdk_5Fmsgs/srv/PlayTts
  std::string tts_domain_;              // 调用方来源标识
  std::string stop_text_;               // 停止播报使用的近静音文本
  int tts_priority_level_{6};           // SDK 播报主档位（仲裁 level*100+weight）
  int tts_priority_weight_{0};          // SDK 同档位加权 0-99
  int tts_retry_seconds_{3};            // 被仲裁拒绝后的快速重试间隔
  int keep_alive_seconds_{30};
  int connect_timeout_seconds_{10};
  int heartbeat_interval_seconds_{30};
  int tts_request_timeout_seconds_{5};
  int tts_wait_timeout_seconds_{0};     // 启动等待服务时长，0=不等待
  int dedupe_capacity_{1024};
  // ---- 唤醒抑制：看门狗续期（TTL 短，worker 周期续期贴住实际播报
  // 时长；SDK 无播完事件，尾部宽限 lead+估算×scale，参数校准见 yaml）----
  std::string suppress_service_name_;   // 默认 /aimdk_5Fmsgs/srv/SetAgentSuppression
  bool suppress_wakeup_{true};          // false 关闭（恢复默认可打断行为）
  int suppress_ttl_ms_{8000};           // 每次抑制的 TTL（须大于续期间隔）
  int suppress_renew_interval_ms_{5000};  // 续期间隔（worker 周期刷新）
  // 尾部宽限：tail_ms = lead + 估算秒×1000×scale（max>0 封顶）；
  // flush=停续期收尾刷的 TTL（0=云端默认 3000ms，1=立即解除）
  int tts_speech_ms_per_char_{220};     // 每字时长（CJK/数字档，拉丁≈0.73 倍）
  int suppress_tail_lead_ms_{0};        // 固定合成启动延迟（校准填拟合截距）
  double suppress_tail_scale_{0.8};     // 估算放大系数
  int suppress_tail_max_ms_{0};         // 尾部宽限封顶（0=不封顶）
  int suppress_tail_flush_ms_{1};       // 停续期收尾刷的抑制 TTL
  bool suppress_renew_active_{false};   // 续期进行中（仅 worker 线程访问）
  std::chrono::steady_clock::time_point suppress_next_at_;  // 下次续期时刻
  std::optional<std::chrono::steady_clock::time_point> suppress_stop_at_;
    // 尾部宽限截止：最后一条已受理（语音仍在播），续期持续到该时刻再停
  // 迎宾互斥窗口截止：窗口内新 scene/navigate 拒绝（无回执面，丢弃记
  // 日志）；任一会话替换/tts.stop 即失效（迎宾音频被打断 = 流程终止）
  std::optional<std::chrono::steady_clock::time_point> welcome_busy_until_;
  // ---- 展厅讲解增量参数 ----
  std::vector<std::string> welcome_default_texts_;  // scene 未带 texts 时的迎宾词
  int tour_speak_interval_seconds_{2};  // 迎宾/讲解词每两条间隔（载荷无间隔字段）
  std::string nav_executor_name_;       // 当前仅 mock（真机实现后扩展）
  int mock_nav_seconds_{0};             // Mock 导航模拟耗时
  // ---- 迎宾变形（scene=welcome 时确保人形形态）：由 McTransform 模块
  // 承担，参数校验在 start() 内完成（welcome_transform 等）----

  // ---- 播报伴随动作（TTS 受理成功时触发预设动作）----
  std::string mc_source_name_;          // SetMc* 请求 source（变形模块共用）
  bool motion_enabled_{true};            // false 关闭（只播语音不做动作）
  std::vector<int32_t> motion_preset_ids_;   // 动作轮换池（参数注入；空=关闭）
  std::size_t motion_next_index_{0};     // 轮换索引（仅 worker 线程访问）
  int motion_request_timeout_seconds_{2};
  rclcpp::Client<aimdk_msgs::srv::SetMcPresetMotion>::SharedPtr motion_client_;
  rclcpp::CallbackGroup::SharedPtr motion_callback_group_;
  // ---- 迎宾二维码（v0.22.0 P-036：哈希快照 + 文件面按需下载）----
  std::string qr_cache_dir_;            // 本地二维码缓存目录（跨重启保留）
  int qr_download_timeout_seconds_{5};  // 单次下载（连接+传输）超时

  // ---- 运行时状态 ----
  std::string robot_id_;               // enroll 响应分配（同 SN 稳定），空=尚未取得
  std::string username_;               // 当次 enroll 的 MQTT 凭据（仅内存）
  std::string password_;
  std::string down_topic_;              // {prefix}/{id}/down/commands（下行指令）
  std::string up_commands_topic_;       // {prefix}/{id}/up/commands（tts.result 回执）
  std::string heartbeat_topic_;
  std::string presence_topic_;
  std::string boot_id_;
  std::string session_id_;              // 每次连接重生成（uuidv7，契约要求）
  std::uint64_t heartbeat_sequence_{0};  // 契约要求从 1 递增
  std::atomic_bool ever_online_{false};
  std::chrono::steady_clock::time_point started_at_;
  std::atomic_bool running_{false};
  std::atomic_bool connected_{false};
  std::atomic_bool subscribed_{false};

  std::unique_ptr<RobotEnroller> enroller_;
  std::unique_ptr<mqtt::async_client> client_;
  std::unique_ptr<TtsClient> tts_;
  std::unique_ptr<ScreenDisplay> screen_;  // 迎宾二维码显示（脸屏）
  std::unique_ptr<QrCodeCache> qr_cache_;  // 迎宾二维码缓存/下载（P-036）
  rclcpp::Client<aimdk_msgs::srv::SetAgentSuppression>::SharedPtr suppress_client_;
  rclcpp::CallbackGroup::SharedPtr suppress_callback_group_;
  std::unique_ptr<McTransform> transform_;  // 迎宾变形（确保人形形态）
  std::unique_ptr<NavExecutor> nav_;    // MockNavExecutor（真机导航对接点）
  std::unique_ptr<TourRunner> tour_;    // 讲解模块状态机
  rclcpp::TimerBase::SharedPtr maintenance_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::thread connect_thread_;          // enroll + MQTT（重）连接驱动线程

  // 连接线程内部状态（仅该线程读写，无需加锁）
  bool credentials_ready_{false};       // 已拿到当次 enroll 凭据
  bool need_enroll_{true};              // 需要发起 enroll（首次/凭据被拒后）
  bool enroll_halted_{false};           // robot_revoked：停止 enroll 重试等运维

  // 下行队列：paho 回调线程生产 payload（topic 已在入队前过滤为本机下行主题），
  // 工作线程消费
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::string> queue_;
  std::thread worker_thread_;

  // 音频会话（仅 worker 线程访问，无锁）：tts / 迎宾 / 讲解均为单轮播完。
  // 任一新会话替换旧会话（音频单会话策略，docs/design.md §3.4）
  std::optional<PlaylistSession> session_;
  std::string session_source_;          // 会话来源（日志：tts/迎宾/讲解 operation_id）
  std::chrono::steady_clock::time_point session_next_at_;

  // message_id 有界去重表：set 查重 + deque 记 FIFO 淘汰顺序
  std::mutex dedupe_mutex_;
  std::unordered_set<std::string> dedupe_;
  std::deque<std::string> dedupe_order_;

  // operation_id -> 已发回执信封原文（含 event_id；仅 tts.result）。
  // 契约 v0.6.0：同一 operation_id 的指令重发必须重发同一回执（同 event_id），
  // 因此缓存原文原样重发；与 dedupe_ 同容量上限
  std::mutex receipt_mutex_;
  std::unordered_map<std::string, std::string> receipt_cache_;
};

}  // namespace guide_pulse
