// ============================================================================
// 屏幕二维码显示组件（迎宾场景），两阶段（迎宾时序：上屏必须在变形
// 完成之后、TTS 开播之前——变形动画/结束表情会顶掉变形前上屏的内容，
// PlayEmotion 单通道新请求即打断）：
//   迎宾指令触发 -> QrCodeCache 提供本地二维码图片（缓存/下载见
//   qr_code_cache.hpp）-> prepare()：本地 ffmpeg 转 848x480 H.264 MP4
//   + sshpass scp 推送到运控板（耗时大头，与迎宾变形并行重叠）->
//   play_prepared()：PlayEmotion(file, loop_count=0) 无限循环常驻 ->
//   3 分钟超时或新业务到达时 cancel() 恢复默认表情打断视频。
//
// 多板架构（实测结论，见 docs/design.md §屏幕显示）：
//   PlayEmotion 服务由运控板 (默认 10.1.1.100) 实际执行，file_paths 必须
//   是运控板可访问的绝对路径；本组件部署在大脑板，故先 scp 推送再播放。
//
// 线程模型：play_prepared/cancel/active/deadline 由桥接节点 worker 线程
// 调用（与 TtsClient 一致：服务客户端挂独立回调组，worker 线程
// future.wait_for 等回执，不参与节点 spin）。prepare 例外：迎宾流程
//（handle_scene）把它放到阶段 1 后台线程与引导语/变形并行——全程
// subprocess（ffmpeg/scp）不触碰 ROS 客户端，其写入的 prepared_/
// prepared_content_id_ 由 worker 经 future.get() 收割后消费（跨线程
// happens-before 由 future 建立）。超时不设 ROS timer：deadline_ 由
// 桥接 worker_loop 纳入定时唤醒统一调度。
// ============================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <aimdk_msgs/srv/play_emotion.hpp>

namespace guide_pulse
{

// 拼接 ffmpeg 转码命令行（二维码图片(PNG/JPEG，ffmpeg 自动识别) ->
// 848x480 H.264 MP4，静态图 1s@10fps，短视频+播放端无限循环，实测转码
// 约 0.8s）。纯函数，独立单测。
std::string build_ffmpeg_command(
  const std::string & ffmpeg_bin, const std::string & image_path,
  const std::string & mp4_path);

class ScreenDisplay
{
public:
  struct Config
  {
    std::string emotion_service_name{"/aimdk_5Fmsgs/srv/PlayEmotion"};  // SDK 固定
    // ---- 运控板推送（PlayEmotion 服务执行板）----
    std::string remote_host{"10.1.1.100"};
    std::string remote_user{"run"};
    std::string remote_pass;                  // 必填：scp 推送用（设备内网）
    std::string remote_dir{"/home/run/screen_display"};
    std::string sshpass_bin{"sshpass"};
    std::string ffmpeg_bin{"ffmpeg"};
    std::string local_tmp_dir{"/tmp/guide_pulse"};  // MP4 本地暂存
    // ---- 行为参数 ----
    int request_timeout_seconds{5};           // 单次 PlayEmotion 调用超时
    int show_timeout_seconds{180};            // 二维码常驻时长（默认 3 分钟，
                                               // 与迎宾流程超时独立计时互不影响）
    int restore_emotion_id{90};               // 取消显示后恢复的默认表情
  };

  struct Result
  {
    bool ok{false};
    bool service_failed{false};   // PlayEmotion 不可达/超时（区别于转换/推送失败）
    std::string detail;           // 失败描述
  };

  // node: 所属节点（桥接节点）；config: 见上
  ScreenDisplay(rclcpp::Node::SharedPtr node, Config config);

  // ---- 两阶段显示（迎宾时序用：上屏必须在变形完成之后、TTS 开播
  // 之前——变形动画/结束表情会顶掉变形前上屏的内容，PlayEmotion 单
  // 通道新请求即打断；下载/转码/推送是耗时大头，仍可与变形并行）----

  // 阶段 1（可与迎宾变形并行重叠）：ffmpeg 转码 + scp 推送运控板。
  // ok=true 表示内容就绪（可 play_prepared）；失败清除就绪状态并带
  // detail 原因（本次不显示，不影响后续重试）
  Result prepare(
    const std::string & image_path, const std::string & content_id = "");

  // 阶段 2（变形完成后、TTS 开播前调用）：PlayEmotion 无限循环上屏。
  // 无就绪内容（prepare 未成功/已被消费）时返回失败，不改变显示状态。
  // 成功后 active()=true 且 deadline() 生效
  Result play_prepared(const std::string & source);

  // 取消显示（超时/新业务到达）：恢复默认表情打断循环视频。
  // 未显示时为空操作（幂等）。
  Result cancel(const std::string & reason);

  // 当前是否处于"二维码常驻"状态；deadline() 为超时时刻（worker 唤醒用）
  bool active() const;
  std::chrono::steady_clock::time_point deadline() const;

private:
  // 调 PlayEmotion：file 模式（loop_count<=0 无限循环）或 emotion 恢复
  Result call_play_emotion(
    const std::string & type, const std::string & file_path,
    const std::vector<int32_t> & emotion_ids, int32_t loop_count);
  // scp 推送 mp4 到运控板 remote_dir（system 子进程 sshpass）。
  // content_id 非空时推 QRCode_{id}.mp4 并清掉旧 QRCode*.mp4（防堆积；
  // rm 正在播放的旧文件无碍：播放器持有 fd，且新路径播放即打断）
  bool push_to_remote(const std::string & mp4_path, const std::string & content_id,
    std::string & error);
  // 运控板上 MP4 的绝对路径（PlayEmotion file_paths 用）：
  // content_id 非空 -> QRCode_{id}.mp4；空 -> QRCode.mp4（兼容旧行为）
  std::string remote_mp4_path(const std::string & content_id) const;

  rclcpp::Node::SharedPtr node_;
  Config config_;
  bool showing_{false};                              // 仅 worker 线程访问
  std::chrono::steady_clock::time_point deadline_;   // 仅 worker 线程访问
  bool prepared_{false};          // 待上屏标记（迎宾后台准备线程写，worker 读）
  std::string prepared_content_id_;  // 就绪内容的寻址文件名（QRCode_{id}.mp4）
                                   // 同上：经 future.get() 收割后消费
  rclcpp::Client<aimdk_msgs::srv::PlayEmotion>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace guide_pulse
