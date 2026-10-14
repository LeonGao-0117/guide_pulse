// ============================================================================
// 屏幕二维码显示组件（迎宾场景）：
//   迎宾指令触发 -> (可选)App 下发二维码图 base64 -> 本地 ffmpeg 转
//   848x480 H.264 MP4 -> sshpass scp 推送到运控板 -> PlayEmotion(file,
//   loop_count=0) 无限循环常驻 -> 5 分钟超时或新业务到达时 cancel() 恢复
//   默认表情打断视频。
//
// 多板架构（实测结论，见 docs/design.md §屏幕显示）：
//   PlayEmotion 服务由运控板 (默认 10.1.1.100) 实际执行，file_paths 必须
//   是运控板可访问的绝对路径；本组件部署在大脑板，故先 scp 推送再播放。
//
// 线程模型：全由桥接节点 worker 线程调用（与 TtsClient 一致：服务客户端
// 挂独立回调组，worker 线程 future.wait_for 等回执，不参与节点 spin）。
// 超时不设 ROS timer：deadline_ 由桥接 worker_loop 纳入定时唤醒统一调度。
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

// 标准 base64 解码（含填充校验）。输入非法返回 false。纯函数，独立单测。
bool decode_base64(const std::string & in, std::string & out);

// 拼接 ffmpeg 转码命令行（qrcode -> 848x480 H.264 MP4，静态图 1s@10fps，
// 短视频+播放端无限循环，实测转码约 0.8s）。纯函数，独立单测。
std::string build_ffmpeg_command(
  const std::string & ffmpeg_bin, const std::string & png_path,
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
    std::string local_tmp_dir{"/tmp/guide_pulse"};  // PNG/MP4 本地暂存
    // ---- 行为参数 ----
    int request_timeout_seconds{5};           // 单次 PlayEmotion 调用超时
    int show_timeout_seconds{300};            // 二维码常驻时长（默认 5 分钟）
    int restore_emotion_id{90};               // 取消显示后恢复的默认表情
    int max_png_bytes{2 * 1024 * 1024};       // base64 解码后图片上限（防御）
  };

  struct Result
  {
    bool ok{false};
    bool service_failed{false};   // PlayEmotion 不可达/超时（区别于转换/推送失败）
    std::string detail;           // 失败描述
  };

  // node: 所属节点（桥接节点）；config: 见上
  ScreenDisplay(rclcpp::Node::SharedPtr node, Config config);

  // 显示二维码。qrcode_png_base64 空 -> 直接播放运控板预置 MP4
  // （{remote_dir}/QRCode.mp4，由运维预部署）；非空 -> 解码落盘 ->
  // ffmpeg 转码 -> scp 推送 -> PlayEmotion 无限循环。成功后 active()=true
  // 且 deadline() 生效。失败不改变当前显示状态。
  Result show(const std::string & qrcode_png_base64, const std::string & source);

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
  // scp 推送 mp4 到运控板 remote_dir（system 子进程 sshpass）
  bool push_to_remote(const std::string & mp4_path, std::string & error);
  // 运控板上 MP4 的绝对路径（PlayEmotion file_paths 用）
  std::string remote_mp4_path() const;

  rclcpp::Node::SharedPtr node_;
  Config config_;
  bool showing_{false};                              // 仅 worker 线程访问
  std::chrono::steady_clock::time_point deadline_;   // 仅 worker 线程访问
  rclcpp::Client<aimdk_msgs::srv::PlayEmotion>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace guide_pulse
