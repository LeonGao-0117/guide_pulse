// ============================================================================
// 屏幕二维码显示组件实现（接口说明见 screen_display.hpp 文件头）
// ============================================================================
#include "guide_pulse/screen_display.hpp"

#include <cstdlib>

namespace guide_pulse
{

std::string build_ffmpeg_command(
  const std::string & ffmpeg_bin, const std::string & image_path,
  const std::string & mp4_path)
{
  // 848x480 为 T1 脸屏原生画幅（display_ui 内置表情同款，其他画幅会被拉伸）；
  // 图标最大边 441（短边 92%）居中防圆屏裁角；1s@10fps 静态图视觉无差，
  // 转码约 0.8s；yuv420p+faststart 为嵌入式播放端最兼容组合。
  // 输入支持 PNG/JPEG 及其变体（ffmpeg 按内容自动探测）。
  // 首帧复制用 loop「滤镜」（libavfilter 标准能力）而非 -loop 1 输入选项：
  // 后者是 image2 demuxer 私有选项，部分图片字节（如 App 上传的某些
  // JPEG/PNG 变体）会让 ffmpeg 内容探测选中 *_pipe demuxer 而不支持它，
  // 真机部分图片会报 "Option loop not found"；帧率同理用输出侧 -r。
  return ffmpeg_bin + " -y -loglevel error -i '" + image_path +
         "' -t 1 -vf \"scale='min(iw,441)':'min(ih,441)':"
         "force_original_aspect_ratio=decrease,"
         "pad=848:480:(ow-iw)/2:(oh-ih)/2:black,format=yuv420p,"
         "loop=loop=-1:size=1:start=0\" "
         "-r 10 -c:v libx264 -preset veryfast -crf 23 -movflags +faststart -an '" +
         mp4_path + "'";
}

ScreenDisplay::ScreenDisplay(rclcpp::Node::SharedPtr node, Config config)
: node_(std::move(node)), config_(std::move(config))
{
  // 独立回调组：节点被 executor spin 时，worker 线程可阻塞等回执
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  client_ = node_->create_client<aimdk_msgs::srv::PlayEmotion>(
    config_.emotion_service_name, rmw_qos_profile_services_default, callback_group_);
}

bool ScreenDisplay::active() const
{
  return showing_;
}

std::chrono::steady_clock::time_point ScreenDisplay::deadline() const
{
  return deadline_;
}

std::string ScreenDisplay::remote_mp4_path(const std::string & content_id) const
{
  // remote_dir 末尾去重斜杠后拼文件名；content_id 带内容寻址（见头文件）
  const std::string & dir = config_.remote_dir;
  const std::string base = (dir.back() == '/' ? dir.substr(0, dir.size() - 1U) : dir);
  if (content_id.empty()) {
    return base + "/QRCode.mp4";
  }
  return base + "/QRCode_" + content_id + ".mp4";
}

bool ScreenDisplay::push_to_remote(
  const std::string & mp4_path, const std::string & content_id, std::string & error)
{
  // 空密码防御：sshpass -p '' 行为不可控，直接拒绝
  if (config_.remote_pass.empty()) {
    error = "remote_pass is empty (required for scp push)";
    return false;
  }
  const std::string ssh_opts = "-o StrictHostKeyChecking=no -o ConnectTimeout=5";
  const std::string target =
    config_.remote_user + "@" + config_.remote_host;
  const std::string remote_file = remote_mp4_path(content_id);
  // 1) 建目录（幂等）+ 清旧 QRCode*.mp4（内容寻址换名后防堆积）
  //    rm 正在循环播放的旧文件无碍：播放器已持有 fd 继续播完当前内容，
  //    且随后新路径的 PlayEmotion 会立即打断它
  // 2) 推送
  const std::string mkdir_cmd = config_.sshpass_bin + " -p '" + config_.remote_pass +
    "' ssh " + ssh_opts + " " + target + " mkdir -p '" + config_.remote_dir +
    "' && rm -f '" + config_.remote_dir + "/QRCode'*.mp4";
  const std::string scp_cmd = config_.sshpass_bin + " -p '" + config_.remote_pass +
    "' scp " + ssh_opts + " '" + mp4_path + "' " + target + ":'" + remote_file + "'";
  if (std::system(mkdir_cmd.c_str()) != 0) {
    error = "ssh mkdir/rm failed on " + config_.remote_host;
    return false;
  }
  if (std::system(scp_cmd.c_str()) != 0) {
    error = "scp push failed to " + config_.remote_host;
    return false;
  }
  return true;
}

ScreenDisplay::Result ScreenDisplay::call_play_emotion(
  const std::string & type, const std::string & file_path,
  const std::vector<int32_t> & emotion_ids, int32_t loop_count)
{
  Result result;
  if (!client_->service_is_ready()) {
    result.service_failed = true;
    result.detail = "PlayEmotion service not available: " + config_.emotion_service_name;
    return result;
  }
  auto request = std::make_shared<aimdk_msgs::srv::PlayEmotion::Request>();
  request->header.header.stamp = node_->now();
  request->type = type;
  if (!file_path.empty()) {
    request->file_paths.push_back(file_path);
  }
  request->emotion_ids = emotion_ids;
  request->priority = 0;
  request->loop_count = loop_count;

  auto future = client_->async_send_request(request);
  const auto timeout = std::chrono::seconds(config_.request_timeout_seconds);
  if (future.wait_for(timeout) != std::future_status::ready) {
    client_->remove_pending_request(future);
    result.service_failed = true;
    result.detail = "PlayEmotion request timeout (" +
                    std::to_string(config_.request_timeout_seconds) + "s)";
    return result;
  }
  const auto response = future.get();
  if (response->header.header.code == 0) {
    result.ok = true;
    return result;
  }
  result.detail = "PlayEmotion rejected: code=" +
                  std::to_string(response->header.header.code) + " msg=" +
                  response->header.message;
  return result;
}

ScreenDisplay::Result ScreenDisplay::show(
  const std::string & image_path, const std::string & source,
  const std::string & content_id)
{
  Result result;
  // ---- 1. 二维码图片转 848x480 MP4 并推送运控板 ----
  if (image_path.empty()) {
    result.detail = "image_path is empty (no qr image to show)";
    return result;
  }
  const std::string mp4_path = config_.local_tmp_dir + "/QRCode.mp4";
  // 简化目录创建：依赖 shell（下一条 system 命令天然可用）
  const std::string mkdir_local = "mkdir -p '" + config_.local_tmp_dir + "'";
  if (std::system(mkdir_local.c_str()) != 0) {
    result.detail = "cannot create local tmp dir " + config_.local_tmp_dir;
    return result;
  }
  const std::string ffmpeg_cmd = build_ffmpeg_command(
    config_.ffmpeg_bin, image_path, mp4_path);
  if (std::system(ffmpeg_cmd.c_str()) != 0) {
    result.detail = "ffmpeg convert failed (see command log)";
    return result;
  }
  if (!push_to_remote(mp4_path, content_id, result.detail)) {
    return result;
  }
  // ---- 2. 无限循环播放（loop_count=0 常驻，直至超时/新业务打断）----
  // 内容寻址文件名：换图必是新路径，运控板播放器被迫重新加载
  //（固定路径覆盖重播时播放器不重读文件——真机踩坑）
  const auto play = call_play_emotion("file", remote_mp4_path(content_id), {}, 0);
  if (!play.ok) {
    return play;
  }
  showing_ = true;
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::seconds(config_.show_timeout_seconds);
  RCLCPP_INFO(
    node_->get_logger(),
    "二维码显示: source=%s 超时=%ds 后自动恢复", source.c_str(),
    config_.show_timeout_seconds);
  return play;
}

ScreenDisplay::Result ScreenDisplay::cancel(const std::string & reason)
{
  if (!showing_) {
    return {true, false, ""};  // 幂等：未显示时无操作
  }
  showing_ = false;
  // 打断循环视频 = 播放一次默认表情（PlayEmotion 单通道，新请求即打断）
  auto result = call_play_emotion(
    "emotion", "", {static_cast<int32_t>(config_.restore_emotion_id)}, 1);
  RCLCPP_INFO(
    node_->get_logger(), "二维码取消(%s): %s", reason.c_str(),
    result.ok ? "ok" : result.detail.c_str());
  return result;
}

}  // namespace guide_pulse
