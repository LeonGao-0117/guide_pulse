// ============================================================================
// 屏幕二维码显示组件实现（接口说明见 screen_display.hpp 文件头）
// ============================================================================
#include "guide_pulse/screen_display.hpp"

#include <array>
#include <cstdlib>
#include <fstream>

namespace guide_pulse
{

namespace
{
// base64 字符表与逆表（RFC 4648 标准 alphabet）
constexpr char kAlphabet[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int decode_char(char c)
{
  if (c >= 'A' && c <= 'Z') {
    return c - 'A';
  }
  if (c >= 'a' && c <= 'z') {
    return c - 'a' + 26;
  }
  if (c >= '0' && c <= '9') {
    return c - '0' + 52;
  }
  if (c == '+') {
    return 62;
  }
  if (c == '/') {
    return 63;
  }
  return -1;
}
}  // namespace

bool decode_base64(const std::string & in, std::string & out)
{
  out.clear();
  // 允许尾部 0-2 个 '=' 填充，总长须为 4 的倍数且非空
  if (in.empty() || in.size() % 4U != 0U) {
    return false;
  }
  out.reserve(in.size() / 4U * 3U);
  std::uint32_t buffer = 0;
  int bits = 0;
  std::size_t payload = 0;   // 非 '=' 字符数
  std::size_t padding = 0;   // '=' 数量（只能连续出现在尾部）
  for (const char c : in) {
    if (c == '=') {
      padding++;
      continue;
    }
    if (padding > 0U) {
      out.clear();  // '=' 后再出现数据字符：非法
      return false;
    }
    const int value = decode_char(c);
    if (value < 0) {
      out.clear();
      return false;
    }
    buffer = (buffer << 6) | static_cast<std::uint32_t>(value);
    bits += 6;
    payload++;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<char>((buffer >> bits) & 0xFFU));
    }
  }
  // 填充合法性：0 填充 payload%4==0；1 填充 ==3；2 填充 ==2；其余非法
  const std::size_t tail = payload % 4U;
  const bool padding_ok =
    payload != 0U &&
    ((padding == 0U && tail == 0U) || (padding == 1U && tail == 3U) ||
    (padding == 2U && tail == 2U));
  if (!padding_ok) {
    out.clear();
    return false;
  }
  return true;
}

std::string build_ffmpeg_command(
  const std::string & ffmpeg_bin, const std::string & png_path,
  const std::string & mp4_path)
{
  // 848x480 为 T1 脸屏原生画幅（display_ui 内置表情同款，其他画幅会被拉伸）；
  // 图标最大边 441（短边 92%）居中防圆屏裁角；1s@10fps 静态图视觉无差，
  // 转码约 0.8s；yuv420p+faststart 为嵌入式播放端最兼容组合。
  return ffmpeg_bin + " -y -loglevel error -loop 1 -framerate 10 -i '" +
         png_path + "' -t 1 -vf \"scale='min(iw,441)':'min(ih,441)':"
         "force_original_aspect_ratio=decrease,"
         "pad=848:480:(ow-iw)/2:(oh-ih)/2:black,format=yuv420p\" "
         "-c:v libx264 -preset veryfast -crf 23 -movflags +faststart -an '" +
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

std::string ScreenDisplay::remote_mp4_path() const
{
  // remote_dir 末尾去重斜杠后拼文件名
  const std::string & dir = config_.remote_dir;
  return (dir.back() == '/' ? dir.substr(0, dir.size() - 1U) : dir) + "/QRCode.mp4";
}

bool ScreenDisplay::push_to_remote(const std::string & mp4_path, std::string & error)
{
  // 空密码防御：sshpass -p '' 行为不可控，直接拒绝
  if (config_.remote_pass.empty()) {
    error = "remote_pass is empty (required for scp push)";
    return false;
  }
  const std::string ssh_opts = "-o StrictHostKeyChecking=no -o ConnectTimeout=5";
  // 1) 建目录（幂等）2) 推送
  const std::string mkdir_cmd = config_.sshpass_bin + " -p '" + config_.remote_pass +
    "' ssh " + ssh_opts + " " + config_.remote_user + "@" + config_.remote_host +
    " mkdir -p '" + config_.remote_dir + "'";
  const std::string scp_cmd = config_.sshpass_bin + " -p '" + config_.remote_pass +
    "' scp " + ssh_opts + " '" + mp4_path + "' " + config_.remote_user + "@" +
    config_.remote_host + ":'" + config_.remote_dir + "/QRCode.mp4'";
  if (std::system(mkdir_cmd.c_str()) != 0) {
    error = "ssh mkdir failed on " + config_.remote_host;
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
  const std::string & qrcode_png_base64, const std::string & source)
{
  Result result;
  // ---- 1. 确定运控板可播放的 MP4：下发 base64 则转换推送，否则用预置 ----
  if (!qrcode_png_base64.empty()) {
    std::string png;
    if (!decode_base64(qrcode_png_base64, png)) {
      result.detail = "qrcode_png_base64 is not valid base64";
      return result;
    }
    if (png.empty() ||
      static_cast<int>(png.size()) > config_.max_png_bytes)
    {
      result.detail = "qrcode png size out of range (max " +
                      std::to_string(config_.max_png_bytes) + " bytes)";
      return result;
    }
    const std::string png_path = config_.local_tmp_dir + "/qrcode.png";
    const std::string mp4_path = config_.local_tmp_dir + "/QRCode.mp4";
    // 简化目录创建：依赖 shell（下一条 system 命令天然可用）
    const std::string mkdir_local = "mkdir -p '" + config_.local_tmp_dir + "'";
    if (std::system(mkdir_local.c_str()) != 0) {
      result.detail = "cannot create local tmp dir " + config_.local_tmp_dir;
      return result;
    }
    std::ofstream file(png_path, std::ios::binary | std::ios::trunc);
    if (!file) {
      result.detail = "cannot write " + png_path;
      return result;
    }
    file.write(png.data(), static_cast<std::streamsize>(png.size()));
    file.close();
    const std::string ffmpeg_cmd = build_ffmpeg_command(
      config_.ffmpeg_bin, png_path, mp4_path);
    if (std::system(ffmpeg_cmd.c_str()) != 0) {
      result.detail = "ffmpeg convert failed (see command log)";
      return result;
    }
    if (!push_to_remote(mp4_path, result.detail)) {
      return result;
    }
  }
  // ---- 2. 无限循环播放（loop_count=0 常驻，直至超时/新业务打断）----
  const auto play = call_play_emotion("file", remote_mp4_path(), {}, 0);
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
