// ============================================================================
// 屏幕二维码显示组件实现（接口说明见 screen_display.hpp 文件头）
// ============================================================================
#include "guide_pulse/screen_display.hpp"

#include <cstdlib>
#include <fstream>

namespace guide_pulse
{

// UTF-8 码点计数（与 guide_protocol.cpp / point_map.cpp 同规则：
// 非续字节即码点起点——文字布局按码点计，中文/emoji 与字节解耦）
namespace
{
std::size_t utf8_scalar_count(const std::string & value)
{
  std::size_t count = 0;
  for (const unsigned char ch : value) {
    if ((ch & 0xC0U) != 0x80U) {
      ++count;  // 非续字节（10xxxxxx）即一个码点的起点
    }
  }
  return count;
}
}  // namespace

std::string build_ffmpeg_command(
  const std::string & ffmpeg_bin, const std::string & image_path,
  const std::string & mp4_path, bool keep_size)
{
  // 848x480 为 T1 脸屏原生画幅（display_ui 内置表情同款，其他画幅会被拉伸）；
  // 1s@10fps 静态图视觉无差，转码约 0.8s；yuv420p+faststart 为嵌入式
  // 播放端最兼容组合。
  // 输入支持 PNG/JPEG 及其变体（ffmpeg 按内容自动探测）。
  // 首帧复制用 loop「滤镜」（libavfilter 标准能力）而非 -loop 1 输入选项：
  // 后者是 image2 demuxer 私有选项，部分图片字节（如 App 上传的某些
  // JPEG/PNG 变体）会让 ffmpeg 内容探测选中 *_pipe demuxer 而不支持它，
  // 真机部分图片会报 "Option loop not found"；帧率同理用输出侧 -r。
  // keep_size=true（文字图）：输入已是精确 848x480 满屏排版，跳过
  // scale/pad 原尺寸直转（2026-11-04 现场故障修复：441 缩放使上屏
  // 文字缩小一半，是「文字太小」的真正根源）
  const std::string scale_vf =
    keep_size
    ? std::string("format=yuv420p,loop=loop=-1:size=1:start=0")
    : std::string(
      "scale='min(iw,441)':'min(ih,441)':"
      "force_original_aspect_ratio=decrease,"
      "pad=848:480:(ow-iw)/2:(oh-ih)/2:black,format=yuv420p,"
      "loop=loop=-1:size=1:start=0");
  return ffmpeg_bin + " -y -loglevel error -i '" + image_path +
         "' -t 1 -vf \"" + scale_vf + "\" "
         "-r 10 -c:v libx264 -preset veryfast -crf 23 -movflags +faststart -an '" +
         mp4_path + "'";
}

TextLayoutParams pick_text_layout(std::size_t char_count)
{
  // 848x480 脸屏排版（黑底白字，四周各留约 30px 安全边）：
  //   可用宽度 ~790px，CJK 方块字单字宽≈字号 -> 每行字数 = 790/字号
  //   行高 = 字号×1.42（含 drawtext line_spacing 12）-> 最多行数 = 440/行高
  // 字号从大到小试到放得下为止：短状态词（如「正在通话中」5 字）用
  // 144 号巨幅醒目（2026-11-04 现场反馈 96 号偏小改大，约占屏高
  // 30%），长迎宾词自动缩字号多行。映射表 text 契约上限 2048 码点，
  // 最小档也放不下时（>308 字）由 wrap 截断加省略号兜底
  struct Candidate
  {
    int fontsize;
    int chars_per_line;
    int max_lines;
  };
  static const Candidate candidates[] = {
    {144, 5, 2}, {96, 8, 3}, {72, 10, 4}, {56, 14, 5},
    {48, 16, 6}, {40, 19, 7}, {34, 23, 9}, {28, 28, 11},
  };
  for (const auto & c : candidates) {
    if (char_count <= static_cast<std::size_t>(c.chars_per_line) * c.max_lines) {
      return {c.fontsize, c.chars_per_line, c.max_lines};
    }
  }
  // 超长文本（>308 码点）：最小字号 + 截断兜底（wrap 内处理）
  const auto & smallest = candidates[sizeof(candidates) / sizeof(candidates[0]) - 1];
  return {smallest.fontsize, smallest.chars_per_line, smallest.max_lines};
}

std::string wrap_text_for_screen(
  const std::string & text, int chars_per_line, int max_lines)
{
  if (text.empty() || chars_per_line <= 0 || max_lines <= 0) {
    return {};
  }
  // 逐码点扫描折行（保证多字节字符不被截半；换行符不占折行宽度，
  // 原文自带换行先归一成空格——drawtext textfile 内真实换行只由本
  // 函数控制，排版才可控）
  std::string wrapped;
  int line_chars = 0;   // 当前行已放码点数
  int lines = 1;        // 当前行号（1 起）
  bool overflowed = false;
  std::size_t i = 0;
  while (i < text.size()) {
    // 解码一个 UTF-8 码点（容错：非法续字节按单字节处理，不中断）
    std::size_t len = 1;
    const unsigned char lead = static_cast<unsigned char>(text[i]);
    if ((lead & 0xE0U) == 0xC0U) {
      len = 2;
    } else if ((lead & 0xF0U) == 0xE0U) {
      len = 3;
    } else if ((lead & 0xF8U) == 0xF0U) {
      len = 4;
    }
    len = std::min(len, text.size() - i);
    const char ch = text[i];
    // 原文换行/回车归一为空格（排版换行权归本函数）
    const bool is_newline = (ch == '\n' || ch == '\r');
    if (!is_newline) {
      if (line_chars >= chars_per_line) {
        if (lines >= max_lines) {
          overflowed = true;  // 行数与行宽同时到顶：截断
          break;
        }
        wrapped += '\n';
        ++lines;
        line_chars = 0;
      }
      wrapped.append(text, i, len);
      ++line_chars;
    } else {
      wrapped += ' ';
      ++line_chars;
    }
    i += len;
  }
  if (overflowed) {
    // 截断收尾：直接追加省略号（U+2026，UTF-8 三字节）让现场看到
    // 「后面还有内容」。不回退末字：chars_per_line 按 ~790px 可用宽
    // 计算而屏宽 848px，多一个省略号（≈1 字宽）仍在安全边内不会被裁
    wrapped += "\xE2\x80\xA6";  // U+2026 省略号
  }
  return wrapped;
}

std::string build_text_image_command(
  const std::string & ffmpeg_bin, const std::string & font_path,
  const std::string & textfile_path, const std::string & png_path,
  int fontsize, int line_spacing)
{
  // 黑底 848x480 单帧 -> drawtext 居中渲染 -> PNG。文字内容经 textfile
  // 文件传入（drawtext 原生支持，按原始字节读取，换行生效）：映射表
  // text 可含引号/冒号/百分号等任意字符，走文件面彻底绕开命令行与
  // 滤镜参数两层转义（text= 内联方式对 ' : % \ 都要转义，真机不可控）。
  // fontfile 指定中文字体（默认 Droid Sans Fallback，覆盖 CJK）。
  // x/y=(w-text_w)/2、(h-text_h)/2 = 单行/多行整体水平垂直居中
  return ffmpeg_bin + " -y -loglevel error -f lavfi -i color=c=black:s=848x480:d=1 "
         "-vf \"drawtext=fontfile='" + font_path +
         "':textfile='" + textfile_path +
         "':fontcolor=white:fontsize=" + std::to_string(fontsize) +
         ":line_spacing=" + std::to_string(line_spacing) +
         ":x=(w-text_w)/2:y=(h-text_h)/2\" -frames:v 1 '" + png_path + "'";
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
  const std::vector<int32_t> & emotion_ids, int32_t loop_count, int32_t priority)
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
  request->priority = priority;
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

ScreenDisplay::Result ScreenDisplay::prepare(
  const std::string & image_path, const std::string & content_id, bool keep_size)
{
  prepared_ = false;  // 每次准备重置：失败即无就绪内容（play_prepared 拒绝）
  Result result;
  // ---- 二维码图片转 848x480 MP4 并推送运控板（耗时大头，可与变形并行）----
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
    config_.ffmpeg_bin, image_path, mp4_path, keep_size);
  if (std::system(ffmpeg_cmd.c_str()) != 0) {
    result.detail = "ffmpeg convert failed (see command log)";
    return result;
  }
  if (!push_to_remote(mp4_path, content_id, result.detail)) {
    return result;
  }
  prepared_ = true;
  prepared_content_id_ = content_id;
  result.ok = true;  // 就绪（play_prepared 才真正上屏）
  return result;
}

ScreenDisplay::Result ScreenDisplay::prepare_text(
  const std::string & text, const std::string & content_id)
{
  // 文字版阶段 1（2026-11-04 现场需求）：drawtext 渲染 PNG 后复用
  // 与二维码图片完全相同的 转码+推送 管线（prepare(image)）
  prepared_ = false;  // 每次准备重置：失败即无就绪内容（play_prepared 拒绝）
  Result result;
  if (text.empty()) {
    result.detail = "text is empty (nothing to render)";
    return result;
  }
  if (config_.text_font_path.empty()) {
    result.detail = "text_font_path is empty (text display disabled)";
    return result;
  }
  const std::string txt_path = config_.local_tmp_dir + "/DisplayText.txt";
  const std::string png_path = config_.local_tmp_dir + "/DisplayText.png";
  const std::string mkdir_local = "mkdir -p '" + config_.local_tmp_dir + "'";
  if (std::system(mkdir_local.c_str()) != 0) {
    result.detail = "cannot create local tmp dir " + config_.local_tmp_dir;
    return result;
  }
  // 自适应布局 + 折行后写 textfile（内容走文件面，任意字符安全）
  const auto layout = pick_text_layout(utf8_scalar_count(text));
  const std::string wrapped =
    wrap_text_for_screen(text, layout.chars_per_line, layout.max_lines);
  {
    std::ofstream txt(txt_path, std::ios::binary | std::ios::trunc);
    if (!txt.is_open() || !(txt << wrapped)) {
      result.detail = "cannot write textfile " + txt_path;
      return result;
    }
  }
  const std::string render_cmd = build_text_image_command(
    config_.ffmpeg_bin, config_.text_font_path, txt_path, png_path,
    layout.fontsize, /*line_spacing=*/12);
  if (std::system(render_cmd.c_str()) != 0) {
    result.detail = "ffmpeg drawtext render failed (font missing? " +
                    config_.text_font_path + ")";
    return result;
  }
  // PNG 就绪，走二维码图片同款推送管线（图->MP4 + scp），但保持原
  // 尺寸转码（keep_size=true）：DisplayText.png 已是精确 848x480 满屏
  // 排版，二维码管线的 441 缩放会把文字缩小一半（2026-11-04 现场
  // 故障修复）
  return prepare(png_path, content_id, /*keep_size=*/true);
}

ScreenDisplay::Result ScreenDisplay::play_prepared(
  const std::string & source, int timeout_seconds)
{
  Result result;
  if (!prepared_) {
    result.detail = "nothing prepared (prepare failed or not called)";
    return result;
  }
  prepared_ = false;  // 一次性消费：重复调用不再上屏
  // ---- 无限循环播放（loop_count=0 常驻，直至超时/新业务打断）----
  // 内容寻址文件名：换图必是新路径，运控板播放器被迫重新加载
  //（固定路径覆盖重播时播放器不重读文件——真机踩坑）
  const auto play = call_play_emotion(
    "file", remote_mp4_path(prepared_content_id_), {}, 0);
  if (!play.ok) {
    return play;
  }
  showing_ = true;
  // 常驻时长：本次覆盖值（>0）优先，否则用配置默认（二维码 2 分钟；
  // 文字显示按 2026-11-04 现场需求独立传 60s）
  const int timeout = timeout_seconds > 0
    ? timeout_seconds
    : config_.show_timeout_seconds;
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::seconds(timeout);
  RCLCPP_INFO(
    node_->get_logger(),
    "屏幕内容显示: source=%s 超时=%ds 后自动恢复", source.c_str(), timeout);
  return play;
}

ScreenDisplay::Result ScreenDisplay::cancel(const std::string & reason)
{
  if (!showing_) {
    return {true, false, ""};  // 幂等：未显示时无操作
  }
  // 打断循环视频 = 播放一次默认表情（PlayEmotion 单通道，新请求即打断
  // 之外还有优先级仲裁——同优先级不打断循环视频；恢复请求须以高于
  // 上屏（0）的 restore_priority 发出，否则运控板受理 ok 但屏幕滞留
  // 二维码，2026-10-23 真机故障）
  auto result = call_play_emotion(
    "emotion", "", {static_cast<int32_t>(config_.restore_emotion_id)}, 1,
    static_cast<int32_t>(config_.restore_priority));
  if (result.ok) {
    // 只有受理成功才清常驻状态：失败保持 showing_ 并顺延 deadline
    // 5s，worker 超时分支到点自动重试撤屏——先置 false 再调用的旧
    // 顺序会在一次失败后把超时兜底与后续业务入口的 cancel 全部变
    // 成 no-op，二维码永久滞留屏幕
    showing_ = false;
    RCLCPP_INFO(
      node_->get_logger(), "屏幕内容取消(%s): ok", reason.c_str());
  } else {
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    RCLCPP_WARN(
      node_->get_logger(),
      "屏幕内容取消(%s)失败(5s后自动重试): %s", reason.c_str(),
      result.detail.c_str());
  }
  return result;
}

}  // namespace guide_pulse
