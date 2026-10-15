// ============================================================================
// 屏幕显示组件纯函数单测：ffmpeg 转码命令拼装（v0.22.0 P-036 起二维码
// 图片由 QrCodeCache 落盘提供，base64 通道已删除；base64 旧用例随实现
// 一并移除）。服务调用/ssh 推送链路为真机集成路径，不在此覆盖。
// ============================================================================
#include <gtest/gtest.h>

#include <string>

#include "guide_pulse/screen_display.hpp"

namespace
{

TEST(ScreenDisplayFfmpeg, CommandContainsKeyParams)
{
  const auto cmd = guide_pulse::build_ffmpeg_command(
    "ffmpeg", "/tmp/qr_cache/qr_abc123.png", "/tmp/out.mp4");
  // 关键参数齐备：时长/画幅 848x480/H.264/yuv420p/faststart/首帧复制滤镜/输出帧率
  EXPECT_NE(cmd.find("-t 1"), std::string::npos);
  EXPECT_NE(cmd.find("pad=848:480"), std::string::npos);
  EXPECT_NE(cmd.find("libx264"), std::string::npos);
  EXPECT_NE(cmd.find("format=yuv420p"), std::string::npos);
  EXPECT_NE(cmd.find("+faststart"), std::string::npos);
  EXPECT_NE(cmd.find("/tmp/qr_cache/qr_abc123.png"), std::string::npos);
  EXPECT_NE(cmd.find("/tmp/out.mp4"), std::string::npos);
  // 首帧复制用 loop 滤镜（对任何输入 demuxer 通用）
  EXPECT_NE(cmd.find("loop=loop=-1:size=1"), std::string::npos);
  EXPECT_NE(cmd.find("-r 10"), std::string::npos);
}

TEST(ScreenDisplayFfmpeg, AvoidsImage2DemuxerPrivateOptions)
{
  // 回归（真机故障）：-loop/-framerate 是 image2 demuxer 私有
  // 输入选项，App 上传的 JPEG 图片使 ffmpeg 内容探测选中 *_pipe demuxer
  // 而不支持它们（报 "Option loop not found"）。命令不得再依赖 demuxer
  // 私有选项
  const auto cmd = guide_pulse::build_ffmpeg_command(
    "ffmpeg", "/tmp/qr_cache/qr_abc123.jpg", "/tmp/out.mp4");
  EXPECT_EQ(cmd.find("-loop 1"), std::string::npos);
  EXPECT_EQ(cmd.find("-framerate"), std::string::npos);
  EXPECT_NE(cmd.find("-i '/tmp/qr_cache/qr_abc123.jpg'"), std::string::npos);
}

TEST(ScreenDisplayFfmpeg, AcceptsJpegInputPath)
{
  // 输入支持 JPEG（ffmpeg 按内容探测，命令行形状与 PNG 一致）
  const auto cmd = guide_pulse::build_ffmpeg_command(
    "ffmpeg", "/tmp/qr_cache/qr_abc123.jpg", "/tmp/out.mp4");
  EXPECT_NE(cmd.find("/tmp/qr_cache/qr_abc123.jpg"), std::string::npos);
}

}  // namespace
