// ============================================================================
// 屏幕显示组件纯函数单测：ffmpeg 转码命令拼装（v0.22.0 P-036 起二维码
// 图片由 QrCodeCache 落盘提供，base64 通道已删除；base64 旧用例随实现
// 一并移除）。服务调用/ssh 推送链路为真机集成路径，不在此覆盖。
// ============================================================================
#include <gtest/gtest.h>

#include <algorithm>
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

TEST(ScreenDisplayFfmpeg, KeepSizeSkipsIconScale)
{
  // keep_size=true（文字图，2026-11-04 现场故障修复）：输入已是精确
  // 848x480 满屏排版，跳过 441 缩放/pad（否则上屏文字被缩小一半），
  // format/loop 首帧复制保持
  const auto cmd = guide_pulse::build_ffmpeg_command(
    "ffmpeg", "/tmp/guide_pulse/DisplayText.png", "/tmp/out.mp4",
    /*keep_size=*/true);
  EXPECT_EQ(cmd.find("scale="), std::string::npos);
  EXPECT_EQ(cmd.find("pad=848:480"), std::string::npos);
  EXPECT_NE(cmd.find("format=yuv420p"), std::string::npos);
  EXPECT_NE(cmd.find("loop=loop=-1:size=1"), std::string::npos);
  // 默认（二维码）不受影响：仍走 441 图标缩放
  const auto qr_cmd = guide_pulse::build_ffmpeg_command(
    "ffmpeg", "/tmp/qr.png", "/tmp/out.mp4");
  EXPECT_NE(qr_cmd.find("min(iw,441)"), std::string::npos);
  EXPECT_NE(qr_cmd.find("pad=848:480"), std::string::npos);
}

TEST(ScreenDisplayFfmpeg, AcceptsJpegInputPath)
{
  // 输入支持 JPEG（ffmpeg 按内容探测，命令行形状与 PNG 一致）
  const auto cmd = guide_pulse::build_ffmpeg_command(
    "ffmpeg", "/tmp/qr_cache/qr_abc123.jpg", "/tmp/out.mp4");
  EXPECT_NE(cmd.find("/tmp/qr_cache/qr_abc123.jpg"), std::string::npos);
}

// ---------------------------------------------------------------------------
// 屏幕文字显示纯函数（2026-11-04 现场需求：navigate 分流 name=display
// 的映射表 text 上屏——布局/折行/drawtext 命令拼装）
// ---------------------------------------------------------------------------

TEST(ScreenDisplayText, LayoutPicksLargeFontForShortStatus)
{
  // 短状态词（「正在通话中」5 字）：最大字号档（2026-11-04 现场反馈
  // 96 号偏小改大 -> 144 号巨幅档，5 字/行 2 行内）
  const auto layout = guide_pulse::pick_text_layout(5);
  EXPECT_EQ(layout.fontsize, 144);
  EXPECT_GE(layout.chars_per_line, 5);  // 单行放得下
  // 档位单调：字数越多字号不增
  const auto mid = guide_pulse::pick_text_layout(70);
  const auto large = guide_pulse::pick_text_layout(10);
  EXPECT_LE(mid.fontsize, large.fontsize);
  // 超长文本（>308 码点）落最小档（28 号），由 wrap 截断兜底
  const auto overflow = guide_pulse::pick_text_layout(2048);
  EXPECT_EQ(overflow.fontsize, 28);
  EXPECT_GT(overflow.max_lines, 0);
}

TEST(ScreenDisplayText, LayoutCapacityMatchesPick)
{
  // 每档容量 = 每行字数 × 最多行数：档内字数（含边界）均选该档
  //（144 号档容量 5×2=10：10 字选 144，11 字降档到 96；96 号档容量
  // 8×3=24：24 字选 96，25 字再降档）
  EXPECT_EQ(guide_pulse::pick_text_layout(10).fontsize, 144);
  EXPECT_LT(guide_pulse::pick_text_layout(11).fontsize, 144);
  EXPECT_EQ(guide_pulse::pick_text_layout(24).fontsize, 96);
  EXPECT_LT(guide_pulse::pick_text_layout(25).fontsize, 96);
}

TEST(ScreenDisplayText, WrapInsertsNewlinesByCodepoints)
{
  // 12 个中文（UTF-8 每字 3 字节）按每行 5 字折 -> 3 行（5+5+2），
  // 多字节字符不被截半（结果仍可按码点数对齐）
  const std::string text = "一二三四五六七八九十甲乙";
  const auto wrapped = guide_pulse::wrap_text_for_screen(text, 5, 10);
  EXPECT_EQ(wrapped, std::string("一二三四五\n六七八九十\n甲乙"));
  // 行数与换行符个数一致（3 行 = 2 个 \n）
  EXPECT_EQ(std::count(wrapped.begin(), wrapped.end(), '\n'), 2);
}

TEST(ScreenDisplayText, WrapNormalizesOriginalNewlines)
{
  // 原文换行/回车归一为空格（排版换行权归 wrap：格式可控）
  const auto wrapped = guide_pulse::wrap_text_for_screen("甲\n乙\r丙", 10, 5);
  EXPECT_EQ(wrapped, std::string("甲 乙 丙"));
  EXPECT_EQ(wrapped.find('\n'), std::string::npos);
}

TEST(ScreenDisplayText, WrapTruncatesWithEllipsisOnOverflow)
{
  // 超行数截断：末尾以省略号（U+2026，UTF-8 \xE2\x80\xA6）收尾
  const std::string text = "一二三四五六七八九十";
  const auto wrapped = guide_pulse::wrap_text_for_screen(text, 4, 2);
  // 2 行 × 4 字 = 8 字装下，第 9 字起截断 ->「一二三四\n五六七八…」
  EXPECT_EQ(wrapped, std::string("一二三四\n五六七八\xE2\x80\xA6"));
  // 省略号是完整码点（不截半多字节字符：倒数第 4 字节起恰为 U+2026）
  EXPECT_NE(wrapped.find("\xE2\x80\xA6"), std::string::npos);
}

TEST(ScreenDisplayText, WrapHandlesEmptyAndDegenerateParams)
{
  // 空文本/非法参数返回空串（防御）
  EXPECT_TRUE(guide_pulse::wrap_text_for_screen("", 5, 5).empty());
  EXPECT_TRUE(guide_pulse::wrap_text_for_screen("一二三", 0, 5).empty());
  EXPECT_TRUE(guide_pulse::wrap_text_for_screen("一二三", 5, 0).empty());
}

TEST(ScreenDisplayText, BuildTextImageCommandContainsKeyParams)
{
  // drawtext 命令：字体/文字文件走 textfile（内容不过命令行，任意字符
  // 安全）/居中/字号/行距/单帧 PNG 输出
  const auto cmd = guide_pulse::build_text_image_command(
    "ffmpeg", "/usr/share/fonts/droid/DroidSansFallbackFull.ttf",
    "/tmp/guide_pulse/DisplayText.txt", "/tmp/guide_pulse/DisplayText.png",
    96, 12);
  EXPECT_NE(cmd.find("drawtext"), std::string::npos);
  EXPECT_NE(cmd.find("fontfile='/usr/share/fonts/droid/DroidSansFallbackFull.ttf'"),
    std::string::npos);
  EXPECT_NE(cmd.find("textfile='/tmp/guide_pulse/DisplayText.txt'"), std::string::npos);
  EXPECT_NE(cmd.find("fontsize=96"), std::string::npos);
  EXPECT_NE(cmd.find("line_spacing=12"), std::string::npos);
  EXPECT_NE(cmd.find("x=(w-text_w)/2"), std::string::npos);
  EXPECT_NE(cmd.find("y=(h-text_h)/2"), std::string::npos);
  EXPECT_NE(cmd.find("color=c=black:s=848x480"), std::string::npos);
  EXPECT_NE(cmd.find("-frames:v 1"), std::string::npos);
  EXPECT_NE(cmd.find("'/tmp/guide_pulse/DisplayText.png'"), std::string::npos);
  // 文字内容不得以 text= 内联参数出现（转义面：引号/冒号/百分号不可控；
  // 注意 "drawtext=" 自身含 "text=" 子串，故按参数分隔形态 ":text=" 断言）
  EXPECT_EQ(cmd.find(":text="), std::string::npos);
}

}  // namespace
