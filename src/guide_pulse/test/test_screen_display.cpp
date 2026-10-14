// ============================================================================
// 屏幕显示组件纯函数单测：base64 解码（RFC 4648，含填充与非法输入）与
// ffmpeg 转码命令拼装。服务调用/ssh 推送链路为真机集成路径，不在此覆盖。
// ============================================================================
#include <gtest/gtest.h>

#include <string>

#include "guide_pulse/screen_display.hpp"

namespace
{

// "Man" -> "TWFu"（经典向量）；"Ma" -> "TWE="；"M" -> "TQ=="
TEST(ScreenDisplayBase64, RoundTripKnownVectors)
{
  std::string out;
  ASSERT_TRUE(guide_pulse::decode_base64("TWFu", out));
  EXPECT_EQ(out, "Man");
  ASSERT_TRUE(guide_pulse::decode_base64("TWE=", out));
  EXPECT_EQ(out, "Ma");
  ASSERT_TRUE(guide_pulse::decode_base64("TQ==", out));
  EXPECT_EQ(out, "M");
}

TEST(ScreenDisplayBase64, DecodesBinaryPngHeader)
{
  // PNG 魔数头 \x89PNG\r\n\x1a\n 的 base64（iVBORw0KGgo=，标准库生成）
  std::string out;
  ASSERT_TRUE(guide_pulse::decode_base64("iVBORw0KGgo=", out));
  ASSERT_EQ(out.size(), 8U);
  EXPECT_EQ(static_cast<unsigned char>(out[0]), 0x89);
  EXPECT_EQ(out.substr(1, 3), "PNG");
}

TEST(ScreenDisplayBase64, RejectsInvalidInput)
{
  std::string out;
  EXPECT_FALSE(guide_pulse::decode_base64("", out));          // 空
  EXPECT_FALSE(guide_pulse::decode_base64("TW Fu", out));     // 含空格
  EXPECT_FALSE(guide_pulse::decode_base64("TWF", out));       // 长度非 4 倍数
  EXPECT_FALSE(guide_pulse::decode_base64("TW==Fu", out));    // '=' 后出现数据
  EXPECT_FALSE(guide_pulse::decode_base64("T===", out));      // 3 个填充
  EXPECT_FALSE(guide_pulse::decode_base64("====", out));      // 全填充
  EXPECT_FALSE(guide_pulse::decode_base64("TW=u", out));      // 尾部混入非法字符
}

TEST(ScreenDisplayFfmpeg, CommandContainsKeyParams)
{
  const auto cmd = guide_pulse::build_ffmpeg_command(
    "ffmpeg", "/tmp/in.png", "/tmp/out.mp4");
  // 关键参数齐备：输入帧率/时长/画幅 848x480/H.264/yuv420p/faststart
  EXPECT_NE(cmd.find("-framerate 10"), std::string::npos);
  EXPECT_NE(cmd.find("-t 1"), std::string::npos);
  EXPECT_NE(cmd.find("pad=848:480"), std::string::npos);
  EXPECT_NE(cmd.find("libx264"), std::string::npos);
  EXPECT_NE(cmd.find("format=yuv420p"), std::string::npos);
  EXPECT_NE(cmd.find("+faststart"), std::string::npos);
  EXPECT_NE(cmd.find("/tmp/in.png"), std::string::npos);
  EXPECT_NE(cmd.find("/tmp/out.mp4"), std::string::npos);
}

}  // namespace
