// ============================================================================
// 迎宾模块单元测试（纯 C++，无 ROS 依赖）
// 覆盖：指令 texts 优先、参数默认词回退、双空兜底（迎宾不哑火）。
// ============================================================================
#include <gtest/gtest.h>

#include "guide_pulse/welcome.hpp"

namespace
{

using guide_pulse::make_welcome_texts;

TEST(WelcomeTest, CommandTextsTakePriority)
{
  // 协议 v0.20.0：App 指定迎宾词，Server 原样转发——优先使用
  const auto texts = make_welcome_texts({"您好，欢迎参观"}, {"默认词"});
  EXPECT_EQ(texts, (std::vector<std::string>{"您好，欢迎参观"}));
}

TEST(WelcomeTest, FallsBackToDefaultTexts)
{
  // scene 未带 texts：回落本地默认迎宾词
  const auto texts = make_welcome_texts({}, {"默认迎宾词一", "默认迎宾词二"});
  EXPECT_EQ(texts, (std::vector<std::string>{"默认迎宾词一", "默认迎宾词二"}));
}

TEST(WelcomeTest, NeverReturnsEmpty)
{
  // 配置防御：指令与默认词都空时兜底内置词（迎宾至少播一条）
  const auto texts = make_welcome_texts({}, {});
  EXPECT_FALSE(texts.empty());
  EXPECT_EQ(texts, guide_pulse::builtin_welcome_texts());
}

}  // namespace
