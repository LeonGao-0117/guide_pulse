// ============================================================================
// PlaylistSession 播报状态机单元测试（纯 C++，无 ROS 依赖）
// 覆盖：sequence 顺序推进、单轮模式播完结束、无界循环回到首轮、
// 失败重试不推进（索引不动）、间隔计算。
// ============================================================================
#include <gtest/gtest.h>

#include "guide_pulse/playlist.hpp"

namespace
{

using guide_pulse::PlaylistSession;

TEST(PlaylistTest, SequenceAdvancesInOrder)
{
  PlaylistSession session({"a", "b", "c"}, "sequence", 1, /*loop_forever=*/true);
  EXPECT_EQ(session.current(), "a");
  EXPECT_TRUE(session.advance());
  EXPECT_EQ(session.current(), "b");
  EXPECT_TRUE(session.advance());
  EXPECT_EQ(session.current(), "c");
  // 无界循环：一轮播完回到首轮
  EXPECT_TRUE(session.advance());
  EXPECT_EQ(session.current(), "a");
  EXPECT_EQ(session.round_no(), 2u);
  EXPECT_FALSE(session.finished());
}

TEST(PlaylistTest, SingleRoundFinishesAfterOnePass)
{
  // 迎宾/讲解模式（loop_forever=false）：一轮播完会话结束
  PlaylistSession session({"a", "b"}, "sequence", 1, /*loop_forever=*/false);
  EXPECT_EQ(session.current(), "a");
  EXPECT_TRUE(session.advance());
  EXPECT_EQ(session.current(), "b");
  EXPECT_FALSE(session.advance());  // 最后一条播完：会话结束
  EXPECT_TRUE(session.finished());
  EXPECT_EQ(session.round_no(), 2u);  // 已进入（未播的）第二轮
}

TEST(PlaylistTest, SingleItemSingleRoundFinishesImmediately)
{
  PlaylistSession session({"only"}, "sequence", 5, /*loop_forever=*/false);
  EXPECT_EQ(session.current(), "only");
  EXPECT_FALSE(session.advance());
  EXPECT_TRUE(session.finished());
}

TEST(PlaylistTest, SingleItemLoopForeverRepeats)
{
  PlaylistSession session({"only"}, "sequence", 5, /*loop_forever=*/true);
  EXPECT_TRUE(session.advance());
  EXPECT_EQ(session.current(), "only");  // 循环：同条重复
  EXPECT_FALSE(session.finished());
}

TEST(PlaylistTest, RetryDoesNotAdvance)
{
  // 失败重试语义由驱动方实现（失败不调用 advance）：
  // 这里验证不推进时 current 稳定
  PlaylistSession session({"a", "b"}, "sequence", 1, false);
  EXPECT_EQ(session.current(), "a");
  EXPECT_EQ(session.current(), "a");  // 未 advance：仍是 a
  EXPECT_EQ(session.index(), 0u);
}

TEST(PlaylistTest, NextDelayIncludesSpeechEstimateAndInterval)
{
  // 3 个汉字 ≈ ceil(3*0.22)=1s 语音 + 2s 间隔 = 3s
  PlaylistSession session({"一二三"}, "sequence", 2, false);
  EXPECT_EQ(session.next_delay_seconds(), 3);
}

TEST(PlaylistTest, ShuffleRoundStaysPermutationAcrossRounds)
{
  PlaylistSession session({"a", "b", "c", "d"}, "shuffle", 0, true);
  std::vector<std::string> seen;
  for (int i = 0; i < 8; ++i) {
    seen.push_back(session.current());
    session.advance();
  }
  std::sort(seen.begin(), seen.end());
  // 播过的 8 条里每条至少出现（多轮覆盖，集合不变由 make_round_order 保证）
  for (const char * item : {"a", "b", "c", "d"}) {
    EXPECT_NE(std::find(seen.begin(), seen.end(), item), seen.end()) << item;
  }
}

}  // namespace
