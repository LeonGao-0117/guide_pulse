// ============================================================================
// AgentMonitor 纯函数单测：语料归一化与匹配的不变式。
//（节点订阅/回调链路属真机集成，由 docs/agent_bridge_notes.md 现场验证）
// ============================================================================
#include <gtest/gtest.h>

#include <string>

#include "guide_pulse/agent_monitor.hpp"

namespace
{

using guide_pulse::normalize_phrase_text;
using guide_pulse::phrase_matches;

// 归一化：中文标点/空白剔除，文字保留
TEST(NormalizePhraseText, RemovesCjkPunctuationAndSpaces)
{
  EXPECT_EQ(normalize_phrase_text("好嘞，马上开始迎接我的顾客喽"), "好嘞马上开始迎接我的顾客喽");
  EXPECT_EQ(normalize_phrase_text("好嘞， 马上开始迎接我的顾客喽！"), "好嘞马上开始迎接我的顾客喽");
  EXPECT_EQ(normalize_phrase_text("  好 嘞 。 马上　开始  "), "好嘞马上开始");  // 含全角空格
}

// 归一化：ASCII 字母转小写、数字保留、标点剔除
TEST(NormalizePhraseText, AsciiLowercaseAndDigits)
{
  EXPECT_EQ(normalize_phrase_text("OK, Start 3!"), "okstart3");
  EXPECT_EQ(normalize_phrase_text("Robot-Car v0.9"), "robotcarv09");
}

// 归一化：纯标点/空白归一化为空（不作为有效触发词）
TEST(NormalizePhraseText, PunctuationOnlyToEmpty)
{
  EXPECT_EQ(normalize_phrase_text("，。！？…—  "), "");
  EXPECT_EQ(normalize_phrase_text(""), "");
}

// 匹配：识别文本包含语料即命中（语气词前后夹不影响）
TEST(PhraseMatches, SubstringAfterNormalize)
{
  EXPECT_TRUE(phrase_matches("嗯，好嘞，马上开始迎接我的顾客喽哈", "好嘞，马上开始迎接我的顾客喽"));
  EXPECT_TRUE(phrase_matches("好嘞马上开始迎接我的顾客喽", "好嘞，马上开始迎接我的顾客喽！"));
  EXPECT_TRUE(phrase_matches("OK Start 3", "ok, start 3"));
}

// 匹配：不同语句/短于语料不命中（识别文本是语料子串不算，防误触发）
TEST(PhraseMatches, NoFalsePositive)
{
  EXPECT_FALSE(phrase_matches("今天天气不错", "好嘞，马上开始迎接我的顾客喽"));
  EXPECT_FALSE(phrase_matches("迎接我的顾客", "好嘞，马上开始迎接我的顾客喽"));
  EXPECT_FALSE(phrase_matches("随便说点什么", "，。！"));  // 空语料不命中
}

// 匹配：二维码话术命中（2026-10-16 语料触发二维码：agent 云端应答
// 话术经归一化后匹配，标点/空白写法差异不影响；截断话术不命中）
TEST(PhraseMatches, QrShowPhrase)
{
  const std::string phrase = "收到，这就给您显示二维码，您可以扫描二维码留下您的个人信息哦。";
  EXPECT_TRUE(phrase_matches(phrase, phrase));                       // 原句
  EXPECT_TRUE(phrase_matches(                                        // 前后夹语气词
    "嗯，收到这就给您显示二维码您可以扫描二维码留下您的个人信息哦，谢谢", phrase));
  EXPECT_TRUE(phrase_matches(                                        // 配置省略句尾标点
    "收到，这就给您显示二维码，您可以扫描二维码留下您的个人信息哦", phrase));
  EXPECT_FALSE(phrase_matches("收到，这就给您显示二维码", phrase));  // 话术截断不命中
  EXPECT_FALSE(phrase_matches("您可以扫描二维码哦", phrase));        // 短语重叠不命中
}

// 匹配：取消二维码话术命中（2026-10-17 现场需求：agent 说「好的，这就
// 取消二维码显示」-> 撤屏；与显示语料互不误触——「取消」二字是区分点）
TEST(PhraseMatches, QrCancelPhrase)
{
  const std::string phrase = "好的，这就取消二维码显示";
  EXPECT_TRUE(phrase_matches(phrase, phrase));                 // 原句
  EXPECT_TRUE(phrase_matches("好的，这就取消二维码显示。", phrase));  // 尾标点
  EXPECT_TRUE(phrase_matches(                                  // 前后夹语气词
    "嗯好的这就取消二维码显示啦", phrase));
  EXPECT_FALSE(phrase_matches("这就取消二维码", phrase));      // 话术截断不命中
  // 与显示语料互不误触（两条语料归一化后互不为子串）
  const std::string show_phrase =
    "收到，这就给您显示二维码，您可以扫描二维码留下您的个人信息哦";
  EXPECT_FALSE(phrase_matches(show_phrase, phrase));
  EXPECT_FALSE(phrase_matches(phrase, show_phrase));
}

// 匹配：重定位话术命中（2026-10-16 语料触发重定位：展厅 A~E 五点各一条
// 语料；点位字母不同互不误触，缺「开始重定位」不命中）
TEST(PhraseMatches, RelocatePhrase)
{
  const std::string phrase_a = "好的，这就在展厅A点开始重定位";
  EXPECT_TRUE(phrase_matches("好的，这就在展厅A点开始重定位。", phrase_a));  // 原句+尾标点
  EXPECT_TRUE(phrase_matches(                                                // 前后夹语气词+大小写
    "嗯，好的这就在展厅a点开始重定位哈", phrase_a));
  EXPECT_FALSE(phrase_matches("好的，这就在展厅B点开始重定位", phrase_a));  // B 点话术不命中 A
  EXPECT_FALSE(phrase_matches("好的，这就带您去展厅A点", phrase_a));        // 缺「开始重定位」
}

// 归一化：拉丁扩展/IPA 字母保留（2026-10-19 动态导航语料：
// 车型名含音标如「比亚迪ætəʊ 1」，æ/ə/ʊ 属拉丁扩展区间——丢弃会使
// 语料缺字符被截短，过短易误命中其他「比亚迪..N」话术，须完整参与
// 匹配）；拉丁扩展-1 大写（À..Þ）+0x20 转小写，×/÷ 是符号非字母仍丢弃
TEST(NormalizePhraseText, LatinExtendedPreserved)
{
  EXPECT_EQ(normalize_phrase_text("比亚迪ætəʊ 1"), "比亚迪ætəʊ1");
  EXPECT_EQ(normalize_phrase_text("比亚迪Æ 2"), "比亚迪æ2");       // 拉丁扩展-1 大写转小写
  EXPECT_EQ(normalize_phrase_text("2×3÷4"), "234");               // ×÷ 丢弃
}

// 匹配：动态生成的导航语料（点位映射表 name 按模板生成，2026-10-19
// 现场需求）与 agent 话术写法差异（空格/标点/大小写）归一化后命中；
// 车型名相近（M9 与 M9 DM-i、ætəʊ 1 与 2）互不误触
TEST(PhraseMatches, NavigateDynamicPhrase)
{
  const std::string phrase = "要去看比亚迪ætəʊ 1，没问题";
  EXPECT_TRUE(phrase_matches("要去看比亚迪ætəʊ 1，没问题。", phrase));   // 尾标点
  EXPECT_TRUE(phrase_matches("嗯，要去看比亚迪ætəʊ 1，没问题哈", phrase));  // 前后夹语气词
  EXPECT_TRUE(phrase_matches(                                             // 语料侧带空格/大小写差异
    "要去看比亚迪 Ætəʊ 1，没问题", phrase));
  EXPECT_TRUE(phrase_matches(                                             // sea lion 大小写（现场实测 agent 说小写）
    "要去看比亚迪sea lion 7，没问题", "要去看比亚迪Sea Lion 7，没问题"));
  EXPECT_FALSE(phrase_matches("要去看比亚迪ætəʊ 2，没问题", phrase));   // 相邻车型不误触
  EXPECT_FALSE(phrase_matches("要去看比亚迪M9，没问题", "要去看比亚迪M9 DM-i，没问题"));
  EXPECT_FALSE(phrase_matches("要去看比亚迪，没问题", phrase));         // 缺车型名不命中
}

// 匹配：固定指令居句尾、前面带自由发挥长内容也命中（2026-10-20 现场
// 实话术：agent 应答「安排！…看看越野模式的中控界面～要去看腾势 B5，
// 没问题」，指令在句尾且「腾势 B5」带空格）；另覆盖当时未命中的真实
// 原因——2 条测试表生成的「要去看腾势，没问题」缺 b5，不是该句的
// 子串，正确不命中（防退化回归）
TEST(PhraseMatches, NavigatePhraseAtTailWithFreeText)
{
  const std::string phrase = "要去看腾势B5，没问题";  // 点位表 name=腾势B5 按模板生成
  // 2026-10-20 21:36 现场 agent 原话（指令在句尾+自由发挥前缀+空格差异）
  EXPECT_TRUE(phrase_matches(
    "安排！这就带您到腾势B5旁边，摸一摸NAPPA皮座椅，看看越野模式的中控界面～"
    "要去看腾势 B5，没问题", phrase));
  // 2 条测试表场景（当时未命中的根因）：「要去看腾势，没问题」缺 b5
  EXPECT_FALSE(phrase_matches(
    "安排！这就带您到腾势B5旁边，摸一摸NAPPA皮座椅，看看越野模式的中控界面～"
    "要去看腾势 B5，没问题", "要去看腾势，没问题"));
  // 相邻车型不误触
  EXPECT_FALSE(phrase_matches("要去看腾势 B5，没问题", "要去看腾势B8，没问题"));
}

// 匹配：空语料/空识别不命中
TEST(PhraseMatches, EmptyInputs)
{
  EXPECT_FALSE(phrase_matches("", "好嘞，马上开始迎接我的顾客喽"));
  EXPECT_FALSE(phrase_matches("好嘞", ""));
}

}  // namespace
