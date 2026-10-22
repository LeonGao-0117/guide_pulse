// ============================================================================
// 点位映射表本地存储单测（v0.25.0 P-039；v0.28.0 P-042 播放词 text）：
// 全量替换语义、原子落盘、跨实例加载（重启恢复）、name->code 解析、
// 播放词随表持久化与查询、损坏文件防御、纯内存模式。
// 无 ROS 依赖。
// ============================================================================
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "guide_pulse/point_map.hpp"

namespace
{

using guide_pulse::PointMapEntry;
using guide_pulse::PointMapStore;

std::vector<PointMapEntry> sample_points()
{
  return {
    {"A", "比亚迪ATTO 1"},
    {"B", "比亚迪 M9 DM-i"},
    {"C", "腾势 B5"},
    {"D", "腾势 D9"},
    {"E", "腾势 Z9 GT"},
  };
}

class PointMapStoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::filesystem::temp_directory_path() /
      ("guide_pulse_point_map_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(dir_);
  }

  std::filesystem::path dir_;
};

// 全量替换：每次下发整表覆盖旧表（不做增量合并），内存即时生效
TEST_F(PointMapStoreTest, ReplaceIsFullSwapNotMerge)
{
  PointMapStore store((dir_ / "map.json").string());
  ASSERT_TRUE(store.load());
  EXPECT_EQ(store.size(), 0U);

  ASSERT_TRUE(store.replace(sample_points()));
  ASSERT_EQ(store.size(), 5U);
  ASSERT_EQ(store.points()[0].code, "A");
  ASSERT_EQ(store.points()[4].name, "腾势 Z9 GT");

  // 第二次下发 2 条：旧 5 条全部被替换（非合并）
  const std::vector<PointMapEntry> second = {{"X", "东门"}, {"Y", "充电桩"}};
  ASSERT_TRUE(store.replace(second));
  ASSERT_EQ(store.size(), 2U);
  EXPECT_EQ(store.points()[0].code, "X");
  EXPECT_EQ(store.points()[1].name, "充电桩");
}

// 落盘 + 跨实例加载：重启后表自动恢复（App 无需重推）
TEST_F(PointMapStoreTest, PersistAndReloadAcrossRestart)
{
  {
    PointMapStore store((dir_ / "map.json").string());
    ASSERT_TRUE(store.load());
    ASSERT_TRUE(store.replace(sample_points()));
  }
  PointMapStore reloaded((dir_ / "map.json").string());
  ASSERT_TRUE(reloaded.load());
  ASSERT_EQ(reloaded.size(), 5U);
  EXPECT_EQ(reloaded.resolve("比亚迪ATTO 1").value_or(""), "A");
  EXPECT_EQ(reloaded.resolve("腾势 Z9 GT").value_or(""), "E");
}

// point_name -> code 解析：两级精确匹配（先 name 后 code，均区分
// 大小写）、未命中/空表返回 nullopt
TEST_F(PointMapStoreTest, ResolveNameToCode)
{
  PointMapStore store((dir_ / "map.json").string());
  ASSERT_TRUE(store.load());

  // 空表：未命中
  EXPECT_FALSE(store.resolve("比亚迪ATTO 1").has_value());

  ASSERT_TRUE(store.replace(sample_points()));
  // 一级：code 直接匹配（2026-10-17 联调确认 App 下发的就是 code，如「A」）
  EXPECT_EQ(store.resolve("A"), std::optional<std::string>("A"));
  EXPECT_EQ(store.resolve("E"), std::optional<std::string>("E"));
  // 二级：name 匹配（兜底兼容人话名）
  EXPECT_EQ(store.resolve("比亚迪ATTO 1"), std::optional<std::string>("A"));
  EXPECT_EQ(store.resolve("腾势 B5"), std::optional<std::string>("C"));
  // code 优先于 name：code 命中条目与 name 命中条目不同时以 code 为准
  const std::vector<PointMapEntry> crossed = {
    {"A", "B"}, {"B", "展位B"}};
  ASSERT_TRUE(store.replace(crossed));
  EXPECT_EQ(store.resolve("B"), std::optional<std::string>("B"));  // 命中 code「B」
  EXPECT_EQ(store.resolve("展位B"), std::optional<std::string>("B"));
  // 未命中的名称
  EXPECT_FALSE(store.resolve("比亚迪ATTO 3").has_value());
  EXPECT_FALSE(store.resolve("a").has_value());  // 大小写敏感
  EXPECT_FALSE(store.resolve("").has_value());
}

// 名称关键词 -> code 子串匹配（2026-10-19 现场需求：welcome_prep 语料
// 点位部分写「迎宾」，映射表 name 如「迎宾点」-> code Y）：首个 name
// 含关键词的条目命中；空关键词/未命中返回 nullopt
TEST_F(PointMapStoreTest, ResolveByNameKeyword)
{
  PointMapStore store((dir_ / "map.json").string());
  ASSERT_TRUE(store.load());

  // 空表：未命中
  EXPECT_FALSE(store.resolve_by_name_keyword("迎宾").has_value());

  ASSERT_TRUE(store.replace(sample_points()));
  // 表内无 name 含「迎宾」的条目：未命中
  EXPECT_FALSE(store.resolve_by_name_keyword("迎宾").has_value());
  // name 子串命中（含空格差异自然消解：name「比亚迪 M9 DM-i」）
  EXPECT_EQ(store.resolve_by_name_keyword("M9"), std::optional<std::string>("B"));
  EXPECT_EQ(store.resolve_by_name_keyword("腾势 D9"), std::optional<std::string>("D"));
  // 多条命中取先到者（与 App 展示顺序一致）
  EXPECT_EQ(store.resolve_by_name_keyword("腾势"), std::optional<std::string>("C"));
  // 空关键词不匹配（防空串全命中）
  EXPECT_FALSE(store.resolve_by_name_keyword("").has_value());
  // 大小写敏感（与 resolve 精确匹配同口径）
  EXPECT_FALSE(store.resolve_by_name_keyword("m9").has_value());

  // 迎宾点场景：name「迎宾点」-> code Y（换 code 只需 App 改表）
  const std::vector<PointMapEntry> with_welcome = {
    {"A", "比亚迪ATTO 1"}, {"Y", "迎宾点"}};
  ASSERT_TRUE(store.replace(with_welcome));
  EXPECT_EQ(store.resolve_by_name_keyword("迎宾"), std::optional<std::string>("Y"));
  // 精确解析优先不受影响：resolve("Y") 一级命中 code
  EXPECT_EQ(store.resolve("Y"), std::optional<std::string>("Y"));
}

// 落盘失败：replace 返回 false 且内存不应用本次表（未生效语义）。
// 构造法：父路径是普通文件 -> create_directories/ofstream 均失败
//（root 运行也有效，权限法对 root 无效）
TEST_F(PointMapStoreTest, FailedPersistDoesNotApplyToMemory)
{
  const auto blocker = dir_ / "blocker";
  {std::ofstream out(blocker, std::ios::trunc); out << "regular file";}
  PointMapStore broken((blocker / "map.json").string());
  ASSERT_TRUE(broken.load());  // 文件不存在：空表正常

  // 落盘失败：本次表未生效（内存保持原状——这里是空表）
  EXPECT_FALSE(broken.replace(sample_points()));
  EXPECT_EQ(broken.size(), 0U);
  EXPECT_FALSE(broken.resolve("比亚迪ATTO 1").has_value());

  // 连续失败仍保持未生效（调用方按 failed/point_map_failed 回执）
  const std::vector<PointMapEntry> second = {{"Z", "新点"}};
  EXPECT_FALSE(broken.replace(second));
  EXPECT_EQ(broken.size(), 0U);
}

// 对照：正常路径首次成功后，replace 失败路径以外的表不受影响
//（points_ 仅在落盘成功后更新——与上例共同锁住「先落盘后改内存」顺序）
TEST_F(PointMapStoreTest, SuccessfulPersistAppliesToMemory)
{
  PointMapStore store((dir_ / "map.json").string());
  ASSERT_TRUE(store.load());
  EXPECT_TRUE(store.replace(sample_points()));
  EXPECT_EQ(store.size(), 5U);
  EXPECT_EQ(store.resolve("腾势 B5"), std::optional<std::string>("C"));
}

// 损坏文件：load 返回 false 且按空表启动（保留文件等 App 重推覆盖）
TEST_F(PointMapStoreTest, CorruptedFileLoadsEmptyAndFails)
{
  const auto path = dir_ / "map.json";
  {std::ofstream out(path, std::ios::trunc); out << "{ not json";}
  PointMapStore store(path.string());
  EXPECT_FALSE(store.load());
  EXPECT_EQ(store.size(), 0U);
  // 损坏文件未被删除（人工排查 / App 重推覆盖）
  EXPECT_TRUE(std::filesystem::exists(path));
}

// 结构不符（points 非数组 / 条目缺字段 / code 重复）：整表作废
TEST_F(PointMapStoreTest, InvalidStructureLoadsEmpty)
{
  const auto path = dir_ / "map.json";
  const std::vector<std::string> bad_docs = {
    "{\"spec_version\":\"1.0\",\"points\":\"not-array\"}",
    "{\"spec_version\":\"1.0\",\"points\":[{\"code\":\"A\"}]}",
    "{\"spec_version\":\"1.0\",\"points\":[{\"code\":\"A\",\"name\":\"x\"},"
    "{\"code\":\"A\",\"name\":\"y\"}]}",
    "{\"unexpected\":true}",
  };
  for (const auto & doc : bad_docs) {
    {std::ofstream out(path, std::ios::trunc); out << doc;}
    PointMapStore store(path.string());
    EXPECT_FALSE(store.load()) << doc;
    EXPECT_EQ(store.size(), 0U) << doc;
  }
}

// 文件不存在（首次部署）：空表合法启动
TEST_F(PointMapStoreTest, MissingFileIsFreshStart)
{
  PointMapStore store((dir_ / "never-written.json").string());
  EXPECT_TRUE(store.load());
  EXPECT_EQ(store.size(), 0U);
}

// 纯内存模式（空路径）：replace 不落盘、重启即失效（联调用）
TEST_F(PointMapStoreTest, InMemoryModeWithoutFile)
{
  PointMapStore store("");
  ASSERT_TRUE(store.load());
  ASSERT_TRUE(store.replace(sample_points()));
  EXPECT_EQ(store.size(), 5U);
  EXPECT_EQ(store.resolve("腾势 D9"), std::optional<std::string>("D"));
}

// 原子性：落盘成功后文件是完整 JSON（无半写状态）
TEST_F(PointMapStoreTest, PersistedFileIsCompleteJson)
{
  const auto path = dir_ / "map.json";
  PointMapStore store(path.string());
  ASSERT_TRUE(store.load());
  ASSERT_TRUE(store.replace(sample_points()));
  // 内容可被下一个实例完整解析（无截断）
  PointMapStore next(path.string());
  ASSERT_TRUE(next.load());
  ASSERT_EQ(next.size(), 5U);
  // 覆盖写第二轮后旧内容完全替换
  const std::vector<PointMapEntry> second = {{"P001", "东门路口"}};
  ASSERT_TRUE(store.replace(second));
  PointMapStore third(path.string());
  ASSERT_TRUE(third.load());
  ASSERT_EQ(third.size(), 1U);
  EXPECT_EQ(third.resolve("东门路口"), std::optional<std::string>("P001"));
  EXPECT_FALSE(third.resolve("比亚迪ATTO 1").has_value());
}

// ---------------------------------------------------------------------------
// 播放文本 text（v0.28.0 P-042）：落盘/加载/查询与点位映射同生命周期
// ---------------------------------------------------------------------------

// text 随表落盘并跨实例恢复（断电持久的行为内核：写文件 + 重读）；
// 无 text 条目往返后仍为 nullopt
TEST_F(PointMapStoreTest, TextPersistsAndReloadsAcrossRestart)
{
  const auto path = dir_ / "map.json";
  {
    PointMapStore store(path.string());
    ASSERT_TRUE(store.load());
    std::vector<PointMapEntry> points = {
      {"A", "比亚迪ætəʊ 1", std::optional<std::string>("这就是比亚迪ætəʊ 1的展车了")},
      {"B", "比亚迪ætəʊ 3"},  // 未配播放词
    };
    ASSERT_TRUE(store.replace(points));
  }
  PointMapStore reloaded(path.string());
  ASSERT_TRUE(reloaded.load());
  ASSERT_EQ(reloaded.size(), 2U);
  EXPECT_EQ(
    reloaded.text_for_code("A"),
    std::optional<std::string>("这就是比亚迪ætəʊ 1的展车了"));
  EXPECT_FALSE(reloaded.text_for_code("B").has_value());
  // code 未命中 / name 不参与 text 查询
  EXPECT_FALSE(reloaded.text_for_code("比亚迪ætəʊ 1").has_value());
  EXPECT_FALSE(reloaded.text_for_code("Z").has_value());
}

// text_for_code：内存态（未重载）同样可查；全量替换后旧 text 一并失效
TEST_F(PointMapStoreTest, TextForCodeReplacedOnFullSwap)
{
  PointMapStore store((dir_ / "map.json").string());
  ASSERT_TRUE(store.load());
  const std::vector<PointMapEntry> first = {
    {"A", "展位A", std::optional<std::string>("A 的播放词")}};
  ASSERT_TRUE(store.replace(first));
  EXPECT_EQ(store.text_for_code("A"), std::optional<std::string>("A 的播放词"));
  // 整表替换后 A 的 text 消失（全量替换语义，不做合并）
  const std::vector<PointMapEntry> second = {{"A", "展位A"}};
  ASSERT_TRUE(store.replace(second));
  EXPECT_FALSE(store.text_for_code("A").has_value());
}

// 落盘文件格式：无 text 条目不写 text 键（omitempty，旧格式/旧工具兼容）
TEST_F(PointMapStoreTest, OmittedTextNotWrittenToFile)
{
  const auto path = dir_ / "map.json";
  PointMapStore store(path.string());
  ASSERT_TRUE(store.load());
  const std::vector<PointMapEntry> points = {
    {"A", "展位A", std::optional<std::string>("有词")},
    {"B", "展位B"}};
  ASSERT_TRUE(store.replace(points));
  std::ifstream in(path, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(in)),
    std::istreambuf_iterator<char>());
  // A 条目落了 text 键，B 条目没有（按条目出现顺序粗验：首个 text 键
  // 出现在 A 之后；B 条目无独立 text 键）
  EXPECT_NE(content.find("\"text\": \"有词\""), std::string::npos);
  const auto b_pos = content.find("\"展位B\"");
  ASSERT_NE(b_pos, std::string::npos);
  EXPECT_EQ(content.find("\"text\"", b_pos), std::string::npos);
}

// 加载防御：text 空串 / 非 string / 超 2048 标量 / 条目未知第 3 字段
// ——整表作废（load=false，空表启动，保留文件等 App 重推覆盖）
TEST_F(PointMapStoreTest, LoadRejectsInvalidTextFields)
{
  const auto path = dir_ / "map.json";
  const std::vector<std::string> bad_docs = {
    "{\"spec_version\":\"1.0\",\"points\":[{\"code\":\"A\",\"name\":\"x\",\"text\":\"\"}]}",
    "{\"spec_version\":\"1.0\",\"points\":[{\"code\":\"A\",\"name\":\"x\",\"text\":123}]}",
    "{\"spec_version\":\"1.0\",\"points\":[{\"code\":\"A\",\"name\":\"x\",\"text\":\"" +
      std::string(2049, 'a') + "\"}]}",
    "{\"spec_version\":\"1.0\",\"points\":[{\"code\":\"A\",\"name\":\"x\",\"extra\":1}]}",
  };
  for (const auto & doc : bad_docs) {
    {std::ofstream out(path, std::ios::trunc); out << doc;}
    PointMapStore store(path.string());
    EXPECT_FALSE(store.load()) << doc;
    EXPECT_EQ(store.size(), 0U) << doc;
  }
}

// 加载兼容：恰 2048 标量 text 与混合（部分条目带 text）均合法
TEST_F(PointMapStoreTest, LoadAcceptsBoundaryText)
{
  const auto path = dir_ / "map.json";
  {
    std::ofstream out(path, std::ios::trunc);
    out << "{\"spec_version\":\"1.0\",\"points\":[{\"code\":\"A\",\"name\":\"x\",\"text\":\""
        << std::string(2048, 'a') << "\"},{\"code\":\"B\",\"name\":\"y\"}]}";
  }
  PointMapStore store(path.string());
  ASSERT_TRUE(store.load());
  ASSERT_EQ(store.size(), 2U);
  ASSERT_TRUE(store.text_for_code("A").has_value());
  EXPECT_EQ(store.text_for_code("A")->size(), 2048U);
  EXPECT_FALSE(store.text_for_code("B").has_value());
}

}  // namespace
