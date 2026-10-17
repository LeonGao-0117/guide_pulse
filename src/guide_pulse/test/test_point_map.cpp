// ============================================================================
// 点位映射表本地存储单测（v0.25.0 P-039）：全量替换语义、原子落盘、
// 跨实例加载（重启恢复）、name->code 解析、损坏文件防御、纯内存模式。
// 无 ROS 依赖。
// ============================================================================
#include <cstdio>
#include <filesystem>
#include <fstream>
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
  EXPECT_FALSE(store.resolve("比亚迪ATTO 2").has_value());
  EXPECT_FALSE(store.resolve("a").has_value());  // 大小写敏感
  EXPECT_FALSE(store.resolve("").has_value());
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

}  // namespace
