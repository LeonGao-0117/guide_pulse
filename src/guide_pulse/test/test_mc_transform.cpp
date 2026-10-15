// ============================================================================
// 迎宾变形纯函数单测：路径表/趴姿集合/reason 人话的不变式。
// 表内容逐条抄自 SDK _ACTION_RULES（真机实测口径），此处不校验「抄得
// 对不对」（源头在厂家文档），只锁结构不变式——防手滑改坏表结构。
// ============================================================================
#include <string>

#include <gtest/gtest.h>

#include "guide_pulse/mc_transform.hpp"

namespace guide_pulse
{
namespace
{

constexpr const char * kToBiped = "QUADRUPED_TO_BIPED_ROTATE";
constexpr const char * kBipedTarget = "BIPED_LOCOMOTION_WBC";

// 趴姿集合的每个动作都在路径表里有起点（否则趴姿查表会走到「未知路径」
// 分支，与「默认扶起」语义矛盾）
TEST(McTransformTables, DownActionsAllHavePaths)
{
  for (const auto & action : mc_down_actions()) {
    EXPECT_TRUE(mc_path_to_biped().count(action) > 0)
      << "趴姿动作缺少路径表起点: " << action;
  }
}

// 非空的路径必须以变形动作收尾（终点都是变成人形），
// 且不含空串步骤（空步骤发出去会被运控拒）；空路径起点由下方专门测试覆盖
TEST(McTransformTables, EveryPathEndsWithTransform)
{
  for (const auto & [start, path] : mc_path_to_biped()) {
    if (path.empty()) {
      continue;  // 空路径 = 已是人形只等待，合法
    }
    ASSERT_EQ(path.back(), kToBiped)
      << "起点 " << start << " 的路径未以变形动作收尾";
    for (const auto & step : path) {
      EXPECT_FALSE(step.empty()) << "起点 " << start << " 的路径含空步骤";
    }
  }
}

// 人形起点的路径为空（已达标：只等待不动作）
TEST(McTransformTables, BipedStartsNeedNoAction)
{
  EXPECT_TRUE(mc_path_to_biped().at(kBipedTarget).empty());
  EXPECT_TRUE(mc_path_to_biped().at("BIPED_STAND_DEFAULT").empty());
}

// 中间步骤只允许出现在已知动作集合内（站定/行走/被动三个中间态）
TEST(McTransformTables, IntermediateStepsAreKnown)
{
  const std::set<std::string> known = {
    "QUADRUPED_STAND_DEFAULT", "QUADRUPED_LOCOMOTION_DEFAULT", "PASSIVE_DEFAULT",
    kToBiped,
  };
  for (const auto & [start, path] : mc_path_to_biped()) {
    for (const auto & step : path) {
      EXPECT_TRUE(known.count(step) > 0)
        << "起点 " << start << " 的中间步骤不在已知集合: " << step;
    }
  }
}

// reason 人话：已知码非空、未知码空串（调用方按空串走裸数字兜底）
TEST(McTransformTables, ReasonTextCoverage)
{
  for (int reason : {4, 5, 6, 9, 10, 11, 12, 13, 14, 15, 19, 22, 27, 30}) {
    EXPECT_FALSE(mc_reason_text(reason).empty()) << "已知 reason=" << reason << " 缺人话";
  }
  for (int reason : {0, 1, 99, -1}) {
    EXPECT_TRUE(mc_reason_text(reason).empty()) << "未知 reason=" << reason << " 应返回空串";
  }
}

// 关停/禁用语义：enabled=false 的模块确保幂等无害（不建客户端、
// ensure_biped 直接返回、可安全析构）
TEST(McTransformDisabled, IsNoOp)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("mc_transform_disabled_test", options);
  McTransform::Config cfg;  // 默认 enabled=false
  McTransform transform(node, cfg);
  transform.ensure_biped("测试");
  EXPECT_FALSE(transform.in_flight());
  EXPECT_FALSE(transform.wait_done());
  transform.shutdown();  // 幂等，可重复调用
  transform.shutdown();
}

}  // namespace
}  // namespace guide_pulse

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  // McTransformDisabled 用例需创建 rclcpp 节点（guard condition 依赖全局 context）
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
