// ============================================================================
// 讲解模块（TourRunner）单元测试（纯 C++，FakeNavExecutor 手控回调）
// 覆盖：到达成功落槽并携带讲解词、到达失败落槽、新导航替换旧导航
//（旧回调作废）、取消丢弃在途回调、导航代次隔离。
// ============================================================================
#include <functional>
#include <gtest/gtest.h>

#include "guide_pulse/tour.hpp"

namespace
{

// 手控导航执行器：记录 goto 请求并允许测试显式触发回调
class FakeNavExecutor final : public guide_pulse::NavExecutor
{
public:
  struct Request
  {
    std::string point;
    std::function<void(guide_pulse::NavOutcome)> on_done;
  };

  void goto_point(
    const std::string & point_name,
    std::function<void(guide_pulse::NavOutcome)> on_done) override
  {
    requests.push_back({point_name, std::move(on_done)});
  }

  void cancel() override
  {
    cancelled.push_back(latest_point());
  }

  // 测试驱动：触发第 n 次 goto 请求的回调
  void finish(std::size_t index, bool ok, const std::string & detail = "")
  {
    guide_pulse::NavOutcome outcome;
    outcome.ok = ok;
    outcome.detail = detail;
    requests.at(index).on_done(outcome);
  }

  std::string latest_point() const
  {
    return requests.empty() ? std::string() : requests.back().point;
  }

  std::vector<Request> requests;
  std::vector<std::string> cancelled;
};

TEST(TourTest, ArrivalSuccessCarriesTexts)
{
  FakeNavExecutor nav;
  guide_pulse::TourRunner tour(nav);
  tour.start("3号展台", {"讲解一", "讲解二"});
  EXPECT_TRUE(tour.navigating());
  EXPECT_FALSE(tour.poll_arrival().has_value());  // 未到达：无事件

  nav.finish(0, true, "ok");
  const auto arrival = tour.poll_arrival();
  ASSERT_TRUE(arrival.has_value());
  EXPECT_TRUE(arrival->ok);
  EXPECT_EQ(arrival->texts, (std::vector<std::string>{"讲解一", "讲解二"}));
  // 槽取走即清：重复 poll 无事件
  EXPECT_FALSE(tour.poll_arrival().has_value());
}

TEST(TourTest, ArrivalFailureFlaggedForLogging)
{
  FakeNavExecutor nav;
  guide_pulse::TourRunner tour(nav);
  tour.start("东门", {});
  nav.finish(0, false, "point not found");
  const auto arrival = tour.poll_arrival();
  ASSERT_TRUE(arrival.has_value());
  EXPECT_FALSE(arrival->ok);
  EXPECT_EQ(arrival->detail, "point not found");
}

TEST(TourTest, NewNavigationReplacesOld)
{
  FakeNavExecutor nav;
  guide_pulse::TourRunner tour(nav);
  tour.start("A展台", {"A词"});
  tour.start("B展台", {"B词"});
  EXPECT_EQ(nav.latest_point(), "B展台");

  // 旧导航（A）的迟到回调作废：不落槽
  nav.finish(0, true, "A ok");
  EXPECT_FALSE(tour.poll_arrival().has_value());

  // 新导航（B）正常落槽
  nav.finish(1, true, "B ok");
  const auto arrival = tour.poll_arrival();
  ASSERT_TRUE(arrival.has_value());
  EXPECT_EQ(arrival->texts, (std::vector<std::string>{"B词"}));
}

TEST(TourTest, CancelDropsInFlightCallback)
{
  FakeNavExecutor nav;
  guide_pulse::TourRunner tour(nav);
  tour.start("A展台", {"A词"});
  tour.cancel();
  EXPECT_FALSE(tour.navigating());

  nav.finish(0, true, "late arrival");  // 取消后的迟到回调：丢弃
  EXPECT_FALSE(tour.poll_arrival().has_value());
}

TEST(TourTest, EmptyTextsMeansSilentArrival)
{
  // navigate 未带 texts：到达事件 ok=true 且 texts 为空（桥接节点只记日志）
  FakeNavExecutor nav;
  guide_pulse::TourRunner tour(nav);
  tour.start("充电桩", {});
  nav.finish(0, true, "ok");
  const auto arrival = tour.poll_arrival();
  ASSERT_TRUE(arrival.has_value());
  EXPECT_TRUE(arrival->ok);
  EXPECT_TRUE(arrival->texts.empty());
}

TEST(TourTest, CancelThenStartWorksAgain)
{
  FakeNavExecutor nav;
  guide_pulse::TourRunner tour(nav);
  tour.start("A", {"a"});
  tour.cancel();
  tour.start("B", {"b"});
  nav.finish(1, true, "ok");  // 第 2 次 goto（index=1）
  const auto arrival = tour.poll_arrival();
  ASSERT_TRUE(arrival.has_value());
  EXPECT_EQ(arrival->texts, (std::vector<std::string>{"b"}));
}

}  // namespace
