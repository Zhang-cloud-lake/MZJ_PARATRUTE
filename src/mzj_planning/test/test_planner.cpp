#include <gtest/gtest.h>
#include "mzj_planning/planner.hpp"
TEST(Planner, PredictsWindAtTouchdown)
{
  mzj_model::Parameters p; p.wind_y = 1;
  auto plan = mzj_planning::plan({}, 125, 25, p);
  ASSERT_FALSE(plan.points.empty());
  EXPECT_LT(plan.landing_error, 1e-6);
  EXPECT_NEAR(plan.points.back().state.z, 0, 1e-9);
  for (const auto & point : plan.points) {
    EXPECT_LE(std::abs(point.yaw_rate), p.max_yaw_rate);
  }
}
TEST(Planner, BlockedAndInvalidInputs)
{
  EXPECT_TRUE(mzj_planning::plan({}, 125, 0, {}, {{0, 0, 100, 100}}).points.empty());
  EXPECT_TRUE(mzj_planning::plan({}, NAN, 0, {}).points.empty());
  EXPECT_TRUE(mzj_planning::plan({0, 0, 1000, 0}, 0, 0, {}).points.empty());
}
TEST(Planner, ChecksSegmentBetweenSamples)
{
  EXPECT_TRUE(mzj_planning::collision({-2, 0, 1, 0}, {2, 0, 1, 0},
    {{0, 0, .1, 2}}, 0));
}
