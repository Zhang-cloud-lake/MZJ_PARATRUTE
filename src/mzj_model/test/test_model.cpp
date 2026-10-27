#include <gtest/gtest.h>
#include "mzj_model/model.hpp"
TEST(Model, WindAndTouchdown)
{
  mzj_model::Parameters p; p.wind_y = 2;
  auto s = mzj_model::step({0, 0, 1, 0}, 0, 0, 3, p);
  EXPECT_DOUBLE_EQ(s.x, 5); EXPECT_DOUBLE_EQ(s.y, 2); EXPECT_DOUBLE_EQ(s.z, 0);
  auto landed = mzj_model::step(s, 1, 0, 1, p);
  EXPECT_DOUBLE_EQ(landed.x, s.x);
}
TEST(Model, AnalyticTurnAndStepConsistency)
{
  mzj_model::Parameters p;
  auto a = mzj_model::step({}, 1, 0, 1, p);
  auto b = mzj_model::step({}, 1, 0, .5, p);
  b = mzj_model::step(b, 1, 0, .5, p);
  EXPECT_NEAR(a.x, b.x, 1e-9); EXPECT_NEAR(a.y, b.y, 1e-9);
  EXPECT_NEAR(a.yaw, .5, 1e-9);
}
TEST(Model, InvalidCommandIsRejected)
{
  EXPECT_THROW(mzj_model::step({}, NAN, 0, .1, {}), std::invalid_argument);
  EXPECT_THROW(mzj_model::step({}, 2, 0, .1, {}), std::invalid_argument);
}
