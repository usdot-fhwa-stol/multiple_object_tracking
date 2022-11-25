#include <gtest/gtest.h>
#include <cooperative_perception/detected_object.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <units.h>

struct CtrvCovariance
{
};

namespace cp = cooperative_perception;

TEST(TestDetectedObject, CtrvObjectDefaultConstruction)
{
  auto object = cp::DetectedObject<cp::CtrvState, CtrvCovariance>{};

  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.timestamp), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.position_x), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.position_y), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.velocity), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.yaw), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.yaw_rate), 0.0);
}

TEST(TestDetectedObject, CtrvObjectCustomConstruction)
{
  using namespace units::literals;

  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
  auto object =
      cp::DetectedObject<cp::CtrvState, CtrvCovariance>{ units::time::second_t{ 1 },
                                                         cp::CtrvState{ 1_m, 2_m, 3_mps, 4_rad, 5_rad_per_s } };
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.timestamp), 1.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.position_x), 1.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.position_y), 2.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.velocity), 3.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.yaw), 4.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(object.state.yaw_rate), 5.0);
}
