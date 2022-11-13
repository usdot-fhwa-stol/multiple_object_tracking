#include <gtest/gtest.h>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/units.hpp>
#include <units.h>

namespace
{

constexpr auto kAbsoluteTolerance{ 1e-8 };

}  // namespace

/**
 * Test CTRV nextState function against pure rotation
 */
TEST(TestCtrvModel, NextStatePureRotation)
{
  using namespace units::literals;

  const cooperative_perception::CtrvState state{ 0_m, 0_m, 0_mps, 0_rad, 1_rad_per_s };
  const auto next_state{ cooperative_perception::nextState(state, 0.5_s) };

  EXPECT_NEAR(units::unit_cast<double>(next_state.position_x), 0, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.position_y), 0, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.velocity), 0, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.yaw), 0.5, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.yaw_rate), 1, kAbsoluteTolerance);
}

/**
 * Test CTRV nextState function against pure translation
 */
TEST(TestCtrvModel, NextStatePureTranslation)
{
  using namespace units::literals;

  const cooperative_perception::CtrvState state{ 0_m, 0_m, 1_mps, 0_rad, 0_rad_per_s };
  const auto next_state{ cooperative_perception::nextState(state, 0.5_s) };

  EXPECT_NEAR(units::unit_cast<double>(next_state.position_x), 0.5, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.position_y), 0, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.velocity), 1, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.yaw), 0, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.yaw_rate), 0, kAbsoluteTolerance);
}

/**
 * Test CTRV nextState function against mixed rotation and translation
 */
TEST(TestCtrvModel, NextStateRotationAndTranslation)
{
  using namespace units::literals;

  const cooperative_perception::CtrvState state{ 0_m, 0_m, 1_mps, 0_rad, 1_rad_per_s };
  const auto next_state{ cooperative_perception::nextState(state, 0.5_s) };

  EXPECT_NEAR(units::unit_cast<double>(next_state.position_x), 0.479425539, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.position_y), 0.122417438, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.velocity), 1, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.yaw), 0.5, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.yaw_rate), 1, kAbsoluteTolerance);
}

TEST(TestCtrvModel, NextStateStochastic)
{
  using namespace units::literals;

  const cooperative_perception::CtrvState state{ 0_m, 0_m, 1_mps, 0_rad, 1_rad_per_s };
  const auto next_state{ cooperative_perception::nextState(state, 0.5_s, 1_mps_sq, 1_rad_per_s_sq) };

  EXPECT_NEAR(units::unit_cast<double>(next_state.position_x), 0.604425539, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.position_y), 0.122417438, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.velocity), 1.5, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.yaw), 0.625, kAbsoluteTolerance);
  EXPECT_NEAR(units::unit_cast<double>(next_state.yaw_rate), 1.5, kAbsoluteTolerance);
}
