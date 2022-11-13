#include <gtest/gtest.h>
#include <cooperative_perception/ctrv_model.hpp>
#include <units.h>

/**
 * Test CTRV nextState function against pure rotation
 */
TEST(TestCtrvModel, NextStatePureRotation)
{
  using namespace units::literals;

  const cooperative_perception::CtrvState state{ 0_m, 0_m, 0_mps, 0_rad, 1_rad_per_s };
  const auto next_state{ cooperative_perception::nextState(state, 0.5_s) };

  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.position_x), 0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.position_y), 0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.velocity), 0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.yaw), 0.5);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.yaw_rate), 1);
}

/**
 * Test CTRV nextState function against pure translation
 */
TEST(TestCtrvModel, NextStatePureTranslation)
{
  using namespace units::literals;

  const cooperative_perception::CtrvState state{ 0_m, 0_m, 1_mps, 0_rad, 0_rad_per_s };
  const auto next_state{ cooperative_perception::nextState(state, 0.5_s) };

  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.position_x), 0.5);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.position_y), 0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.velocity), 1);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.yaw), 0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.yaw_rate), 0);
}

/**
 * Test CTRV nextState function against mixed rotation and translation
 */
TEST(TestCtrvModel, NextStateRotationAndTranslation)
{
  using namespace units::literals;

  const cooperative_perception::CtrvState state{ 0_m, 0_m, 1_mps, 0_rad, 1_rad_per_s };
  const auto next_state{ cooperative_perception::nextState(state, 0.5_s) };

  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.position_x), 0.479425539);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.position_y), 0.122417438);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.velocity), 1);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.yaw), 0.5);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(next_state.yaw_rate), 1);
}
