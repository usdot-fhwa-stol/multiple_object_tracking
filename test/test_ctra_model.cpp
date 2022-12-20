#include <gtest/gtest.h>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/units.hpp>
#include <units.h>

namespace cp = cooperative_perception;

/**
 * Test CTRA nextState function against pure rotation
 */
TEST(TestCtraModel, NextStatePureRotation)
{
  using namespace units::literals;

  constexpr cp::CtraState state{ 0_m, 0_m, 0_mps, 0_rad, 1_rad_per_s, 0_mps_sq };
  const auto next_state{ cp::nextState(state, 0.5_s) };
  constexpr cp::CtraState expected_state{ 0_m, 0_m, 0_mps, 0.5_rad, 1_rad_per_s, 0_mps_sq };

  EXPECT_TRUE(cp::utils::almostEqual(next_state, expected_state));
}

/**
 * Test CTRA nextState function against pure translation
 */
TEST(TestCtraModel, NextStatePureTranslation)
{
  using namespace units::literals;

  constexpr cp::CtraState state{ 0_m, 0_m, 1_mps, 0_rad, 0_rad_per_s, 1_mps_sq };
  const auto next_state{ cp::nextState(state, 0.5_s) };
  constexpr cp::CtraState expected_state{ 0.5_m, 0_m, 2_mps, 0_rad, 0_rad_per_s, 1_mps_sq };

  EXPECT_TRUE(cp::utils::almostEqual(next_state, expected_state));
}

/**
 * Test CTRA nextState function against mixed rotation and translation
 */
TEST(TestCtraModel, NextStateRotationAndTranslation)
{
  using namespace units::literals;

  constexpr cp::CtraState state{ 0_m, 0_m, 1_mps, 0_rad, 1_rad_per_s, 1_mps_sq };
  const auto next_state{ cp::nextState(state, 0.5_s) };
  constexpr cp::CtraState expected_state{ 0.065056359_m, 0.5_m, 2_mps, 0.5_rad, 1_rad_per_s, 1_mps_sq };

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(next_state, 9), expected_state));
}

TEST(TestCtraModel, Equality)
{
  using namespace units::literals;

  constexpr cp::CtraState state1{ 1.23_m, 2.435_m, 5544_mps, 34656.6543_rad, 595633.555_rad_per_s, 100_mps_sq };
  constexpr cp::CtraState state2{ 1.23_m, 2.435_m, 5544_mps, 34656.6543_rad, 595633.555_rad_per_s, 100_mps_sq };
  constexpr cp::CtraState state3{ 1_m, 2_m, 4_mps, 3_rad, 1.000000000000000000000000001_rad_per_s, 1_mps_sq };
  constexpr cp::CtraState state4{ 1_m, 2_m, 4_mps, 3_rad, 1_rad_per_s, 1_mps_sq };

  EXPECT_FALSE(cp::utils::almostEqual(state1, state2));
  EXPECT_TRUE(cp::utils::almostEqual(state3, state4));
  EXPECT_FALSE(cp::utils::almostEqual(state1, state4));
  EXPECT_FALSE(cp::utils::almostEqual(state2, state3));
  EXPECT_TRUE(cp::utils::almostEqual(state3, state3));
}
