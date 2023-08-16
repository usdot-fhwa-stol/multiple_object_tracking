/*
 * Copyright 2022 Leidos
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Developed by the Human and Vehicle Ensembles (HIVE) Lab at Virginia Commonwealth University (VCU)
 */

#include <gtest/gtest.h>
#include <units.h>

#include <cooperative_perception/angle.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/units.hpp>

namespace cp = cooperative_perception;

/**
 * Test CTRV get_next_state function against pure rotation
 */
TEST(TestCtrvModel, NextStatePureRotation)
{
  using namespace units::literals;

  const cp::CtrvState state{0_m, 0_m, 0_mps, cp::Angle(0_rad), 1_rad_per_s};
  const auto next_state{cp::get_next_state(state, 0.5_s)};
  const cp::CtrvState expected_state{0_m, 0_m, 0_mps, cp::Angle(0.5_rad), 1_rad_per_s};

  EXPECT_TRUE(cp::utils::almost_equal(next_state, expected_state));
}

/**
 * Test CTRV get_next_state function against pure translation
 */
TEST(TestCtrvModel, NextStatePureTranslation)
{
  using namespace units::literals;

  const cp::CtrvState state{0_m, 0_m, 1_mps, cp::Angle(0_rad), 0_rad_per_s};
  const auto next_state{cp::get_next_state(state, 0.5_s)};
  const cp::CtrvState expected_state{0.5_m, 0_m, 1_mps, cp::Angle(0_rad), 0_rad_per_s};

  EXPECT_TRUE(cp::utils::almost_equal(next_state, expected_state));
}

/**
 * Test CTRV get_next_state function against mixed rotation and translation
 */
TEST(TestCtrvModel, NextStateRotationAndTranslation)
{
  using namespace units::literals;

  const cp::CtrvState state{0_m, 0_m, 1_mps, cp::Angle(0_rad), 1_rad_per_s};
  const auto next_state{cp::get_next_state(state, 0.5_s)};
  const cp::CtrvState expected_state{
    0.479425539_m, 0.122417438_m, 1_mps, cp::Angle(0.5_rad), 1_rad_per_s};

  EXPECT_TRUE(
    cp::utils::almost_equal(cp::utils::round_to_decimal_place(next_state, 9), expected_state));
}

TEST(TestCtrvModel, NextStateStochastic)
{
  using namespace units::literals;

  const cp::CtrvState state{0_m, 0_m, 1_mps, cp::Angle(0_rad), 1_rad_per_s};
  constexpr cp::CtrvProcessNoise noise{1_mps_sq, 1_rad_per_s_sq};
  const auto next_state{cp::get_next_state(state, 0.5_s, noise)};
  const cp::CtrvState expected_state{
    0.604425539_m, 0.122417438_m, 1.5_mps, cp::Angle(0.625_rad), 1.5_rad_per_s};

  EXPECT_TRUE(
    cp::utils::almost_equal(cp::utils::round_to_decimal_place(next_state, 9), expected_state));
}

TEST(TestCtrvModel, Equality)
{
  using namespace units::literals;

  const cp::CtrvState state1{
    1.23_m, 2.435_m, 5544_mps, cp::Angle(34656.6543_rad), 595633.555_rad_per_s};
  const cp::CtrvState state2{1.2_m, 20.45_m, 4_mps, cp::Angle(34656.65435_rad), 5953.55_rad_per_s};
  const cp::CtrvState state3{
    1_m, 2_m, 4_mps, cp::Angle(3_rad), 1.000000000000000000000000001_rad_per_s};
  const cp::CtrvState state4{1_m, 2_m, 4_mps, cp::Angle(3_rad), 1_rad_per_s};

  EXPECT_TRUE(cp::utils::almost_equal(state1, state1));
  EXPECT_FALSE(cp::utils::almost_equal(state1, state2));
  EXPECT_TRUE(cp::utils::almost_equal(state3, state4));
}
