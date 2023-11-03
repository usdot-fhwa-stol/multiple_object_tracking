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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <units.h>

#include <boost/math/constants/constants.hpp>
#include <multiple_object_tracking/angle.hpp>
#include <multiple_object_tracking/units.hpp>

using namespace units::literals;

namespace cp = multiple_object_tracking;

/**
 * Test CTRV get_next_state function against pure rotation
 */
TEST(TestAngle, AngleArithmeticOperations)
{
  // Define some known angles
  const auto angle_pi{cp::Angle{3.141592654_rad}};
  const auto angle_pi_2{cp::Angle{1.570796327_rad}};
  const auto angle_pi_4{cp::Angle{0.785398163_rad}};
  const auto angle_pi_6{cp::Angle{0.523598776_rad}};

  // Perform additive and scalar operations
  auto res1 = cp::remove_units((angle_pi / 2.0).get_angle());
  auto res2 = cp::remove_units((angle_pi_4 * 2.0).get_angle());
  auto res3 = cp::remove_units((angle_pi + angle_pi_2).get_angle());
  auto res4 = cp::remove_units((angle_pi - angle_pi_4).get_angle());
  auto res5 = cp::remove_units((angle_pi * 2.2).get_angle());

  // Set tolerance for these tests
  constexpr double tolerance = 1.e-8;

  EXPECT_NEAR(res1, boost::math::double_constants::half_pi, tolerance);
  EXPECT_NEAR(res2, boost::math::double_constants::half_pi, tolerance);
  EXPECT_NEAR(res3, 3 * boost::math::double_constants::half_pi, tolerance);
  EXPECT_NEAR(res4, (3.0 / 4.0) * boost::math::double_constants::pi, tolerance);
  EXPECT_NEAR(res5, 0.2 * boost::math::double_constants::pi, tolerance);
}

TEST(TestAngle, AngleBooleanOperations)
{
  static constexpr double tolerance{1.e-8};

  const auto angle_pi{cp::Angle{3.141592654_rad}};
  const auto angle_pi_2{cp::Angle{1.570796327_rad}};
  const auto angle_difference{angle_pi - angle_pi_2};

  const auto angle_pi_raw{cp::remove_units(angle_pi.get_angle())};
  const auto angle_pi_2_raw{cp::remove_units(angle_pi_2.get_angle())};
  const auto angle_difference_raw{cp::remove_units(angle_difference.get_angle())};

  using ::testing::DoubleNear;
  using ::testing::Not;

  EXPECT_NEAR(angle_pi_raw, angle_pi_raw, tolerance);
  EXPECT_THAT(angle_pi_raw, Not(DoubleNear(angle_pi_2_raw, tolerance)));
  EXPECT_NEAR(angle_pi_2_raw, angle_difference_raw, tolerance);
};
