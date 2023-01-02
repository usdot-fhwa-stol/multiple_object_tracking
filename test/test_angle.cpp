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

#include <gtest/gtest.h>
#include <cooperative_perception/angle.hpp>
#include <units.h>
#include <boost/math/constants/constants.hpp>

using namespace units::literals;

namespace cp = cooperative_perception;

/**
 * Test CTRV nextState function against pure rotation
 */
TEST(TestAngle, AngleArithmeticOperations)
{
  // Define some known angles
  const auto angle_pi{ cp::Angle{ 3.141592654_rad } };
  const auto angle_pi_2{ cp::Angle{ 1.570796327_rad } };
  const auto angle_pi_4{ cp::Angle{ 0.785398163_rad } };
  const auto angle_pi_6{ cp::Angle{ 0.523598776_rad } };

  // Perform additive and scalar operations
  auto res1 = angle_pi / 2.0;
  auto res2 = angle_pi_4 * 2.0;
  auto res3 = angle_pi + angle_pi_2;
  auto res4 = angle_pi - angle_pi_4;
  auto res5 = angle_pi * 2.2;

  // Set tolerance for these tests
  constexpr double tolerance = 1.e-8;

  EXPECT_NEAR(units::unit_cast<double>(res1.get_angle()), boost::math::double_constants::half_pi, tolerance);
  EXPECT_NEAR(units::unit_cast<double>(res2.get_angle()), boost::math::double_constants::half_pi, tolerance);
  EXPECT_NEAR(units::unit_cast<double>(res3.get_angle()), 3 * boost::math::double_constants::half_pi, tolerance);
  EXPECT_NEAR(units::unit_cast<double>(res4.get_angle()), (3.0 / 4.0) * boost::math::double_constants::pi, tolerance);
  EXPECT_NEAR(units::unit_cast<double>(res5.get_angle()), 0.2 * boost::math::double_constants::pi, tolerance);
}

TEST(TestAngle, AngleBooleanOperations)
{
  const auto angle_pi{ cp::Angle{ 3.141592654_rad } };
  const auto angle_pi_2{ cp::Angle{ 1.570796327_rad } };

  EXPECT_TRUE(cp::utils::almostEqual(angle_pi, angle_pi));
  EXPECT_FALSE(cp::utils::almostEqual(angle_pi, angle_pi_2));
  EXPECT_TRUE(cp::utils::almostEqual(angle_pi_2, angle_pi - angle_pi_2));
}
