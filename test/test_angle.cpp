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

using namespace units::literals;

namespace cp = cooperative_perception;

/**
 * Test CTRV nextState function against pure rotation
 */
TEST(TestAngle, AngleArithmeticOperations)
{
  // Define some known angles
  const auto pi{ cp::Angle{ 3.141592654_rad } };
  const auto pi_2{ cp::Angle{ 1.570796327_rad } };
  const auto pi_4{ cp::Angle{ 0.785398163_rad } };
  const auto pi_6{ cp::Angle{ 0.523598776_rad } };

  // Perform additive and scalar operations
  auto res1 = pi / 4.0;
  auto res2 = pi_4 * 2.0;
  auto res3 = pi + pi_2;
  auto res4 = pi - pi_4;
  auto res5 = pi * 2.2;

  EXPECT_TRUE(cp::utils::almostEqual(units::unit_cast<double>(res1.get_angle()), M_PI_4));
  EXPECT_TRUE(cp::utils::almostEqual(units::unit_cast<double>(res2.get_angle()), M_PI_2));
  EXPECT_TRUE(cp::utils::almostEqual(units::unit_cast<double>(res3.get_angle()), 3 * M_PI_2));
  EXPECT_TRUE(cp::utils::almostEqual(units::unit_cast<double>(res4.get_angle()), 3 * M_PI_4));
  EXPECT_TRUE(cp::utils::almostEqual(units::unit_cast<double>(res5.get_angle()), 0.2 * M_PI));
}
