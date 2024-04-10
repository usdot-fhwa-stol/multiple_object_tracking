/*
 * Copyright 2023 Leidos
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

#include <multiple_object_tracking/angle.hpp>
#include <multiple_object_tracking/ctrv_model.hpp>

namespace mot = multiple_object_tracking;

TEST(TestDetection, CtrvDetectionDefaultConstruction)
{
  using namespace units::literals;

  auto const detection = mot::Detection<mot::CtrvState, mot::CtrvStateCovariance>{
    units::time::second_t{0}, mot::CtrvState{0_m, 0_m, 0_mps, mot::Angle(0_rad), 0_rad_per_s},
    mot::CtrvStateCovariance{}, mot::Uuid{""}};

  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.timestamp), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.position_x), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.position_y), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.velocity), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.yaw.get_angle()), 0.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.yaw_rate), 0.0);
}

TEST(TestDetection, CtrvDetectionCustomConstruction)
{
  using namespace units::literals;

  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
  auto const detection = mot::Detection<mot::CtrvState, mot::CtrvStateCovariance>{
    units::time::second_t{1}, mot::CtrvState{1_m, 2_m, 3_mps, 4_rad, 5_rad_per_s},
    mot::CtrvStateCovariance{}, mot::Uuid{""}};
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.timestamp), 1.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.position_x), 1.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.position_y), 2.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.velocity), 3.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.yaw.get_angle()), 4.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.yaw_rate), 5.0);
}
