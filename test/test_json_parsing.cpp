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

#include <gtest/gtest.h>

#include <multiple_object_tracking/dynamic_object.hpp>
#include <multiple_object_tracking/json_parsing.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

TEST(JsonParsing, CtrvDetection)
{
  std::ifstream file{"data/test_json_parsing_ctrv_detection.json"};
  ASSERT_TRUE(file);
  const auto data = nlohmann::json::parse(file);

  const auto detection{data.template get<multiple_object_tracking::CtrvDetection>()};

  Eigen::Matrix<float, 5, 5> expected_covariance;
  expected_covariance << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25;

  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.timestamp), 1.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.position_x), 2.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.position_y), 3.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.velocity), 4.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.yaw.get_angle()), 5.0);
  EXPECT_DOUBLE_EQ(units::unit_cast<double>(detection.state.yaw_rate), 6.0);
  EXPECT_EQ(detection.covariance, expected_covariance);
  EXPECT_EQ(detection.uuid.value(), "ABC");
}
