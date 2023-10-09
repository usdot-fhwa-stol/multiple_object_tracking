#include <gtest/gtest.h>

#include <cooperative_perception/dynamic_object.hpp>
#include <cooperative_perception/json_parsing.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

TEST(JsonParsing, CtrvDetection)
{
  std::ifstream file{"data/test_json_parsing_ctrv_detection.json"};
  ASSERT_TRUE(file);
  const auto data = nlohmann::json::parse(file);

  const auto detection{data.template get<cooperative_perception::CtrvDetection>()};

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
