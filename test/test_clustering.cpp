#include <gtest/gtest.h>

#include <multiple_object_tracking/clustering.hpp>
#include <multiple_object_tracking/ctra_model.hpp>
#include <multiple_object_tracking/ctrv_model.hpp>
#include <multiple_object_tracking/json_parsing.hpp>
#include <fstream>

namespace mot = multiple_object_tracking;

TEST(TestCluster, Empty)
{
  mot::Cluster<mot::CtraDetection> cluster;

  ASSERT_TRUE(cluster.is_empty());
  EXPECT_THROW((void)cluster.get_centroid(), std::runtime_error);
}

TEST(TestCluser, Simple)
{
  mot::Cluster<mot::CtrvDetection> cluster;

  mot::CtrvDetection detection;
  detection.state.position_x = units::length::meter_t{1.0};
  detection.uuid = mot::Uuid{"detection1"};

  cluster.add_detection(detection);
  auto centroid{cluster.get_centroid()};

  // This will use the operator== overload, which for units library is almost-equality
  EXPECT_EQ(centroid.position_x, units::length::meter_t{1.0});
  EXPECT_EQ(centroid.position_y, units::length::meter_t{0.0});
  EXPECT_EQ(centroid.velocity, units::velocity::meters_per_second_t{0.0});
  EXPECT_EQ(centroid.yaw.get_angle(), units::angle::radian_t{0.0});
  EXPECT_EQ(centroid.yaw_rate, units::angular_velocity::radians_per_second_t{0.0});

  detection.state.position_x = units::length::meter_t{3.0};
  detection.uuid = mot::Uuid{"detection2"};
  cluster.add_detection(detection);
  centroid = cluster.get_centroid();

  // This will use the operator== overload, which for units library is almost-equality
  EXPECT_EQ(centroid.position_x, units::length::meter_t{2.0});
  EXPECT_EQ(centroid.position_y, units::length::meter_t{0.0});
  EXPECT_EQ(centroid.velocity, units::velocity::meters_per_second_t{0.0});
  EXPECT_EQ(centroid.yaw.get_angle(), units::angle::radian_t{0.0});
  EXPECT_EQ(centroid.yaw_rate, units::angular_velocity::radians_per_second_t{0.0});

  detection.state.position_x = units::length::meter_t{5.0};
  detection.uuid = mot::Uuid{"detection3"};
  cluster.add_detection(detection);
  centroid = cluster.get_centroid();

  // This will use the operator== overload, which for units library is almost-equality
  EXPECT_EQ(centroid.position_x, units::length::meter_t{3.0});
  EXPECT_EQ(centroid.position_y, units::length::meter_t{0.0});
  EXPECT_EQ(centroid.velocity, units::velocity::meters_per_second_t{0.0});
  EXPECT_EQ(centroid.yaw.get_angle(), units::angle::radian_t{0.0});
  EXPECT_EQ(centroid.yaw_rate, units::angular_velocity::radians_per_second_t{0.0});

  cluster.remove_detection(mot::Uuid{"detection3"});
  centroid = cluster.get_centroid();

  // This will use the operator== overload, which for units library is almost-equality
  EXPECT_EQ(centroid.position_x, units::length::meter_t{2.0});
  EXPECT_EQ(centroid.position_y, units::length::meter_t{0.0});
  EXPECT_EQ(centroid.velocity, units::velocity::meters_per_second_t{0.0});
  EXPECT_EQ(centroid.yaw.get_angle(), units::angle::radian_t{0.0});
  EXPECT_EQ(centroid.yaw_rate, units::angular_velocity::radians_per_second_t{0.0});
}

TEST(TestClustering, Disjoint)
{
  std::ifstream detections_file{"data/test_clustering_disjoint.json"};
  ASSERT_TRUE(detections_file);

  const auto detections{mot::detections_from_json_file<mot::CtrvDetection>(detections_file)};

  EXPECT_EQ(std::size(mot::cluster_detections(detections, 1.0)), 2U);
  EXPECT_EQ(std::size(mot::cluster_detections(detections, 3.0)), 1U);
}

TEST(TestClustering, Overlapping)
{
  std::ifstream detections_file{"data/test_clustering_overlapping.json"};
  ASSERT_TRUE(detections_file);

  const auto detections{mot::detections_from_json_file<mot::CtrvDetection>(detections_file)};

  EXPECT_EQ(std::size(mot::cluster_detections(detections, 1.0)), 1U);
  EXPECT_EQ(std::size(mot::cluster_detections(detections, 3.0)), 1U);
}
