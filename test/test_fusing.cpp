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

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <cooperative_perception/angle.hpp>
#include <cooperative_perception/track.hpp>
#include <cooperative_perception/utils.hpp>
#include <cooperative_perception/fusing.hpp>
#include <cooperative_perception/detection.hpp>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/track_matching.hpp>
#include <cooperative_perception/unscented_transform.hpp>

namespace cp = cooperative_perception;

TEST(TestFusing, GenerateWeight)
{
  // Declaring initial covariances
  Eigen::Matrix3f covariance1;
  covariance1 << 4, 0, 0, 0, 5, 0, 0, 0, 6;

  Eigen::Matrix3f covariance2;
  covariance2 << 7, 0, 0, 0, 8, 0, 0, 0, 9;

  // Expected values
  const auto expected_weight{ 0.5895104895104895 };

  // Call the function under test
  const auto result_weight{ cp::generateWeight(covariance1.inverse(), covariance2.inverse()) };

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almostEqual(expected_weight, result_weight));
}

TEST(TestFusing, ComputeCovarianceIntersectionPureEigen)
{
  // Declaring initial means and covariances
  Eigen::Vector3f mean1(1, 2, 3);
  Eigen::Matrix3f covariance1;
  covariance1 << 4, 0, 0, 0, 5, 0, 0, 0, 6;

  Eigen::Vector3f mean2(4, 5, 6);
  Eigen::Matrix3f covariance2;
  covariance2 << 7, 0, 0, 0, 8, 0, 0, 0, 9;

  // Expected values
  Eigen::Vector3f expected_mean(1.85392169, 2.90970142, 3.95112071);
  Eigen::Matrix3f expected_covariance;
  expected_covariance << 4.85392169, 0, 0, 0, 5.90970142, 0, 0, 0, 6.95112071;

  // Generate weight for CI function
  const auto weight{ cp::generateWeight(covariance1.inverse(), covariance2.inverse()) };

  // Call the function under test
  const auto [result_mean,
              result_covariance]{ cp::computeCovarianceIntersection(mean1, covariance1, mean2, covariance2, weight) };

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almostEqual(expected_mean, result_mean));
  EXPECT_TRUE(cp::utils::almostEqual(expected_covariance, result_covariance));
}

TEST(TestFusing, Example)
{
  using namespace units::literals;

  cp::AssociationMap associations{ { "track1", { "detection3" } },
                                   { "track2", { "detection2" } },
                                   { "track3", { "detection1" } } };

  // I need detections, tracks
  std::vector<cp::DetectionVariant> detections{
    cp::CtrvDetection{
        .timestamp{ units::time::second_t{ 0 } },
        .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s } },
        .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                              { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                              { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                              { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                              { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } },
        .uuid{ "detection1" } },
    cp::CtrvDetection{
        .timestamp{ units::time::second_t{ 0 } },
        .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s } },
        .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                              { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                              { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                              { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                              { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } },
        .uuid{ "detection2" } },
    cp::CtrvDetection{
        .timestamp{ units::time::second_t{ 0 } },
        .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s } },
        .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                              { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                              { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                              { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                              { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } },
        .uuid{ "detection3" } }
  };

  std::vector<cp::TrackVariant> tracks{
    cp::CtrvTrack{ .timestamp{ units::time::second_t{ 0 } },
                   .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s } },
                   .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                                         { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                                         { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                                         { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                                         { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } },
                   .uuid{ "track1" } },
    cp::CtrvTrack{ .timestamp{ units::time::second_t{ 0 } },
                   .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s } },
                   .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                                         { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                                         { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                                         { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                                         { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } },
                   .uuid{ "track2" } },
    cp::CtrvTrack{ .timestamp{ units::time::second_t{ 0 } },
                   .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s } },
                   .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                                         { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                                         { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                                         { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                                         { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } },
                   .uuid{ "track3" } }
  };

  // call function
  const auto result_tracks =
      cp::fuseAssociations(associations, detections, tracks, cp::covariance_intersection_visitor);

  EXPECT_TRUE(true);
}
