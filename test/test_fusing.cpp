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

#include <Eigen/Dense>
#include <cooperative_perception/angle.hpp>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/dynamic_object.hpp>
#include <cooperative_perception/fusing.hpp>
#include <cooperative_perception/json_parsing.hpp>
#include <cooperative_perception/track_matching.hpp>
#include <cooperative_perception/utils.hpp>

namespace cp = cooperative_perception;

using DetectionVariant = std::variant<cp::CtrvDetection, cp::CtraDetection>;
using TrackVariant = std::variant<cp::CtrvTrack, cp::CtraTrack>;

/**
 * Test the generate_weight function
 */
TEST(TestFusing, GenerateWeight)
{
  // Declaring initial covariances
  Eigen::Matrix3f covariance1;
  covariance1 << 4, 0, 0, 0, 5, 0, 0, 0, 6;

  Eigen::Matrix3f covariance2;
  covariance2 << 7, 0, 0, 0, 8, 0, 0, 0, 9;

  // Expected values
  const auto expected_weight{0.5895104895104895};

  // Call the function under test
  const auto result_weight{cp::generate_weight(covariance1.inverse(), covariance2.inverse())};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(expected_weight, result_weight));
}

/**
 * Test the compute_covariance_intersection function using purely Eigen matrices and vectors
 */
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

  // Compute inverse of the covariances
  const auto inverse_covariance1{covariance1.inverse()};
  const auto inverse_covariance2{covariance2.inverse()};

  // Generate weight for CI function
  const auto weight{cp::generate_weight(inverse_covariance1, inverse_covariance2)};

  // Call the function under test
  const auto [result_mean, result_covariance]{cp::compute_covariance_intersection(
    mean1, inverse_covariance1, mean2, inverse_covariance2, weight)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(expected_mean, result_mean));
  EXPECT_TRUE(cp::utils::almost_equal(expected_covariance, result_covariance));
}

/**
 * Test fusing CTRV tracks and detections
 */
TEST(TestFusing, CtrvTracksAndDetections)
{
  using namespace units::literals;

  // Declaring initial values
  const cp::AssociationMap associations{
    {"track1", {"detection3"}}, {"track2", {"detection2"}}, {"track3", {"detection1"}}};

  std::ifstream tracks_file{"data/test_fusing_ctrv_tracks_and_detections_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{cp::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_ctrv_tracks_and_detections_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{cp::detections_from_json_file<DetectionVariant>(detections_file)};

  std::ifstream expected_tracks_file{
    "data/test_fusing_ctrv_tracks_and_detections_expected_tracks.json"};
  ASSERT_TRUE(expected_tracks_file);
  const auto expected_tracks{cp::tracks_from_json_file<TrackVariant>(expected_tracks_file)};

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
}

/**
 * Test fusing CTRA tracks and detections
 */
TEST(TestFusing, CtraTracksAndDetections)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection3"}}, {"track2", {"detection2"}}, {"track3", {"detection1"}}};

  std::ifstream tracks_file{"data/test_fusing_ctra_tracks_and_detections_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{cp::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_ctra_tracks_and_detections_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{cp::detections_from_json_file<DetectionVariant>(detections_file)};

  std::ifstream expected_tracks_file{
    "data/test_fusing_ctra_tracks_and_detections_expected_tracks.json"};
  ASSERT_TRUE(expected_tracks_file);
  const auto expected_tracks{cp::tracks_from_json_file<TrackVariant>(expected_tracks_file)};

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
}

/**
 * Test fusing a mixed vector of CTRV and CTRA tracks and detections
 */
TEST(TestFusing, MixedTracksAndDetections)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection3"}}, {"track2", {"detection2"}}, {"track3", {"detection1"}}};

  std::ifstream tracks_file{"data/test_fusing_mixed_tracks_and_detections_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{cp::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_mixed_tracks_and_detections_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{cp::detections_from_json_file<DetectionVariant>(detections_file)};

  std::ifstream expected_tracks_file{
    "data/test_fusing_mixed_tracks_and_detections_expected_tracks.json"};
  ASSERT_TRUE(expected_tracks_file);
  const auto expected_tracks{cp::tracks_from_json_file<TrackVariant>(expected_tracks_file)};

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
}

/**
 * Test fusing when no matching uuids are found
 */
TEST(TestFusing, UnmatchedAssociations)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection4"}}, {"track2", {"detection5"}}, {"track3", {"detection6"}}};

  std::ifstream tracks_file{"data/test_fusing_unmatched_associations_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{cp::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_unmatched_associations_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{cp::detections_from_json_file<DetectionVariant>(detections_file)};

  // Expected values
  std::vector<TrackVariant> expected_tracks;

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
}

/**
 * Test fusing when only a few associations are matched
 */
TEST(TestFusing, PartialMatchedAssociations)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection4"}}, {"track2", {"detection2"}}, {"track3", {"detection1"}}};

  std::ifstream tracks_file{"data/test_fusing_partial_matched_associations_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{cp::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_partial_matched_associations_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{cp::detections_from_json_file<DetectionVariant>(detections_file)};

  std::ifstream expected_tracks_file{
    "data/test_fusing_partial_matched_associations_expected_tracks.json"};
  ASSERT_TRUE(expected_tracks_file);
  const auto expected_tracks{cp::tracks_from_json_file<TrackVariant>(expected_tracks_file)};

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
};
