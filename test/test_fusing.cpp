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
 * Originally developed for Leidos by the Human and Intelligent Vehicle
 * Ensembles (HIVE) Lab at Virginia Commonwealth University (VCU).
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <multiple_object_tracking/angle.hpp>
#include <multiple_object_tracking/ctra_model.hpp>
#include <multiple_object_tracking/ctrv_model.hpp>
#include <multiple_object_tracking/dynamic_object.hpp>
#include <multiple_object_tracking/fusing.hpp>
#include <multiple_object_tracking/json_parsing.hpp>
#include <multiple_object_tracking/track_matching.hpp>
#include <multiple_object_tracking/utils.hpp>

#include "multiple_object_tracking/test/gmock_matchers.hpp"

namespace mot = multiple_object_tracking;

using DetectionVariant = std::variant<mot::CtrvDetection, mot::CtraDetection>;
using TrackVariant = std::variant<mot::CtrvTrack, mot::CtraTrack>;

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
  const auto result_weight{mot::generate_weight(covariance1.inverse(), covariance2.inverse())};

  // Check that function returns expected value
  EXPECT_FLOAT_EQ(result_weight, expected_weight);
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
  const auto weight{mot::generate_weight(inverse_covariance1, inverse_covariance2)};

  // Call the function under test
  const auto [result_mean, result_covariance]{mot::compute_covariance_intersection(
    mean1, inverse_covariance1, mean2, inverse_covariance2, weight)};

  static constexpr double tolerance{1.e-6};

  EXPECT_THAT(result_mean, EigenMatrixNear(expected_mean, tolerance));
  EXPECT_THAT(result_covariance, EigenMatrixNear(expected_covariance, tolerance));
}

/**
 * Test fusing CTRV tracks and detections
 */
TEST(TestFusing, CtrvTracksAndDetections)
{
  using namespace units::literals;

  const mot::AssociationMap associations{
    {mot::Uuid{"track1"}, {mot::Uuid{"detection3"}}},
    {mot::Uuid{"track2"}, {mot::Uuid{"detection2"}}},
    {mot::Uuid{"track3"}, {mot::Uuid{"detection1"}}}};

  std::ifstream tracks_file{"data/test_fusing_ctrv_tracks_and_detections_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{mot::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_ctrv_tracks_and_detections_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{mot::detections_from_json_file<DetectionVariant>(detections_file)};

  std::ifstream expected_tracks_file{
    "data/test_fusing_ctrv_tracks_and_detections_expected_tracks.json"};
  ASSERT_TRUE(expected_tracks_file);
  const auto expected_tracks{mot::tracks_from_json_file<TrackVariant>(expected_tracks_file)};

  const auto result_tracks{
    mot::fuse_associations(associations, tracks, detections, mot::covariance_intersection_visitor)};

  ASSERT_EQ(std::size(result_tracks), std::size(expected_tracks));

  using ::testing::Pointwise;

  EXPECT_THAT(result_tracks, Pointwise(PointwiseTrackNear(1e-4), expected_tracks));
}

/**
 * Test fusing CTRA tracks and detections
 */
TEST(TestFusing, CtraTracksAndDetections)
{
  using namespace units::literals;

  mot::AssociationMap associations{
    {mot::Uuid{"track1"}, {mot::Uuid{"detection3"}}},
    {mot::Uuid{"track2"}, {mot::Uuid{"detection2"}}},
    {mot::Uuid{"track3"}, {mot::Uuid{"detection1"}}}};

  std::ifstream tracks_file{"data/test_fusing_ctra_tracks_and_detections_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{mot::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_ctra_tracks_and_detections_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{mot::detections_from_json_file<DetectionVariant>(detections_file)};

  std::ifstream expected_tracks_file{
    "data/test_fusing_ctra_tracks_and_detections_expected_tracks.json"};
  ASSERT_TRUE(expected_tracks_file);
  const auto expected_tracks{mot::tracks_from_json_file<TrackVariant>(expected_tracks_file)};

  const auto result_tracks{
    mot::fuse_associations(associations, tracks, detections, mot::covariance_intersection_visitor)};

  ASSERT_EQ(std::size(result_tracks), std::size(expected_tracks));

  using ::testing::Pointwise;

  EXPECT_THAT(result_tracks, Pointwise(PointwiseTrackNear(1e-4), expected_tracks));
}

/**
 * Test fusing a mixed vector of CTRV and CTRA tracks and detections
 */
TEST(TestFusing, MixedTracksAndDetections)
{
  using namespace units::literals;

  mot::AssociationMap associations{
    {mot::Uuid{"track1"}, {mot::Uuid{"detection3"}}},
    {mot::Uuid{"track2"}, {mot::Uuid{"detection2"}}},
    {mot::Uuid{"track3"}, {mot::Uuid{"detection1"}}}};

  std::ifstream tracks_file{"data/test_fusing_mixed_tracks_and_detections_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{mot::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_mixed_tracks_and_detections_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{mot::detections_from_json_file<DetectionVariant>(detections_file)};

  std::ifstream expected_tracks_file{
    "data/test_fusing_mixed_tracks_and_detections_expected_tracks.json"};
  ASSERT_TRUE(expected_tracks_file);
  const auto expected_tracks{mot::tracks_from_json_file<TrackVariant>(expected_tracks_file)};

  const auto result_tracks{
    mot::fuse_associations(associations, tracks, detections, mot::covariance_intersection_visitor)};

  using ::testing::Pointwise;

  EXPECT_THAT(result_tracks, Pointwise(PointwiseTrackNear(1e-4), expected_tracks));
}

/**
 * Test fusing when no matching uuids are found
 */
TEST(TestFusing, UnmatchedAssociations)
{
  using namespace units::literals;

  // Declaring initial values
  mot::AssociationMap associations{
    {mot::Uuid{"track1"}, {mot::Uuid{"detection4"}}},
    {mot::Uuid{"track2"}, {mot::Uuid{"detection5"}}},
    {mot::Uuid{"track3"}, {mot::Uuid{"detection6"}}}};

  std::ifstream tracks_file{"data/test_fusing_unmatched_associations_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{mot::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_unmatched_associations_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{mot::detections_from_json_file<DetectionVariant>(detections_file)};

  std::vector<TrackVariant> expected_tracks;

  const auto result_tracks{
    mot::fuse_associations(associations, tracks, detections, mot::covariance_intersection_visitor)};

  using ::testing::Pointwise;

  EXPECT_THAT(result_tracks, Pointwise(PointwiseTrackNear(1e-5), expected_tracks));
}

/**
 * Test fusing when only a few associations are matched
 */
TEST(TestFusing, PartialMatchedAssociations)
{
  using namespace units::literals;

  mot::AssociationMap associations{
    {mot::Uuid{"track1"}, {mot::Uuid{"detection4"}}},
    {mot::Uuid{"track2"}, {mot::Uuid{"detection2"}}},
    {mot::Uuid{"track3"}, {mot::Uuid{"detection1"}}}};

  std::ifstream tracks_file{"data/test_fusing_partial_matched_associations_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{mot::tracks_from_json_file<TrackVariant>(tracks_file)};

  std::ifstream detections_file{"data/test_fusing_partial_matched_associations_detections.json"};
  ASSERT_TRUE(detections_file);
  const auto detections{mot::detections_from_json_file<DetectionVariant>(detections_file)};

  std::ifstream expected_tracks_file{
    "data/test_fusing_partial_matched_associations_expected_tracks.json"};
  ASSERT_TRUE(expected_tracks_file);
  const auto expected_tracks{mot::tracks_from_json_file<TrackVariant>(expected_tracks_file)};

  const auto result_tracks{
    mot::fuse_associations(associations, tracks, detections, mot::covariance_intersection_visitor)};

  using ::testing::Pointwise;

  EXPECT_THAT(result_tracks, Pointwise(PointwiseTrackNear(1e-4), expected_tracks));
};
