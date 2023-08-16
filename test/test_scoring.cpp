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

#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/dynamic_object.hpp>
#include <cooperative_perception/scoring.hpp>

namespace cp = cooperative_perception;

using DetectionVariant = std::variant<cp::CtrvDetection, cp::CtraDetection>;
using TrackVariant = std::variant<cp::CtrvTrack, cp::CtraTrack>;

TEST(TestScoring, CtrvEuclideanDistance)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>;
  using TestTrack = cp::Track<cp::CtrvState, cp::CtrvStateCovariance>;

  const auto detection =
    TestDetection{.state{cp::CtrvState{1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s}}};

  const auto track =
    TestTrack{.state{cp::CtrvState{6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s}}};

  const auto score = cp::euclidean_distance_score(detection, track);
  ASSERT_TRUE(score.has_value());
  EXPECT_FLOAT_EQ(score.value(), 7.0710678118654755F);
}

TEST(TestScoring, CtrvMahalanobisDistance)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>;
  using TestTrack = cp::Track<cp::CtrvState, cp::CtrvStateCovariance>;

  const auto detection =
    TestDetection{.state{cp::CtrvState{1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s}}};

  const auto track = TestTrack{
    .state{cp::CtrvState{5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s}},
    .covariance{cp::CtrvStateCovariance{
      {0.0043, -0.0013, 0.0030, -0.0022, -0.0020},
      {-0.0013, 0.0077, 0.0011, 0.0071, 0.0060},
      {0.0030, 0.0011, 0.0054, 0.0007, 0.0008},
      {-0.0022, 0.0071, 0.0007, 0.0098, 0.0100},
      {-0.0020, 0.0060, 0.0008, 0.0100, 0.0123}}}};

  const auto mahalanobis_dist =
    cp::mahalanobis_distance(track.state, track.covariance, detection.state);
  EXPECT_FLOAT_EQ(mahalanobis_dist, 74.37377728947332F);
}

TEST(TestScoring, CtraEuclideanDistance)
{
  using namespace units::literals;

  const auto detection = cp::Detection<cp::CtraState, cp::CtraStateCovariance>{
    units::time::second_t{0},
    cp::CtraState{1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s, 6_mps_sq}};

  const auto track = cp::Track<cp::CtraState, cp::CtraStateCovariance>{
    units::time::second_t{0},
    cp::CtraState{6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s, 12_mps_sq}};

  const auto score = cp::euclidean_distance_score(detection, track);
  ASSERT_TRUE(score.has_value());
  EXPECT_FLOAT_EQ(score.value(), 7.0710678118654755);
}

TEST(TestScoring, CtraMahalanobisDistance)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtraState, cp::CtraStateCovariance>;
  using TestTrack = cp::Track<cp::CtraState, cp::CtraStateCovariance>;

  const auto detection =
    TestDetection{.state{cp::CtraState{1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s, 6_mps_sq}}};

  const auto track = TestTrack{
    .state{cp::CtraState{6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s, 12_mps_sq}},
    .covariance{cp::CtraStateCovariance{
      {0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5},
      {-0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123},
      {0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34},
      {-0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009},
      {-0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021},
      {0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701},
    }}};

  const auto mahalanobis_dist =
    cp::mahalanobis_distance(track.state, track.covariance, detection.state);
  EXPECT_FLOAT_EQ(mahalanobis_dist, 122.3575692494651);
}

TEST(TestScoring, TrackToDetectionScoringEuclidean)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtraState, cp::CtraStateCovariance>;
  using TestTrack = cp::Track<cp::CtraState, cp::CtraStateCovariance>;

  const std::vector<TrackVariant> tracks{
    TestTrack{
      .state{cp::CtraState{6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s, 12_mps_sq}},
      .uuid{"test_track1"}},
    TestTrack{
      .state{cp::CtraState{8_m, 2_m, 3_mps, cp::Angle(1_rad), 12_rad_per_s, 11_mps_sq}},
      .uuid{"test_track2"}},
    cp::Track<cp::CtrvState, cp::CtrvStateCovariance>{
      .state{1_m, 1_m, 1_mps, cp::Angle(1_rad), 1_rad_per_s}, .uuid{"test_track3"}}};

  const std::vector<DetectionVariant> detections{
    TestDetection{
      .state{cp::CtraState{1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s, 6_mps_sq}},
      .uuid{"test_detection1"}},
    TestDetection{
      .state{cp::CtraState{2_m, 3_m, 6_mps, cp::Angle(2_rad), 20_rad_per_s, 9_mps_sq}},
      .uuid{"test_detection2"}},
    cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>{
      .state{1_m, 1_m, 1_mps, cp::Angle(1_rad), 1_rad_per_s}, .uuid{"test_detection3"}}};

  const auto scores =
    cp::score_tracks_and_detections(tracks, detections, cp::euclidean_distance_score);

  const cp::ScoreMap expected_scores{
    {std::pair{"test_track1", "test_detection1"}, 7.0710678},
    {std::pair{"test_track1", "test_detection2"}, 5.7445626},
    {std::pair{"test_track2", "test_detection1"}, 7.2801099},
    {std::pair{"test_track2", "test_detection2"}, 6.1644139},
    {std::pair{"test_track3", "test_detection3"}, 0.0},
  };

  EXPECT_EQ(std::size(scores), std::size(expected_scores));

  for (const auto & [key, value] : scores) {
    EXPECT_FLOAT_EQ(expected_scores.at(key), value);
  }
}

TEST(TestScoring, TrackToDetectionScoringMahalanobis)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtraState, cp::CtraStateCovariance>;
  using TestTrack = cp::Track<cp::CtraState, cp::CtraStateCovariance>;

  const std::vector<TrackVariant> tracks{
    TestTrack{
      .state{cp::CtraState{6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s, 12_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5},
        {-0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123},
        {0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34},
        {-0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009},
        {-0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021},
        {0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701},
      }},
      .uuid{"test_track1"}},
    TestTrack{
      .state{cp::CtraState{8_m, 2_m, 3_mps, cp::Angle(1_rad), 12_rad_per_s, 11_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5},
        {-0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123},
        {0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34},
        {-0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009},
        {-0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021},
        {0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701},
      }},
      .uuid{"test_track2"}},
    cp::Track<cp::CtrvState, cp::CtrvStateCovariance>{
      .state{1_m, 1_m, 1_mps, cp::Angle(1_rad), 1_rad_per_s},
      .covariance{cp::CtrvStateCovariance{
        {0.0043, -0.0013, 0.0030, -0.0022, -0.0020},
        {-0.0013, 0.0077, 0.0011, 0.0071, 0.0060},
        {0.0030, 0.0011, 0.0054, 0.0007, 0.0008},
        {-0.0022, 0.0071, 0.0007, 0.0098, 0.0100},
        {-0.0020, 0.0060, 0.0008, 0.0100, 0.0123}}},
      .uuid{"test_track3"}}};

  const std::vector<DetectionVariant> detections{
    TestDetection{
      .state{cp::CtraState{1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s, 6_mps_sq}},
      .uuid{"test_detection1"}},
    TestDetection{
      .state{cp::CtraState{2_m, 3_m, 6_mps, cp::Angle(2_rad), 20_rad_per_s, 9_mps_sq}},
      .uuid{"test_detection2"}},
    cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>{
      .state{1_m, 1_m, 1_mps, cp::Angle(1_rad), 1_rad_per_s}, .uuid{"test_detection3"}}};

  const auto scores =
    cp::score_tracks_and_detections(tracks, detections, cp::mahalanobis_distance_score);

  const cp::ScoreMap expected_scores{
    {std::pair{"test_track1", "test_detection1"}, 122.35757},
    {std::pair{"test_track1", "test_detection2"}, 90.688416},
    {std::pair{"test_track2", "test_detection1"}, 109.70312},
    {std::pair{"test_track2", "test_detection2"}, 95.243896},
    {std::pair{"test_track3", "test_detection3"}, 0.0},
  };

  EXPECT_EQ(std::size(scores), std::size(expected_scores));

  for (const auto & [key, value] : scores) {
    EXPECT_FLOAT_EQ(expected_scores.at(key), value);
  }
}
