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

#include <gtest/gtest.h>
#include <units.h>

#include <multiple_object_tracking/ctra_model.hpp>
#include <multiple_object_tracking/ctrv_model.hpp>
#include <multiple_object_tracking/dynamic_object.hpp>
#include <multiple_object_tracking/json_parsing.hpp>
#include <multiple_object_tracking/scoring.hpp>
#include <fstream>

namespace mot = multiple_object_tracking;

using DetectionVariant = std::variant<mot::CtrvDetection, mot::CtraDetection>;
using TrackVariant = std::variant<mot::CtrvTrack, mot::CtraTrack>;

TEST(TestScoring, CtrvEuclideanDistance)
{
  using namespace units::literals;

  using TestDetection = mot::Detection<mot::CtrvState, mot::CtrvStateCovariance>;
  using TestTrack = mot::Track<mot::CtrvState, mot::CtrvStateCovariance>;

  const auto detection = TestDetection{
    .state{mot::CtrvState{1_m, 2_m, 3_mps, mot::Angle(3_rad), 5_rad_per_s}}, .uuid{mot::Uuid{""}}};

  const auto track = TestTrack{
    .state{mot::CtrvState{6_m, 7_m, 8_mps, mot::Angle(3_rad), 10_rad_per_s}}, .uuid{mot::Uuid{""}}};

  const auto score = mot::euclidean_distance_score(detection, track);
  ASSERT_TRUE(score.has_value());
  EXPECT_FLOAT_EQ(score.value(), 7.0710678118654755F);
}

TEST(TestScoring, CtrvMahalanobisDistance)
{
  using namespace units::literals;

  using TestDetection = mot::Detection<mot::CtrvState, mot::CtrvStateCovariance>;
  using TestTrack = mot::Track<mot::CtrvState, mot::CtrvStateCovariance>;

  const auto detection = TestDetection{
    .state{mot::CtrvState{1_m, 2_m, 3_mps, mot::Angle(3_rad), 5_rad_per_s}}, .uuid{mot::Uuid{""}}};

  mot::CtrvStateCovariance covariance;
  covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
    0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, -0.0020,
    0.0060, 0.0008, 0.0100, 0.0123;

  const auto track = TestTrack{
    .state{mot::CtrvState{5.7441_m, 1.3800_m, 2.2049_mps, mot::Angle(0.5015_rad), 0.3528_rad_per_s}},
    .covariance = covariance,
    .uuid{mot::Uuid{""}}};

  const auto mahalanobis_dist =
    mot::mahalanobis_distance(track.state, track.covariance, detection.state);
  EXPECT_FLOAT_EQ(mahalanobis_dist, 74.37377728947332F);
}

TEST(TestScoring, CtraEuclideanDistance)
{
  using namespace units::literals;

  const auto detection = mot::Detection<mot::CtraState, mot::CtraStateCovariance>{
    .timestamp{units::time::second_t{0}},
    .state{mot::CtraState{1_m, 2_m, 3_mps, mot::Angle(3_rad), 5_rad_per_s, 6_mps_sq}},
    .uuid{mot::Uuid{""}}};

  const auto track = mot::Track<mot::CtraState, mot::CtraStateCovariance>{
    .timestamp{units::time::second_t{0}},
    .state{mot::CtraState{6_m, 7_m, 8_mps, mot::Angle(3_rad), 10_rad_per_s, 12_mps_sq}},
    .uuid{mot::Uuid{""}}};

  const auto score = mot::euclidean_distance_score(detection, track);
  ASSERT_TRUE(score.has_value());
  EXPECT_FLOAT_EQ(score.value(), 7.0710678118654755);
}

TEST(TestScoring, CtraMahalanobisDistance)
{
  using namespace units::literals;

  using TestDetection = mot::Detection<mot::CtraState, mot::CtraStateCovariance>;
  using TestTrack = mot::Track<mot::CtraState, mot::CtraStateCovariance>;

  const auto detection = TestDetection{
    .state{mot::CtraState{1_m, 2_m, 3_mps, mot::Angle(3_rad), 5_rad_per_s, 6_mps_sq}},
    .uuid{mot::Uuid{""}}};

  mot::CtraStateCovariance covariance;
  covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5, -0.0013, 0.0077, 0.0011, 0.0071,
    0.0060, 0.123, 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34, -0.0022, 0.0071, 0.0007, 0.0098,
    0.0100, 0.009, -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021, 0.5, 0.123, -0.34, 0.009,
    0.0021, -0.8701;

  const auto track = TestTrack{
    .state{mot::CtraState{6_m, 7_m, 8_mps, mot::Angle(3_rad), 10_rad_per_s, 12_mps_sq}},
    .covariance = covariance,
    .uuid{mot::Uuid{""}}};

  const auto mahalanobis_dist =
    mot::mahalanobis_distance(track.state, track.covariance, detection.state);
  EXPECT_FLOAT_EQ(mahalanobis_dist, 122.3575692494651);
}

TEST(TestScoring, TrackToDetectionScoringEuclidean)
{
  using namespace units::literals;

  using TestDetection = mot::Detection<mot::CtraState, mot::CtraStateCovariance>;
  using TestTrack = mot::Track<mot::CtraState, mot::CtraStateCovariance>;

  const std::vector<TrackVariant> tracks{
    TestTrack{
      .state{mot::CtraState{6_m, 7_m, 8_mps, mot::Angle(3_rad), 10_rad_per_s, 12_mps_sq}},
      .uuid{mot::Uuid{"test_track1"}}},
    TestTrack{
      .state{mot::CtraState{8_m, 2_m, 3_mps, mot::Angle(1_rad), 12_rad_per_s, 11_mps_sq}},
      .uuid{mot::Uuid{"test_track2"}}},
    mot::Track<mot::CtrvState, mot::CtrvStateCovariance>{
      .state{1_m, 1_m, 1_mps, mot::Angle(1_rad), 1_rad_per_s}, .uuid{mot::Uuid{"test_track3"}}}};

  const std::vector<DetectionVariant> detections{
    TestDetection{
      .state{mot::CtraState{1_m, 2_m, 3_mps, mot::Angle(3_rad), 5_rad_per_s, 6_mps_sq}},
      .uuid{mot::Uuid{"test_detection1"}}},
    TestDetection{
      .state{mot::CtraState{2_m, 3_m, 6_mps, mot::Angle(2_rad), 20_rad_per_s, 9_mps_sq}},
      .uuid{mot::Uuid{"test_detection2"}}},
    mot::Detection<mot::CtrvState, mot::CtrvStateCovariance>{
      .state{1_m, 1_m, 1_mps, mot::Angle(1_rad), 1_rad_per_s}, .uuid{mot::Uuid{"test_detection3"}}}};

  const auto scores =
    mot::score_tracks_and_detections(tracks, detections, mot::euclidean_distance_score);

  const mot::ScoreMap expected_scores{
    {std::pair{mot::Uuid{"test_track1"}, mot::Uuid{"test_detection1"}}, 7.0710678},
    {std::pair{mot::Uuid{"test_track1"}, mot::Uuid{"test_detection2"}}, 5.7445626},
    {std::pair{mot::Uuid{"test_track2"}, mot::Uuid{"test_detection1"}}, 7.2801099},
    {std::pair{mot::Uuid{"test_track2"}, mot::Uuid{"test_detection2"}}, 6.1644139},
    {std::pair{mot::Uuid{"test_track3"}, mot::Uuid{"test_detection3"}}, 0.0},
  };

  EXPECT_EQ(std::size(scores), std::size(expected_scores));

  for (const auto & [key, value] : scores) {
    EXPECT_FLOAT_EQ(expected_scores.at(key), value);
  }
}

TEST(TestScoring, TrackToDetectionScoringMahalanobis)
{
  using namespace units::literals;

  using TestDetection = mot::Detection<mot::CtraState, mot::CtraStateCovariance>;
  using TestTrack = mot::Track<mot::CtraState, mot::CtraStateCovariance>;

  std::ifstream tracks_file{"data/test_scoring_track_to_detection_scoring_mahalanobis_tracks.json"};
  ASSERT_TRUE(tracks_file);
  const auto tracks{mot::tracks_from_json_file<TrackVariant>(tracks_file)};

  // Uncomment the following lines to use hardcoded tracks instead of reading from a file
  // const std::vector<TrackVariant> tracks{
  //   TestTrack{
  //     .state{mot::CtraState{6_m, 7_m, 8_mps, mot::Angle(3_rad), 10_rad_per_s, 12_mps_sq}},
  //     .covariance{mot::CtraStateCovariance{
  //       {0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5},
  //       {-0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123},
  //       {0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34},
  //       {-0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009},
  //       {-0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021},
  //       {0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701},
  //     }},
  //     .uuid{mot::Uuid{"test_track1"}}},
  //   TestTrack{
  //     .state{mot::CtraState{8_m, 2_m, 3_mps, mot::Angle(1_rad), 12_rad_per_s, 11_mps_sq}},
  //     .covariance{mot::CtraStateCovariance{
  //       {0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5},
  //       {-0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123},
  //       {0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34},
  //       {-0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009},
  //       {-0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021},
  //       {0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701},
  //     }},
  //     .uuid{mot::Uuid{"test_track2"}}},
  //   mot::Track<mot::CtrvState, mot::CtrvStateCovariance>{
  //     .state{1_m, 1_m, 1_mps, mot::Angle(1_rad), 1_rad_per_s},
  //     .covariance{mot::CtrvStateCovariance{
  //       {0.0043, -0.0013, 0.0030, -0.0022, -0.0020},
  //       {-0.0013, 0.0077, 0.0011, 0.0071, 0.0060},
  //       {0.0030, 0.0011, 0.0054, 0.0007, 0.0008},
  //       {-0.0022, 0.0071, 0.0007, 0.0098, 0.0100},
  //       {-0.0020, 0.0060, 0.0008, 0.0100, 0.0123}}},
  //     .uuid{mot::Uuid{"test_track3"}}}};

  const std::vector<DetectionVariant> detections{
    TestDetection{
      .state{mot::CtraState{1_m, 2_m, 3_mps, mot::Angle(3_rad), 5_rad_per_s, 6_mps_sq}},
      .uuid{mot::Uuid{"test_detection1"}}},
    TestDetection{
      .state{mot::CtraState{2_m, 3_m, 6_mps, mot::Angle(2_rad), 20_rad_per_s, 9_mps_sq}},
      .uuid{mot::Uuid{"test_detection2"}}},
    mot::Detection<mot::CtrvState, mot::CtrvStateCovariance>{
      .state{1_m, 1_m, 1_mps, mot::Angle(1_rad), 1_rad_per_s}, .uuid{mot::Uuid{"test_detection3"}}}};

  const auto scores =
    mot::score_tracks_and_detections(tracks, detections, mot::mahalanobis_distance_score);

  const mot::ScoreMap expected_scores{
    {std::pair{mot::Uuid{"test_track1"}, mot::Uuid{"test_detection1"}}, 122.35757},
    {std::pair{mot::Uuid{"test_track1"}, mot::Uuid{"test_detection2"}}, 90.688416},
    {std::pair{mot::Uuid{"test_track2"}, mot::Uuid{"test_detection1"}}, 109.70312},
    {std::pair{mot::Uuid{"test_track2"}, mot::Uuid{"test_detection2"}}, 95.243896},
    {std::pair{mot::Uuid{"test_track3"}, mot::Uuid{"test_detection3"}}, 0.0},
  };

  EXPECT_EQ(std::size(scores), std::size(expected_scores));

  for (const auto & [key, value] : scores) {
    EXPECT_FLOAT_EQ(expected_scores.at(key), value);
  }
}
