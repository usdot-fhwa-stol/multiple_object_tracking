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

#include <dlib/optimization/max_cost_assignment.h>
#include <gtest/gtest.h>

#include <multiple_object_tracking/dynamic_object.hpp>
#include <multiple_object_tracking/scoring.hpp>
#include <multiple_object_tracking/track_matching.hpp>

namespace mot = multiple_object_tracking;

/**
 * Test dlib library installation using example from library
 */
TEST(TestTrackMatching, VerifyLibraryInstallation)
{
  using namespace dlib;
  // Example from library
  // source: http://dlib.net/max_cost_assignment_ex.cpp.html

  matrix<int> cost(3, 3);
  cost = 1, 2, 6, 5, 3, 6, 4, 5, 0;

  // To find out the best assignment of people to jobs we just need to call this function.
  std::vector<long> result_assignment = max_cost_assignment(cost);

  // Assignments should be:  [2, 0, 1] which indicates that we should assign
  // the person from the first row of the cost matrix to job 2, the middle row person to
  // job 0, and the bottom row person to job 1.
  std::vector<long> expected_assignment{2, 0, 1};

  // The optimal cost should be:  16.0
  // which is correct since our optimal assignment is 6+5+5
  auto expected_optimal_cost{16.0};
  // Compute optimal cost
  auto result_optimal_cost{assignment_cost(cost, result_assignment)};

  for (unsigned int i = 0; i < result_assignment.size(); i++) {
    EXPECT_EQ(expected_assignment[i], result_assignment[i]);
  }

  EXPECT_DOUBLE_EQ(expected_optimal_cost, result_optimal_cost);
}

/**
 * Test the GnnAssociator function for associating detections to tracks
 */
TEST(TestTrackMatching, GnnAssociator)
{
  mot::ScoreMap scores{
    {{mot::Uuid{"track1"}, mot::Uuid{"detection1"}}, 10},
    {{mot::Uuid{"track1"}, mot::Uuid{"detection2"}}, 2.0},
    {{mot::Uuid{"track1"}, mot::Uuid{"detection3"}}, 1.0},
    {{mot::Uuid{"track2"}, mot::Uuid{"detection1"}}, 2.0},
    {{mot::Uuid{"track2"}, mot::Uuid{"detection2"}}, 1.0},
    {{mot::Uuid{"track2"}, mot::Uuid{"detection3"}}, 10.0},
    {{mot::Uuid{"track3"}, mot::Uuid{"detection1"}}, 3.0},
    {{mot::Uuid{"track3"}, mot::Uuid{"detection2"}}, 10.0},
    {{mot::Uuid{"track3"}, mot::Uuid{"detection3"}}, 5.0}};

  mot::AssociationMap expected_associations{
    {mot::Uuid{"track1"}, {mot::Uuid{"detection3"}}},
    {mot::Uuid{"track2"}, {mot::Uuid{"detection2"}}},
    {mot::Uuid{"track3"}, {mot::Uuid{"detection1"}}}};

  auto result_associations =
    mot::associate_detections_to_tracks(scores, mot::gnn_association_visitor);

  EXPECT_EQ(std::size(expected_associations), std::size(result_associations));

  // Compare expected_associations with result_associations
  for (const auto & pair : expected_associations) {
    const auto & track_uuid = pair.first;
    const auto & expected_detections = pair.second;

    // Check if track_uuid exists in result_associations
    EXPECT_TRUE(result_associations.count(track_uuid) > 0);

    // Get the corresponding vector of detections from result_associations
    const auto & result_detections = result_associations[track_uuid];

    // Check if the expected and result detections have the same size
    EXPECT_EQ(std::size(expected_detections), std::size(result_detections));

    // Check if each detection in expected_detections exists in result_detections
    for (const auto & detection_uuid : expected_detections) {
      EXPECT_TRUE(
        std::find(result_detections.begin(), result_detections.end(), detection_uuid) !=
        result_detections.end());
    }
  }
}

/**
 * Test the GnnAssociator function with gated scores for associating detections to tracks
 */
TEST(TestTrackMatching, GnnAssociatorWithGatedScores)
{
  mot::ScoreMap scores{
    {{mot::Uuid{"track1"}, mot::Uuid{"detection3"}}, 1.0},
    {{mot::Uuid{"track2"}, mot::Uuid{"detection2"}}, 1.0},
    {{mot::Uuid{"track3"}, mot::Uuid{"detection1"}}, 1.0}};

  mot::AssociationMap expected_associations{
    {mot::Uuid{"track1"}, {mot::Uuid{"detection3"}}},
    {mot::Uuid{"track2"}, {mot::Uuid{"detection2"}}},
    {mot::Uuid{"track3"}, {mot::Uuid{"detection1"}}}};

  auto result_associations =
    mot::associate_detections_to_tracks(scores, mot::gnn_association_visitor);

  EXPECT_EQ(std::size(expected_associations), std::size(result_associations));

  // Compare expected_associations with result_associations
  for (const auto & pair : expected_associations) {
    const auto & track_uuid = pair.first;
    const auto & expected_detections = pair.second;

    // Check if track_uuid exists in result_associations
    EXPECT_TRUE(result_associations.count(track_uuid) > 0);

    // Get the corresponding vector of detections from result_associations
    const auto & result_detections = result_associations[track_uuid];

    // Check if the expected and result detections have the same size
    EXPECT_EQ(std::size(expected_detections), std::size(result_detections));

    // Check if each detection in expected_detections exists in result_detections
    for (const auto & detection_uuid : expected_detections) {
      EXPECT_TRUE(
        std::find(result_detections.begin(), result_detections.end(), detection_uuid) !=
        result_detections.end());
    }
  }
}

TEST(TestTrackMatching, GnnAssociatorMoreDetectionsThanTracks)
{
  const mot::ScoreMap scores{
    {{mot::Uuid{"track1"}, mot::Uuid{"detection1"}}, 10},
    {{mot::Uuid{"track1"}, mot::Uuid{"detection2"}}, 2.0}};

  const mot::AssociationMap expected_associations{{mot::Uuid{"track1"}, {mot::Uuid{"detection2"}}}};

  const auto result_associations =
    mot::associate_detections_to_tracks(scores, mot::gnn_association_visitor);

  EXPECT_EQ(std::size(expected_associations), std::size(result_associations));

  for (const auto & [expected_track_uuid, expected_detection_uuids] : expected_associations) {
    EXPECT_GT(result_associations.count(expected_track_uuid), 0);

    const auto & result_detection_uuids = result_associations.at(expected_track_uuid);

    EXPECT_EQ(std::size(expected_detection_uuids), std::size(result_detection_uuids));

    for (const auto & expected_detection_uuid : expected_detection_uuids) {
      EXPECT_NE(
        std::find(
          std::cbegin(result_detection_uuids), std::cend(result_detection_uuids),
          expected_detection_uuid),
        std::cend(result_detection_uuids));
    }
  }
}
