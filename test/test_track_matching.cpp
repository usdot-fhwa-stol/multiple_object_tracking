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
#include <dlib/optimization/max_cost_assignment.h>
#include <cooperative_perception/track.hpp>
#include <cooperative_perception/scoring.hpp>
#include <cooperative_perception/detected_object.hpp>
#include <cooperative_perception/track_matching.hpp>

namespace cp = cooperative_perception;

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
  std::vector<long> expected_assignment{ 2, 0, 1 };

  // The optimal cost should be:  16.0
  // which is correct since our optimal assignment is 6+5+5
  auto expected_optimal_cost{ 16.0 };
  // Compute optimal cost
  auto result_optimal_cost{ assignment_cost(cost, result_assignment) };

  for (unsigned int i = 0; i < result_assignment.size(); i++)
  {
    EXPECT_EQ(expected_assignment[i], result_assignment[i]);
  }

  EXPECT_DOUBLE_EQ(expected_optimal_cost, result_optimal_cost);
}

TEST(TestTrackMatching, Example)
{
  using namespace units::literals;

  using TestObject = cp::DetectedObject<cp::CtraState, cp::CtraStateCovariance>;
  using TestTrack = cp::Track<cp::CtraState, cp::CtraStateCovariance>;

  const std::vector<cp::TrackType> tracks{
    TestTrack{ .state{ cp::CtraState{ 6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s, 12_mps_sq } },
               .covariance{ cp::CtraStateCovariance{
                   { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5 },
                   { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123 },
                   { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34 },
                   { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009 },
                   { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021 },
                   { 0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701 },
               } },
               .uuid{ "test_track1" } },
    TestTrack{ .state{ cp::CtraState{ 8_m, 2_m, 3_mps, cp::Angle(1_rad), 12_rad_per_s, 11_mps_sq } },
               .covariance{ cp::CtraStateCovariance{
                   { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5 },
                   { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123 },
                   { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34 },
                   { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009 },
                   { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021 },
                   { 0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701 },
               } },
               .uuid{ "test_track2" } }
  };

  const std::vector<cp::DetectedObjectType> objects{
    TestObject{ .state{ cp::CtraState{ 1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s, 6_mps_sq } },
                .uuid{ "test_object1" } },
    TestObject{ .state{ cp::CtraState{ 2_m, 3_m, 6_mps, cp::Angle(2_rad), 20_rad_per_s, 9_mps_sq } },
                .uuid{ "test_object2" } }
  };

  cp::assign_objects_to_tracks(objects, tracks);
}
