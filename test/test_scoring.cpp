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
#include <cooperative_perception/detected_object.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/track.hpp>
#include <cooperative_perception/scoring.hpp>
#include <units.h>

namespace cp = cooperative_perception;

TEST(TestScoring, CtrvEuclideanDistance)
{
  using namespace units::literals;

  const auto object = cp::DetectedObject<cp::CtrvState, cp::CtrvStateCovariance>{
    units::time::second_t{ 0 }, cp::CtrvState{ 1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s }
  };

  const auto track = cp::Track<cp::CtrvState, cp::CtrvStateCovariance>{
    units::time::second_t{ 0 }, cp::CtrvState{ 6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s }
  };

  const auto euclidean_dist = cp::euclidean_distance(object, track);
  EXPECT_FLOAT_EQ(euclidean_dist, 10.0F);
}

TEST(TestScoring, CtrvMahalanobisDistance)
{
  using namespace units::literals;

  const auto object = cp::DetectedObject<cp::CtrvState, cp::CtrvStateCovariance>{
    units::time::second_t{ 0 }, cp::CtrvState{ 1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s }
  };

  const auto track = cp::Track<cp::CtrvState, cp::CtrvStateCovariance>{
    units::time::second_t{ 0 },
    cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s },
    cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                             { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                             { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                             { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                             { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } }
  };

  const auto mahalanobis_dist = cp::mahalanobis_distance(object, track);
  EXPECT_FLOAT_EQ(mahalanobis_dist, 120.31613151700158F);
}

TEST(TestScoring, CtraEuclideanDistance)
{
  using namespace units::literals;

  const auto object = cp::DetectedObject<cp::CtraState, cp::CtraStateCovariance>{
    units::time::second_t{ 0 }, cp::CtraState{ 1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s, 6_mps_sq }
  };

  const auto track = cp::Track<cp::CtraState, cp::CtraStateCovariance>{
    units::time::second_t{ 0 }, cp::CtraState{ 6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s, 12_mps_sq }
  };

  const auto euclidean_dist = cp::euclidean_distance(object, track);
  EXPECT_FLOAT_EQ(euclidean_dist, 2 * sqrt(34));
}

TEST(TestScoring, CtraMahalanobisDistance)
{
  using namespace units::literals;

  const auto object = cp::DetectedObject<cp::CtraState, cp::CtraStateCovariance>{
    units::time::second_t{ 0 }, cp::CtraState{ 1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s, 6_mps_sq }
  };

  const auto track = cp::Track<cp::CtraState, cp::CtraStateCovariance>{
    units::time::second_t{ 0 }, cp::CtraState{ 6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s, 12_mps_sq },
    cp::CtraStateCovariance{
        { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5 },
        { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123 },
        { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34 },
        { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009 },
        { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021 },
        { 0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701 },
    }
  };

  const auto mahalanobis_dist = cp::mahalanobis_distance(object, track);
  EXPECT_FLOAT_EQ(mahalanobis_dist, 215.22495427069776);
}

TEST(TestScoring, TrackToObjectScoring)
{
  using namespace units::literals;

  using TestObject = cp::DetectedObject<cp::CtraState, cp::CtraStateCovariance>;
  using TestTrack = cp::Track<cp::CtraState, cp::CtraStateCovariance>;

  const std::vector<cp::TrackType> tracks{
    TestTrack{ .state{ cp::CtraState{ 6_m, 7_m, 8_mps, cp::Angle(3_rad), 10_rad_per_s, 12_mps_sq } },
               .uuid{ "test_track1" } },
    TestTrack{ .state{ cp::CtraState{ 8_m, 2_m, 3_mps, cp::Angle(1_rad), 12_rad_per_s, 11_mps_sq } },
               .uuid{ "test_track2" } }
  };

  const std::vector<cp::DetectedObjectType> objects{
    TestObject{ .state{ cp::CtraState{ 1_m, 2_m, 3_mps, cp::Angle(3_rad), 5_rad_per_s, 6_mps_sq } },
                .uuid{ "test_object1" } },
    TestObject{ .state{ cp::CtraState{ 2_m, 3_m, 6_mps, cp::Angle(2_rad), 20_rad_per_s, 9_mps_sq } },
                .uuid{ "test_object2" } }
  };

  const auto scores = cp::score_tracks_and_objects(tracks, objects);

  const std::map<std::pair<std::string, std::string>, float> expected_scores{
    { std::pair{ "test_track1", "test_object1" }, 11.661903 },
    { std::pair{ "test_track1", "test_object2" }, 12.083046 },
    { std::pair{ "test_track2", "test_object1" }, 11.269427 },
    { std::pair{ "test_track2", "test_object2" }, 10.723805 },
  };

  EXPECT_EQ(std::size(scores), std::size(expected_scores));

  for (const auto& [key, value] : scores)
  {
    EXPECT_FLOAT_EQ(expected_scores.at(key), value);
  }
}
