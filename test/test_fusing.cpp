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
#include <cooperative_perception/angle.hpp>
#include <cooperative_perception/track.hpp>
#include <cooperative_perception/fusing.hpp>
#include <cooperative_perception/detection.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/track_matching.hpp>

namespace cp = cooperative_perception;

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
