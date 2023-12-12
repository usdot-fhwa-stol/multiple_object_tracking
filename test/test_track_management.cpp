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

#include <gtest/gtest.h>
#include <units.h>

#include <multiple_object_tracking/ctrv_model.hpp>
#include <multiple_object_tracking/track_management.hpp>
#include <multiple_object_tracking/uuid.hpp>

#include "multiple_object_tracking/test/gmock_matchers.hpp"

namespace mot = multiple_object_tracking;

TEST(TestTrackManagement, Simple)
{
  mot::FixedThresholdTrackManager<mot::CtrvTrack> track_manager{
    mot::PromotionThreshold{3}, mot::RemovalThreshold{1}};

  mot::CtrvTrack track;
  track.uuid = mot::Uuid{"test_track"};

  mot::AssociationMap association_map;
  association_map[mot::Uuid{"test_track"}].push_back(mot::Uuid{"test_detection"});

  track_manager.add_tentative_track(track);

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.update_track_lists(association_map);

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.update_track_lists(association_map);

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.update_track_lists(mot::AssociationMap{});

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.update_track_lists(mot::AssociationMap{});
  track_manager.update_track_lists(mot::AssociationMap{});

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 0U);
}

TEST(TestTrackManagement, Getters)
{
  const mot::FixedThresholdTrackManager<mot::CtrvTrack> track_manager{
    mot::PromotionThreshold{3}, mot::RemovalThreshold{1}};

  EXPECT_EQ(track_manager.get_promotion_threshold().value, mot::PromotionThreshold{3U}.value);
  EXPECT_EQ(track_manager.get_removal_threshold().value, mot::RemovalThreshold{1U}.value);
}

TEST(TestTrackManagement, Setters)
{
  mot::FixedThresholdTrackManager<mot::CtrvTrack> track_manager{
    mot::PromotionThreshold{3}, mot::RemovalThreshold{1}};

  mot::CtrvTrack track;
  track.uuid = mot::Uuid{"test_track"};

  mot::AssociationMap association_map;
  association_map[mot::Uuid{"test_track"}].push_back(mot::Uuid{"test_detection"});

  track_manager.add_tentative_track(track);

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.update_track_lists(association_map);

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.update_track_lists(association_map);

  track_manager.set_promotion_threshold_and_update(mot::PromotionThreshold{1U});

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.set_promotion_threshold_and_update(mot::PromotionThreshold{10U});

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.set_removal_threshold_and_update(mot::RemovalThreshold{5U});

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 0U);
}

TEST(TestTrackManagement, UpdateTrack)
{
  using units::literals::operator""_m;
  using units::literals::operator""_mps;
  using units::literals::operator""_rad;
  using units::literals::operator""_rad_per_s;
  using units::literals::operator""_s;

  mot::CtrvTrack track;
  track.uuid = mot::Uuid{"test_track"};
  track.timestamp = 0_s;
  track.state = mot::CtrvState{};
  track.covariance = mot::CtrvStateCovariance{};

  mot::FixedThresholdTrackManager<mot::CtrvTrack> track_manager{
    mot::PromotionThreshold{3U}, mot::RemovalThreshold{0U}};
  track_manager.add_tentative_track(track);

  track_manager.update_track(
    track.uuid, 1_s, mot::CtrvState{1_m, 2_m, 3_mps, mot::Angle{4_rad}, 5_rad_per_s},
    mot::CtrvStateCovariance::Identity());

  const auto tracks{track_manager.get_all_tracks()};
  ASSERT_EQ(std::size(tracks), 1U);

  EXPECT_EQ(mot::get_uuid(tracks.at(0)), track.uuid);
  EXPECT_EQ(mot::get_timestamp(tracks.at(0)), 1_s);
  EXPECT_THAT(
    tracks.at(0).state,
    CtrvStateNear(mot::CtrvState{1_m, 2_m, 3_mps, mot::Angle{4_rad}, 5_rad_per_s}, 1e-9));
  EXPECT_THAT(tracks.at(0).covariance, EigenMatrixNear(mot::CtrvStateCovariance::Identity(), 1e-9));
}
