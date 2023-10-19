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

#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/track_management.hpp>
#include <cooperative_perception/uuid.hpp>

namespace cp = cooperative_perception;

TEST(TestTrackManagement, Simple)
{
  cp::FixedThresholdTrackManager<cp::CtrvTrack> track_manager{
    cp::PromotionThreshold{3}, cp::RemovalThreshold{1}};

  cp::CtrvTrack track;
  track.uuid = cp::Uuid{"test_track"};

  cp::AssociationMap association_map;
  association_map[cp::Uuid{"test_track"}].push_back(cp::Uuid{"test_detection"});

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

  track_manager.update_track_lists(cp::AssociationMap{});

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 1U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 1U);

  track_manager.update_track_lists(cp::AssociationMap{});
  track_manager.update_track_lists(cp::AssociationMap{});

  EXPECT_EQ(std::size(track_manager.get_tentative_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_confirmed_tracks()), 0U);
  EXPECT_EQ(std::size(track_manager.get_all_tracks()), 0U);
}
