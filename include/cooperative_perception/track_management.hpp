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

#ifndef COOPERATIVE_PERCEPTION_TRACK_MANAGEMENT_HPP
#define COOPERATIVE_PERCEPTION_TRACK_MANAGEMENT_HPP

#include "cooperative_perception/uuid.hpp"

namespace cooperative_perception
{
template <typename Type, typename Tag>
struct TaggedType
{
  Type value;
};

using PromotionThreshold = TaggedType<std::size_t, struct PromotionThresholdTag>;
using RemovalThreshold = TaggedType<std::size_t, struct RemovalThresholdTag>;

class FixedThresholdManagementPolicy
{
public:
  explicit FixedThresholdManagementPolicy(
    PromotionThreshold promotion_threshold, RemovalThreshold removal_threshold)
  : promotion_threshold_{promotion_threshold}, removal_threshold_{removal_threshold}
  {
  }

  auto update(const cp::AssociationMap & associations) -> void;
  auto should_promote(const std::string & uuid) const -> bool;
  auto should_demote(const std::string & uuid) const -> bool;
  auto should_remove(const std::string & uuid) const -> bool;

private:
  PromotionThreshold promotion_threshold_;
  RemovalThreshold removal_threshold_;
  std::unordered_map<std::string, std::size_t> occurrence_counts;
};

/**
 * @brief Track status
 *
 * Tracks are tentative until they have been perceived beyond a threshold, at
 * which point they become confirmed.
 */
enum class TrackStatus
{
  kConfirmed,
  kTentative
};

template <typename TrackType, typename ManagementPolicy>
class TrackManager
{
public:
  explicit TrackManager(ManagementPolicy management_policy) : management_policy_{management_policy}
  {
  }

  auto update_track_lists(const cp::AssociationMap & associations) -> void
  {
    management_policy_.update(associations);

    for (const auto & [uuid, track] : tracks_) {
      if (management_policy_.should_promote(uuid)) {
        track_statuses_[uuid] = "CONFIRMED";

      } else if (management_policy_.should_demote(uuid)) {
        track_statuses_[uuid] = "TENTATIVE";

      } else if (management_policy_.should_remove(uuid)) {
        tracks_.erase(uuid);
        track_statuses_.erase(uuid);
      }
    }
  }

  auto add_tentative_track(const TrackType & track) -> void
  {
    const auto uuid = get_uuid(track);
    tracks_[uuid] = track;
    track_statuses_[uuid] = "TENTATIVE";
  }

  auto get_tentative_tracks() const -> std::vector<TrackType>
  {
    std::vector<TrackType> tracks;

    for (const auto & [uuid, track] : tracks_) {
      if (track_statuses_[uuid] == "TENTATIVE") {
        tracks.emplace_back(track);
      }
    }

    return tracks;
  }

  auto get_confirmed_tracks() const -> std::vector<TrackType>
  {
    std::vector<TrackType> tracks;

    for (const auto & [uuid, track] : tracks_) {
      if (track_statuses_[uuid] == "CONFIRMED") {
        tracks.emplace_back(track);
      }
    }

    return tracks;
  }

  auto get_all_tracks() const -> std::vector<TrackType>
  {
    std::vector<TrackType> tracks;

    for (const auto & [uuid, track] : tracks_) {
      tracks.emplace_back(track);
    }

    return tracks;
  }

private:
  ManagementPolicy management_policy_;
  std::unordered_map<std::string, TrackType> tracks_;
  std::unordered_map<std::string, cp::TrackStatus> track_statuses_;
};

}  // namespace cooperative_perception

#endif COOPERATIVE_PERCEPTION_TRACK_MANAGEMENT_HPP
