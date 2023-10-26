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

#ifndef COOPERATIVE_PERCEPTION_TRACK_MANAGEMENT_HPP
#define COOPERATIVE_PERCEPTION_TRACK_MANAGEMENT_HPP

#include "cooperative_perception/track_matching.hpp"
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

template <typename Track>
class FixedThresholdTrackManager
{
  enum class TrackStatus
  {
    kConfirmed,
    kTentative
  };

public:
  explicit FixedThresholdTrackManager(
    PromotionThreshold promotion_threshold, RemovalThreshold removal_threshold)
  : promotion_threshold_{promotion_threshold}, removal_threshold_{removal_threshold}
  {
  }

  auto update_track_lists(const AssociationMap & associations) -> void
  {
    for (auto & [uuid, occurrences] : occurrences_) {
      if (associations.count(uuid) == 0) {
        --occurrences;
      } else {
        ++occurrences;
      }
    }

    for (auto it{std::begin(occurrences_)}; it != std::end(occurrences_);) {
      const auto uuid{it->first};
      const auto occurrences{it->second};

      if (occurrences <= removal_threshold_.value) {
        tracks_.erase(uuid);
        statuses_.erase(uuid);
        it = occurrences_.erase(it);
        continue;
      }

      if (occurrences >= promotion_threshold_.value) {
        statuses_.at(uuid) = TrackStatus::kConfirmed;
      } else {
        statuses_.at(uuid) = TrackStatus::kTentative;
      }

      ++it;
    }
  }

  auto add_tentative_track(const Track & track) -> void
  {
    const auto uuid = get_uuid(track);

    if (tracks_.count(uuid) != 0) {
      throw std::logic_error("track '" + uuid.value() + "' already exists");
    }

    tracks_[uuid] = track;
    statuses_[uuid] = TrackStatus::kTentative;
    occurrences_[uuid] = 1;
  }

  auto get_tentative_tracks() const -> std::vector<Track>
  {
    std::vector<Track> tracks;

    for (const auto & [uuid, track] : tracks_) {
      if (statuses_.at(uuid) == TrackStatus::kTentative) {
        tracks.emplace_back(track);
      }
    }

    return tracks;
  }

  auto get_confirmed_tracks() const -> std::vector<Track>
  {
    std::vector<Track> tracks;

    for (const auto & [uuid, track] : tracks_) {
      if (statuses_.at(uuid) == TrackStatus::kConfirmed) {
        tracks.emplace_back(track);
      }
    }

    return tracks;
  }

  auto get_all_tracks() const -> std::vector<Track>
  {
    std::vector<Track> tracks;

    for (const auto & [uuid, track] : tracks_) {
      tracks.emplace_back(track);
    }

    return tracks;
  }

  auto set_promotion_threshold_and_update(const PromotionThreshold & threshold) noexcept -> void
  {
    promotion_threshold_ = threshold;

    for (auto & [uuid, track] : tracks_) {
      if (occurrences_[uuid] >= promotion_threshold_.value) {
        statuses_[uuid] = TrackStatus::kConfirmed;
      } else if (occurrences_[uuid] < promotion_threshold_.value) {
        statuses_[uuid] = TrackStatus::kTentative;
      }
    }
  }

  [[nodiscard]] auto get_promotion_threshold() const noexcept -> PromotionThreshold
  {
    return promotion_threshold_;
  }

  auto set_removal_threshold_and_update(const RemovalThreshold & threshold) noexcept -> void
  {
    removal_threshold_ = threshold;

    for (auto it{std::begin(occurrences_)}; it != std::end(occurrences_);) {
      if (const auto occurrences{it->second}; occurrences <= removal_threshold_.value) {
        const auto uuid{it->first};

        tracks_.erase(uuid);
        statuses_.erase(uuid);
        it = occurrences_.erase(it);
      } else {
        ++it;
      }
    }
  }

  [[nodiscard]] auto get_removal_threshold() const noexcept -> RemovalThreshold
  {
    return removal_threshold_;
  }

private:
  PromotionThreshold promotion_threshold_;
  RemovalThreshold removal_threshold_;
  std::unordered_map<Uuid, Track> tracks_;
  std::unordered_map<Uuid, TrackStatus> statuses_;
  std::unordered_map<Uuid, std::size_t> occurrences_;
};

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_MANAGEMENT_HPP
