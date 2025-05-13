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

#ifndef MULTIPLE_OBJECT_TRACKING_TRACK_MANAGEMENT_HPP
#define MULTIPLE_OBJECT_TRACKING_TRACK_MANAGEMENT_HPP

#include <algorithm>

#include "multiple_object_tracking/track_matching.hpp"
#include "multiple_object_tracking/uuid.hpp"
namespace multiple_object_tracking
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

  auto get_occurrences(const Uuid & uuid) const
  {
    if (occurrences_.count(uuid) == 0) {
      return 0;
    }

    return static_cast<int>(occurrences_.at(uuid));
  }

  auto update_track_lists(const AssociationMap & associations) -> void
  {

    // First loop: Update occurrences based on associations
    for (auto & [uuid, occurrences] : occurrences_) {

      if (associations.count(uuid) == 0) {
        --occurrences;
      } else {
        // Max at promotion value
        occurrences = std::min(occurrences + 1, promotion_threshold_.value);
        // If this was a previously confirmed track, force it to be promotion occurrence
        if (statuses_.at(uuid) == TrackStatus::kConfirmed) {
          occurrences = promotion_threshold_.value;
        }
      }
    }

    // Second loop: Handle track statuses based on updated occurrences
    for (auto it{std::begin(occurrences_)}; it != std::end(occurrences_);) {
      const auto uuid{it->first};
      const auto occurrences{it->second};


      if (occurrences <= removal_threshold_.value) {

        try {
          tracks_.erase(uuid);
        } catch (const std::exception& e) {
          std::cerr << "ERROR: Failed to erase UUID "
            << uuid << " from tracks_: " << e.what() << ", continuing" << std::endl;
        }

        try {
          statuses_.erase(uuid);
        } catch (const std::exception& e) {
          std::cerr << "ERROR: Failed to erase UUID "
            << uuid << " from statuses_: " << e.what() << ", continuing" << std::endl;
        }

        try {
          it = occurrences_.erase(it);
        } catch (const std::exception& e) {
          ++it; // Still advance iterator to avoid infinite loop
          std::cerr << "ERROR: Failed to erase UUID "
            << uuid << " from occurrences_: " << e.what() << ", continuing" << std::endl;
        }

        continue;
      }

      if (occurrences >= promotion_threshold_.value) {
        try {
          TrackStatus old_status = statuses_.at(uuid);
          statuses_.at(uuid) = TrackStatus::kConfirmed;
        } catch (const std::out_of_range& e) {
          std::cerr << "ERROR: UUID "
            << uuid << " not found in statuses_ map: " << e.what() << std::endl;
        } catch (const std::exception& e) {
          std::cerr << "ERROR: Failed to update status for UUID "
            << uuid << ": " << e.what() << std::endl;
        }
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

  template <typename State, typename StateCovariance>
  auto update_track(
    const Uuid track_id, const units::time::second_t & timestamp, const State & state,
    const StateCovariance & state_covariance)
  {
    auto & track = tracks_.at(track_id);

    set_timestamp(track, timestamp);
    set_state(track, state);
    set_state_covariance(track, state_covariance);
  }

  auto update_track(const Uuid track_id, const Track & track)
  {
    set_timestamp(tracks_.at(track_id), get_timestamp(track));
    copy_state(tracks_.at(track_id), track);
    copy_state_covariance(tracks_.at(track_id), track);
  }

private:
  PromotionThreshold promotion_threshold_;
  RemovalThreshold removal_threshold_;
  std::unordered_map<Uuid, Track> tracks_;
  std::unordered_map<Uuid, TrackStatus> statuses_;
  std::unordered_map<Uuid, std::size_t> occurrences_;
};

}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_TRACK_MANAGEMENT_HPP
