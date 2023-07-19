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

#ifndef COOPERATIVE_PERCEPTION_TRACK_MAINTENANCE_HPP
#define COOPERATIVE_PERCEPTION_TRACK_MAINTENANCE_HPP

#include "cooperative_perception/uuid.hpp"

namespace cooperative_perception
{
/**
 * @brief Track manager
 *
 * The track manager suggests if tracks should be promoted, demoted, or removed based on the number
 * of time a specific track has appeared during the manager's lifetime.
 */
struct TrackManager
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] promotion_threshold Number of times a track must consecutively appear before being confirmed. If the
   * track's occurrence count drops below this threshold, it should be demoted.
   */
  explicit TrackManager(std::size_t promotion_threshold) : promotion_threshold_{ promotion_threshold }
  {
  }

  /**
   * @brief Add UUIDs for tracks that should be managed
   *
   * Each UUID will be initialized with an occurrence of 1.
   *
   * @tparam Container Container holding the track UUIDs
   *
   * @param[in] uuids UUIDs that will be managed
   * @return void
   */
  template <typename Container>
  auto addTrackUuids(const Container& uuids) -> void
  {
    for (const auto& uuid : uuids)
    {
      occurrence_counts_[uuid] = 1;
    }
  }

  /**
   * @brief Update occurrence counts for all managed UUIDs
   *
   * UUIDs that are not associated will have their occurrence counts decremented by 1. UUIDs whose counts drop to 0
   * will be removed (i.e., they will no longer be managed).
   *
   * @param[in] associated_uuids UUIDs for tracks that are associated with at least one detection
   * @return void
   */
  template <typename Container>
  auto updateOccurrenceCounts(const Container& associated_uuids) -> void
  {
    for (const auto& uuid : associated_uuids)
    {
      occurrence_counts_[uuid] += 2;
    }

    for (auto& [uuid, occurrence_count] : occurrence_counts_)
    {
      --occurrence_count;

      if (occurrence_count == 0)
      {
        occurrence_counts_.erase(uuid);
      }
    }
  }

  /**
   * @brief Get the occurrence counts for managed UUIDs
   *
   * @return Occurrence counts for each managed UUID
   */
  auto getOccurrenceCounts() const noexcept -> const std::unordered_map<std::string, std::size_t>
  {
    return occurrence_counts_;
  }

  /**
   * @brief Query if track with specified UUID should be promoted
   *
   * Promotion means the track's status should change from tentative to confirmed.
   *
   * @param[in] track_uuid UUID associated with track
   * @return true if the track should be promoted; false otherwise
   */
  auto shouldPromote(const std::string& track_uuid) const -> bool
  {
    return occurrence_counts_.at(track_uuid) >= promotion_threshold_;
  }

  /**
   * @brief Query if track with specified UUID should be demoted
   *
   * Demotion means the track's status should change from confirmed to tentative.
   *
   * @param[in] track_uuid UUID associated with track
   * @return true if the track should be demoted; false otherwise
   */
  auto shouldDemote(const std::string& track_uuid) const -> bool
  {
    return occurrence_counts_.at(track_uuid) < promotion_threshold_;
  }

  /**
   * @brief Query if track with specified UUID should be removed
   *
   * @param[in] track_uuid UUID associated with track
   * @return true if the track should be removed; false otherwise
   */
  auto shouldRemove(const std::string& track_uuid) const noexcept -> bool
  {
    return occurrence_counts_.count(track_uuid) == 0;
  }

private:
  std::size_t promotion_threshold_;
  std::unordered_map<std::string, std::size_t> occurrence_counts_;
};

/**
 * @brief Update track statuses and possibly remove some tracks
 *
 * This function updates the status for each track according to a track
 * manager. Tracks that the track manager suggests should be removed will
 * be pruned from the container holding the tracks.
 *
 * @tparam Container Container type holding the tracks
 *
 * @param[in] tracks Tracks being updated
 * @param[in] track_manager Track manager deciding how each track should be updated
 * @return void
 */
template <typename Container>
auto updateTracks(Container& tracks, const TrackManager& track_manager) -> void
{
  for (auto& track : tracks)
  {
    const auto uuid = getUuid(track);

    if (track_manager.shouldPromote(uuid))
    {
      set_track_status(track, TrackStatus::kConfirmed);
    }
    else if (track_manager.shouldDemote(uuid))
    {
      set_track_status(track, TrackStatus::kTentative);
    }
  }

  std::remove_if(std::begin(tracks), std::end(tracks),
                 [track_manager](const auto& track) { return track_manager.shouldRemove(getUuid(track)); });
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_MAINTENANCE_HPP
