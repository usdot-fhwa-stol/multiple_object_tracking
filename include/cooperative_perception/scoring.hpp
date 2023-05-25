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

#ifndef COOPERATIVE_PERCEPTION_SCORING_HPP
#define COOPERATIVE_PERCEPTION_SCORING_HPP

#include <vector>
#include <map>
#include <utility>
#include <type_traits>

#include "cooperative_perception/track.hpp"
#include "cooperative_perception/detected_object.hpp"
#include "cooperative_perception/ctra_model.hpp"
#include "cooperative_perception/ctrv_model.hpp"

namespace cooperative_perception
{
template <typename DetectedObject, typename Track>
auto mahalanobis_distance(DetectedObject object, Track track) -> float
{
  return mahalanobis_distance(track.state, track.covariance, object.state);
}

template <typename DetectedObject, typename Track>
auto euclidean_distance(DetectedObject object, Track track) -> float
{
  return euclidean_distance(object.state, track.state);
}

constexpr utils::Visitor kUuidExtractor{ [](const auto& entity) { return entity.uuid; } };
constexpr utils::Visitor kEuclideanDistanceGetter{
  [](const auto& track,
     const auto& object) -> std::enable_if_t<std::is_same_v<decltype(track.state), decltype(object.state)>, float> {
    return euclidean_distance(track.state, object.state);
  },
  [](const auto& track, const auto& object)
      -> std::enable_if_t<!std::is_same_v<decltype(track.state), decltype(object.state)>, float> { return -1.0F; },
};

auto score_tracks_and_objects(const std::vector<TrackType>& tracks, const std::vector<DetectedObjectType>& objects)
{
  std::map<std::pair<std::string, std::string>, float> scores;

  for (const auto& track : tracks)
  {
    const auto track_uuid = std::visit(kUuidExtractor, track);

    for (const auto& object : objects)
    {
      const auto object_uuid = std::visit(kUuidExtractor, object);
      scores[std::pair{ track_uuid, object_uuid }] = std::visit(kEuclideanDistanceGetter, track, object);
    }
  }

  return scores;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_SCORING_HPP
