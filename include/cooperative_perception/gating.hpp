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

#ifndef COOPERATIVE_PERCEPTION_GATING_HPP
#define COOPERATIVE_PERCEPTION_GATING_HPP
#include <vector>
#include <map>
#include <utility>
#include <type_traits>
#include <stdexcept>

#include "cooperative_perception/track.hpp"
#include "cooperative_perception/detected_object.hpp"

namespace cooperative_perception
{
constexpr utils::Visitor euclidean_gating_visitor{ [](const auto& track, const auto& object) -> std::optional<float> {
  if constexpr (std::is_same_v<decltype(track.state), decltype(object.state)>)
  {
    return mahalanobis_distance(track.state, track.covariance, object.state);
  }
  else
  {
    return std::nullopt;
  }
} };

template <typename GatingVisitor>
auto gate_scores(const std::vector<TrackType>& tracks, const std::vector<DetectedObjectType>& objects,
                 const GatingVisitor& gating_visitor)
    -> std::map<std::pair<std::string, std::string>, std::optional<float>>
{
  std::map<std::pair<std::string, std::string>, std::optional<float>> gated_scores;
  for (const auto& track : tracks)
  {
    const auto track_uuid = std::visit(uuid_visitor, track);

    for (const auto& object : objects)
    {
      const auto object_uuid = std::visit(uuid_visitor, object);
      gated_scores[std::pair{ track_uuid, object_uuid }] = std::visit(gating_visitor, track, object);
    }
  }

  return gated_scores;
}
}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_GATING_HPP
