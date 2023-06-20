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
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "cooperative_perception/common_visitors.hpp"
#include "cooperative_perception/visitor.hpp"

namespace cooperative_perception
{

constexpr Visitor euclidean_distance_visitor{ [](const auto& track, const auto& object) -> std::optional<float> {
  if constexpr (std::is_same_v<decltype(track.state), decltype(object.state)>)
  {
    return euclidean_distance(track.state, object.state);
  }
  else
  {
    return std::nullopt;
  }
} };

constexpr Visitor mahalanobis_distance_visitor{ [](const auto& track, const auto& object) -> std::optional<float> {
  if constexpr (std::is_same_v<decltype(track.state), decltype(object.state)>)
  {
    return mahalanobis_distance(track.state, track.covariance, object.state);
  }
  else
  {
    return std::nullopt;
  }
} };

using ScoreMap = std::map<std::pair<std::string, std::string>, float>;

template <typename TrackType, typename DetectionType, typename MetricVisitor>
auto scoreTracksAndDetections(const std::vector<TrackType>& tracks, const std::vector<DetectionType>& detections,
                              const MetricVisitor& metric_visitor) -> ScoreMap
{
  ScoreMap scores;

  for (const auto& track : tracks)
  {
    const auto track_uuid{ std::visit(uuid_visitor, track) };

    for (const auto& detection : detections)
    {
      const auto detection_uuid{ std::visit(uuid_visitor, detection) };

      if (const auto score = std::visit(metric_visitor, track, detection); score.has_value())
      {
        scores[{ track_uuid, detection_uuid }] = score.value();
      }
    }
  }

  return scores;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_SCORING_HPP
