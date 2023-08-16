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

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "cooperative_perception/uuid.hpp"
#include "cooperative_perception/visitor.hpp"

namespace cooperative_perception
{
constexpr Visitor euclidean_distance_visitor{
  /**
   * @brief Calculate the Euclidean distance between a track and detection
   *
   * The track and detection must have identical state types to be compared.
   *
   * @param[in] track Track being measured (TrackType)
   * @param[in] detection Detection being measured (DetectionType)
   * @return Euclidean distance between track and detection if their state
   * types are identical; std::nullopt otherwise
   */
  [](const auto & track, const auto & detection) -> std::optional<float> {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      return euclidean_distance(track.state, detection.state);
    } else {
      return std::nullopt;
    }
  }};

constexpr Visitor mahalanobis_distance_visitor{
  /**
   * @brief Calculate the Mahalanobis distance between a track and detection
   *
   * The track and detection must have identical state types to be compared.
   *
   * @param[in] track Track being measured (TrackType)
   * @param[in] detection Detection being measured (DetectionType)
   * @return Mahalanobis distance between track and detection if their state
   * types are identical; std::nullopt otherwise
   */
  [](const auto & track, const auto & detection) -> std::optional<float> {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      return mahalanobis_distance(track.state, track.covariance, detection.state);
    } else {
      return std::nullopt;
    }
  }};

using ScoreMap = std::map<std::pair<std::string, std::string>, float>;

template <typename TrackVariant, typename DetectionVariant, typename MetricVisitor>
auto score_tracks_and_detections(
  const std::vector<TrackVariant> & tracks, const std::vector<DetectionVariant> & detections,
  const MetricVisitor & metric_visitor) -> ScoreMap
{
  ScoreMap scores;

  for (const auto & track : tracks) {
    const auto track_uuid{get_uuid(track)};

    for (const auto & detection : detections) {
      const auto detection_uuid{get_uuid(detection)};

      if (const auto score = std::visit(metric_visitor, track, detection); score.has_value()) {
        scores[{track_uuid, detection_uuid}] = score.value();
      }
    }
  }

  return scores;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_SCORING_HPP
