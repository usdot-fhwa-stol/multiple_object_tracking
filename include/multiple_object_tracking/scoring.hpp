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

#ifndef MULTIPLE_OBJECT_TRACKING_SCORING_HPP
#define MULTIPLE_OBJECT_TRACKING_SCORING_HPP

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "multiple_object_tracking/uuid.hpp"
#include "multiple_object_tracking/visitor.hpp"

namespace multiple_object_tracking
{
namespace detail
{
struct euclidean_distance_score_fn
{
  template <typename Track, typename Detection>
  auto operator()(const Track & track, const Detection & detection) const -> std::optional<float>
  {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      return euclidean_distance(track.state, detection.state);
    } else {
      return std::nullopt;
    }
  }

  template <typename... TrackAlternatives, typename... DetectionAlternatives>
  auto operator()(
    const std::variant<TrackAlternatives...> & track,
    const std::variant<DetectionAlternatives...> & detection) const -> std::optional<float>
  {
    return std::visit(
      [this](const auto & track, const auto & detection) { return (*this)(track, detection); },
      track, detection);
  }
};

struct mahalanobis_distance_score_fn
{
  template <typename Track, typename Detection>
  auto operator()(const Track & track, const Detection & detection) const -> std::optional<float>
  {
    if constexpr (std::is_same_v<decltype(track.state), decltype(detection.state)>) {
      return mahalanobis_distance(track.state, track.covariance, detection.state);
    } else {
      return std::nullopt;
    }
  }

  template <typename... TrackAlternatives, typename... DetectionAlternatives>
  auto operator()(
    const std::variant<TrackAlternatives...> & track,
    const std::variant<DetectionAlternatives...> & detection) const -> std::optional<float>
  {
    return std::visit(
      [this](const auto & track, const auto & detection) { return (*this)(track, detection); },
      track, detection);
  }
};

}  // namespace detail

inline constexpr detail::euclidean_distance_score_fn euclidean_distance_score{};
inline constexpr detail::mahalanobis_distance_score_fn mahalanobis_distance_score{};

using ScoreMap = std::map<std::pair<Uuid, Uuid>, float>;

template <typename Track, typename Detection, typename Metric>
auto score_tracks_and_detections(
  const std::vector<Track> & tracks, const std::vector<Detection> & detections,
  const Metric & metric) -> ScoreMap
{
  ScoreMap scores;

  for (const auto & track : tracks) {
    const auto track_uuid{get_uuid(track)};

    for (const auto & detection : detections) {
      const auto detection_uuid{get_uuid(detection)};

      if (const auto score = metric(track, detection); score.has_value()) {
        scores[{track_uuid, detection_uuid}] = score.value();
      }
    }
  }

  return scores;
}

}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_SCORING_HPP
