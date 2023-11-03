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

#ifndef MULTIPLE_OBJECT_TRACKING_GATING_HPP
#define MULTIPLE_OBJECT_TRACKING_GATING_HPP

#include <vector>

#include "multiple_object_tracking/scoring.hpp"

namespace multiple_object_tracking
{
template <typename UnaryPredicate>
auto prune_track_and_detection_scores_if(ScoreMap & scores, UnaryPredicate should_prune) -> void
{
  std::vector<std::pair<Uuid, Uuid>> keys_to_prune;

  for (const auto & [key, score] : scores) {
    if (should_prune(score)) {
      keys_to_prune.emplace_back(key);
    }
  }

  for (const auto & key : keys_to_prune) {
    scores.erase(key);
  }
}

}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_GATING_HPP
