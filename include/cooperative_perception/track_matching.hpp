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

#ifndef COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP
#define COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP

#include "cooperative_perception/track.hpp"
#include "cooperative_perception/scoring.hpp"
#include "cooperative_perception/detection.hpp"
#include <map>
#include <vector>
#include <string>
#include <dlib/optimization/max_cost_assignment.h>

namespace cooperative_perception
{
// association_visitor

template <typename AssociationVisitor>
auto associate_objects_to_tracks(std::map<std::pair<std::string, std::string>, std::optional<float>> gated_scores,
                                 const AssociationVisitor& association_visitor)
    -> std::map<std::string, std::vector<std::string>>
{
  //   using namespace dlib;
  std::map<std::string, std::vector<std::string>> associations;

  for (const auto& [uuid_pair, score] : gated_scores)
  {
    associations[uuid_pair.first] = std::visit(association_visitor, uuid_pair.first, gated_scores);
  }

  return associations;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP
