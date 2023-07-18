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

#ifndef COOPERATIVE_PERCEPTION_FUSING_HPP
#define COOPERATIVE_PERCEPTION_FUSING_HPP

#include <tuple>
#include <vector>
#include <Eigen/Dense>
#include "cooperative_perception/track_matching.hpp"

namespace cooperative_perception
{
constexpr Visitor covariance_intersection_visitor{ [](const auto& associations) {
  // Call CI after implementation
} };

template <typename DetectionContainer, typename TrackContainer, typename FusionVisitor>
auto fuseAssociations(const AssociationMap& associations, const DetectionContainer& detections,
                      const TrackContainer& tracks, const FusionVisitor& fusion_visitor) -> TrackContainer
{
  TrackContainer fused_tracks;

  return fused_tracks;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_FUSING_HPP
