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
#include <variant>
#include <Eigen/Dense>
#include "cooperative_perception/track_matching.hpp"
#include "cooperative_perception/detection.hpp"
#include "cooperative_perception/track.hpp"
#include "cooperative_perception/common_visitors.hpp"
#include "cooperative_perception/visitor.hpp"

namespace cooperative_perception
{
auto computeCovarianceIntersection(const Eigen::VectorXf& mean1, const Eigen::MatrixXf& inverse_covariance1,
                                   const Eigen::VectorXf& mean2, const Eigen::MatrixXf& inverse_covariance2,
                                   float weight) -> std::tuple<Eigen::VectorXf, Eigen::MatrixXf>
{
  const auto inverse_covariance_combined{ weight * inverse_covariance1 + (1 - weight) * inverse_covariance2 };
  const auto covariance_combined{ inverse_covariance_combined.inverse() };
  const auto mean_combined{ covariance_combined *
                            (weight * inverse_covariance1 * mean1 + (1 - weight) * inverse_covariance2 * mean2) };
  return { mean_combined, covariance_combined };
}

auto generateWeight(const Eigen::MatrixXf& inverse_covariance1, const Eigen::MatrixXf& inverse_covariance2) -> float
{
  const auto det_inverse_covariance1{ inverse_covariance1.determinant() };
  const auto det_inverse_covariance2{ inverse_covariance2.determinant() };
  const auto det_inverse_covariance_sum{ (inverse_covariance1 + inverse_covariance2).determinant() };
  const auto weight{ (det_inverse_covariance_sum - det_inverse_covariance2 + det_inverse_covariance1) /
                     (2 * det_inverse_covariance_sum) };
  return weight;
}

constexpr Visitor covariance_intersection_visitor{ [](const auto& track, const auto& detection) -> TrackVariant {
  // Compute inverse of the covariances
  const auto track_inverse_covariance{ track.covariance.inverse() };
  const auto detection_inverse_covariance{ detection.covariance.inverse() };

  // Generate weight for CI function
  const auto weight{ generateWeight(track_inverse_covariance, detection_inverse_covariance) };

  // Fuse states and covariances
  const auto [fused_state, fused_covariance]{ computeCovarianceIntersection(
      track.state.toEigenVector(track.state), track_inverse_covariance, detection.state.toEigenVector(detection.state),
      detection_inverse_covariance, weight) };

  auto fused_track{ track };
  fused_track.state = track.state.fromEigenVector(fused_state);
  fused_track.covariance = fused_covariance;

  return fused_track;
} };

// template <typename DetectionContainer, typename TrackContainer, typename FusionVisitor>
// auto fuseAssociations(const AssociationMap& associations, const DetectionContainer& detections,
//                       const TrackContainer& tracks, const FusionVisitor& fusion_visitor) -> TrackContainer

template <typename FusionVisitor>
auto fuseAssociations(const AssociationMap& associations, const std::vector<TrackVariant>& tracks,
                      const std::vector<DetectionVariant>& detections, const FusionVisitor& fusion_visitor)
    -> std::vector<TrackVariant>
{
  std::vector<TrackVariant> fused_tracks;

  for (const auto& [target_track_uuid, target_detection_uuids] : associations)
  {
    // Find the matching detection and track based on their uuids
    for (const auto& track : tracks)
    {
      const auto track_uuid{ std::visit(uuid_visitor, track) };
      if (track_uuid == target_track_uuid)
      {
        for (const auto& detection : detections)
        {
          const auto detection_uuid{ std::visit(uuid_visitor, detection) };
          if (std::find(target_detection_uuids.begin(), target_detection_uuids.end(), detection_uuid) !=
              target_detection_uuids.end())
          {
            const auto fused_track{ std::visit(fusion_visitor, track, detection) };
            fused_tracks.push_back(fused_track);
          }
        }
      }
    }
  }

  return fused_tracks;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_FUSING_HPP

// template <typename DetectionType, typename TrackType>
// auto fuseDetectionAndTrack(const DetectionType& detection, const TrackType& matched) -> TrackType
// {
//   TrackType fused_track;

//   return fused_track;
// }
// std::cout << "weight: " << weight << "\n";
// std::cout << "State for uuid [" << detection.uuid << "]:\n";
// printState(detection.state);
// std::cout << "Track uuid [" << track.uuid << "]:\n";
// std::cout << "Detection uuid [" << detection.uuid << "]:\n";
// std::cout << "\nFused state: \n";
// std::cout << fused_state;
// std::cout << "\nFused covariance: \n";
// std::cout << fused_covariance << "\n";
