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
auto computeCovarianceIntersection(const Eigen::VectorXf& mean1, const Eigen::MatrixXf& covariance1,
                                   const Eigen::VectorXf& mean2, const Eigen::MatrixXf& covariance2, float weight)
    -> std::tuple<Eigen::VectorXf, Eigen::MatrixXf>
{
  Eigen::MatrixXf covariance_inv1 = covariance1.inverse();
  Eigen::MatrixXf covariance_inv2 = covariance2.inverse();
  Eigen::MatrixXf covariance_inv_combined = weight * covariance_inv1 + (1 - weight) * covariance_inv2;
  Eigen::MatrixXf covariance_combined = covariance_inv_combined.inverse();
  Eigen::VectorXf mean_combined =
      covariance_combined * (weight * covariance_inv1 * mean1 + (1 - weight) * covariance_inv2 * mean2);

  return { mean_combined, covariance_combined };
}

auto generateWeight(const Eigen::MatrixXf& inverse_covariance1, const Eigen::MatrixXf& inverse_covariance2) -> float
{
  float det_inverse_covariance1 = inverse_covariance1.determinant();
  float det_inverse_covariance2 = inverse_covariance2.determinant();
  float det_inverse_covariance_sum = (inverse_covariance1 + inverse_covariance2).determinant();

  float weight = (det_inverse_covariance_sum - det_inverse_covariance2 + det_inverse_covariance1) /
                 (2 * det_inverse_covariance_sum);
  return weight;
}

template <typename DetectionType, typename TrackType>
auto fuseDetectionAndTrack(const DetectionType& detection, const TrackType& matched) -> TrackType
{
  TrackType fused_track;

  return fused_track;
}

constexpr Visitor covariance_intersection_visitor{ [](const auto& detection, const auto& track) -> TrackVariant {
  // code here
  return track;
} };

// template <typename DetectionContainer, typename TrackContainer, typename FusionVisitor>
// auto fuseAssociations(const AssociationMap& associations, const DetectionContainer& detections,
//                       const TrackContainer& tracks, const FusionVisitor& fusion_visitor) -> TrackContainer

template <typename TrackVariant, typename DetectionVariant, typename FusionVisitor>
auto fuseAssociations(const AssociationMap& associations, const std::vector<DetectionVariant>& detections,
                      const std::vector<TrackVariant>& tracks, const FusionVisitor& fusion_visitor)
    -> std::vector<TrackVariant>
{
  std::vector<TrackVariant> fused_tracks;

  for (const auto& association : associations)
  {
    const std::string& trackId = association.first;
    const std::vector<std::string>& detectionIds = association.second;

    // Find the matching detection and track based on their IDs
    for (const auto& track : tracks)
    {
      const auto track_uuid{ std::visit(uuid_visitor, track) };
      if (trackId == track_uuid)
      {
        const auto matched_track = track;
        for (const auto& detection : detections)
        {
          const auto detection_uuid{ std::visit(uuid_visitor, detection) };
          if (std::find(detectionIds.begin(), detectionIds.end(), detection_uuid) != detectionIds.end())
          {
            const auto matched_detection = detection;
            const TrackVariant fused_track = std::visit(fusion_visitor, matched_detection, matched_track);
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
