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

#ifndef MULTIPLE_OBJECT_TRACKING_FUSING_HPP
#define MULTIPLE_OBJECT_TRACKING_FUSING_HPP

#include <Eigen/Dense>
#include <tuple>
#include <variant>
#include <cmath>

#include "multiple_object_tracking/dynamic_object.hpp"
#include "multiple_object_tracking/uuid.hpp"
#include "multiple_object_tracking/visitor.hpp"

namespace multiple_object_tracking
{
/**
 * @brief Compute the covariance intersection of two multivariate Gaussian distributions.
 *
 * This function takes the mean and inverse covariance of two Gaussian distributions and computes their covariance
 * intersection. The covariance intersection is a weighted combination of the two inverse covariances, and the weight
 * determines the influence of each distribution in the resulting covariance. The resulting mean and covariance are
 * returned as a tuple.
 *
 * @param[in] mean1 The mean of the first Gaussian distribution.
 * @param[in] inverse_covariance1 The inverse covariance of the first Gaussian distribution.
 * @param[in] mean2 The mean of the second Gaussian distribution.
 * @param[in] inverse_covariance2 The inverse covariance of the second Gaussian distribution.
 * @param[in] weight The weight (0 to 1) to combine the two inverse covariances.
 * @return A tuple containing the combined mean and covariance of the two Gaussian distributions.
 */
inline auto compute_covariance_intersection(
  const Eigen::VectorXf & mean1, const Eigen::MatrixXf & inverse_covariance1,
  const Eigen::VectorXf & mean2, const Eigen::MatrixXf & inverse_covariance2, float weight)
  -> std::tuple<Eigen::VectorXf, Eigen::MatrixXf>
{
  const auto inverse_covariance_combined{
    weight * inverse_covariance1 + (1 - weight) * inverse_covariance2};
  const auto covariance_combined{inverse_covariance_combined.inverse()};
  const auto mean_combined{
    covariance_combined *
    (weight * inverse_covariance1 * mean1 + (1 - weight) * inverse_covariance2 * mean2)};
  return {mean_combined, covariance_combined};
}

auto wrap_angle(Eigen::VectorXf & vec) {
  while (vec(3) < 0) {
    vec(3) += 2 * 3.14159265359;
  }

  while (vec(3) > 2 * 3.14159265359) {
    vec(3) -= 2 * 3.14159265359;
  }
}

inline auto compute_covariance_intersection2(
  const Eigen::VectorXf & mean1, const Eigen::MatrixXf & inverse_covariance1,
  const Eigen::VectorXf & mean2, const Eigen::MatrixXf & inverse_covariance2, float weight)
  -> std::tuple<Eigen::VectorXf, Eigen::MatrixXf>
{
  const auto inverse_covariance_combined{
    weight * inverse_covariance1 + (1 - weight) * inverse_covariance2};
  const auto covariance_combined{inverse_covariance_combined.inverse()};

  // std::cout << "original mean 1:\n" << mean1 << '\n';

  Eigen::VectorXf weighted_mean1 = weight * inverse_covariance1 * mean1;
  wrap_angle(weighted_mean1);
  // std::cout << "weighted mean 1: \n" << weighted_mean1 << '\n';

  // std::cout << "original mean 2:\n" << mean2 << '\n';

  Eigen::VectorXf weighted_mean2 = (1 - weight) * inverse_covariance2 * mean2;
  // std::cout << "weighted mean 2: \n" << weighted_mean2 << '\n';

  wrap_angle(weighted_mean2);
  // std::cout << "weighted mean 2 (wrapped):\n" << weighted_mean2 << '\n';

  // std::cout << "inverse covariance 1:\n" << inverse_covariance2 << '\n';

  Eigen::VectorXf summed_mean = weighted_mean1 + weighted_mean2;
  wrap_angle(summed_mean);

  Eigen::VectorXf mean_combined2 = covariance_combined * summed_mean;
  wrap_angle(mean_combined2);

  return {mean_combined2, covariance_combined};

  // const auto mean_combined{
  //   covariance_combined *
  //   (weight * inverse_covariance1 * mean1 + (1 - weight) * inverse_covariance2 * mean2)};
  // return {mean_combined, covariance_combined};
}

/**
 * @brief Generate the weight for covariance intersection of two Gaussian distributions.
 *
 * This function calculates the weight used in covariance intersection, which determines the influence of each Gaussian
 * distribution in the resulting covariance. It takes the inverse covariances of the two distributions and computes the
 * weight based on their determinants. The weight is then returned.
 *
 * @param[in] inverse_covariance1 The inverse covariance of the first Gaussian distribution.
 * @param[in] inverse_covariance2 The inverse covariance of the second Gaussian distribution.
 * @return The weight (0 to 1) for covariance intersection of the two Gaussian distributions.
 */
inline auto generate_weight(
  const Eigen::MatrixXf & inverse_covariance1, const Eigen::MatrixXf & inverse_covariance2) -> float
{
  const auto det_inverse_covariance1{inverse_covariance1.determinant()};
  const auto det_inverse_covariance2{inverse_covariance2.determinant()};
  const auto det_inverse_covariance_sum{(inverse_covariance1 + inverse_covariance2).determinant()};
  const auto weight{
    (det_inverse_covariance_sum - det_inverse_covariance2 + det_inverse_covariance1) /
    (2 * det_inverse_covariance_sum)};
  return weight;
}

/**
 * @brief Visitor for covariance intersection fusion of tracks and detections.
 *
 * This visitor is used to fuse the state and covariance of a track and a detection using covariance intersection.
 * It computes the inverse of the covariances, generates the weight for the covariance intersection, and fuses the
 * states and covariances. The resulting fused track is returned.
 *
 * @param[in] track The track object to be fused.
 * @param[in] detection The detection object to be fused.
 * @return The fused track with updated state and covariance.
 */
constexpr Visitor covariance_intersection_visitor{
  [](const auto & track, const auto & detection) -> std::variant<CtrvTrack, CtraTrack> {
    // Compute inverse of the covariances
    const auto track_inverse_covariance{track.covariance.inverse()};
    const auto detection_inverse_covariance{detection.covariance.inverse()};

    // Generate weight for CI function
    const auto weight{generate_weight(track_inverse_covariance, detection_inverse_covariance)};

    // Fuse states and covariances
    const auto [fused_state, fused_covariance]{compute_covariance_intersection2(
      track.state.to_eigen_vector(track.state), track_inverse_covariance,
      detection.state.to_eigen_vector(detection.state), detection_inverse_covariance, weight)};

    // Create a new fused track with updated state and covariance
    auto fused_track{track};
    fused_track.timestamp = detection.timestamp;
    fused_track.state = track.state.from_eigen_vector(fused_state);
    fused_track.covariance = fused_covariance;

    return fused_track;
  }};

/**
 * @brief Fuse track-detection associations using the provided fusion visitor.
 *
 * This function takes a map of track-detection associations, tracks, detections, and a fusion visitor. It iterates
 * through the associations, finds the matching detection and track based on their UUIDs, and then applies the fusion
 * visitor to fuse their states and covariances. The resulting fused tracks are collected in a container and returned.
 *
 * @param[in] associations The map of track-detection associations.
 * @param[in] tracks The container of track objects.
 * @param[in] detections The container of detection objects.
 * @param[in] fusion_visitor The visitor with the implementation for fusing track and detection objects.
 * @return A container of fused tracks with updated states and covariances.
 */
template <
  typename AssociationMap, typename TrackContainer, typename DetectionContainer,
  typename FusionVisitor>
auto fuse_associations(
  const AssociationMap & associations, const TrackContainer & tracks,
  const DetectionContainer & detections, const FusionVisitor & fusion_visitor) -> TrackContainer
{
  TrackContainer fused_tracks;

  for (const auto & [target_track_uuid, target_detection_uuids] : associations) {
    // Find the matching detection and track based on their uuids
    for (const auto & track : tracks) {
      const auto track_uuid{get_uuid(track)};
      if (track_uuid == target_track_uuid) {
        for (const auto & detection : detections) {
          const auto detection_uuid{get_uuid(detection)};
          if (
            std::find(
              target_detection_uuids.begin(), target_detection_uuids.end(),
              detection_uuid) != target_detection_uuids.end()) {
            const auto fused_track{std::visit(fusion_visitor, track, detection)};
            fused_tracks.push_back(fused_track);
          }
        }
      }
    }
  }

  return fused_tracks;
}

}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_FUSING_HPP
