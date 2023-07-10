/*
 * Copyright 2022 Leidos
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

#ifndef COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
#define COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP

#include <variant>
#include <units.h>
#include <stdexcept>
#include "cooperative_perception/covariance_calibration.hpp"
#include "cooperative_perception/unscented_kalman_filter.hpp"
#include "cooperative_perception/detection.hpp"
#include "cooperative_perception/visitor.hpp"

namespace cooperative_perception
{
/**
 * @brief State propagation visitor
 *
 * When called, this visitor will propagate the visited object's state vector and update its timestamp.
 */
constexpr Visitor state_propagation_visitor{ [](auto& detection, units::time::second_t time) {
  detection.state = nextState(detection.state, time - detection.timestamp);
  detection.timestamp = time;
} };

/**
 * @brief Get predicted Detection for specified time
 *
 * @param detection DetectionType being predicted
 * @param time Prediction time
 * @return DetectionType whose state corresponds to the specified time
 */
auto detectionAtTime(const DetectionType& detection, units::time::second_t time) -> DetectionType
{
  DetectionType new_detection{ detection };

  std::visit(state_propagation_visitor, new_detection, std::variant<units::time::second_t>(time));

  return new_detection;
};

/**
 * @brief Get predicted Detections for specified time
 *
 * @param detections List of DetectionTypes being predicted
 * @param time Prediction time
 * @return List of DetectionTypes, each of whose state corresponds to the specified time
 */
auto detectionsAtTime(const DetectionList& detections, units::time::second_t time) -> DetectionList
{
  DetectionList new_detections{ detections };

  std::transform(std::cbegin(new_detections), std::cend(new_detections), std::begin(new_detections),
                 [time](const DetectionType& detection) { return detectionAtTime(detection, time); });

  return new_detections;
}

/**
 * @brief UKF prediction visitor
 *
 * When called, this visitor will predict the visited object's state vector and covariance.
 */
constexpr Visitor ukf_prediction_visitor{ [](auto& detection, units::time::second_t time) {
  const auto alpha{ 1.0 };
  const auto beta{ 2.0 };
  const auto kappa{ 1.0 };
  auto [state, covariance] = unscentedKalmanFilterPredict(detection.state, detection.covariance,
                                                          time - detection.timestamp, alpha, kappa, beta);
  detection.state = state;
  detection.covariance = covariance;
  detection.timestamp = time;
} };

/**
 * @brief Propagate detection to a specific time stamp
 *
 * @param detection Detection being propagated
 * @param time Propagation time
 * @param alignment_visitor Visitor with implementation for propagating detection
 * @return None, object is updated in place
 */
template <typename DetectionType, typename AlignmentVisitor>
auto propagateToTime(DetectionType& detection, units::time::second_t time, const AlignmentVisitor& alignment_visitor)
    -> void
{
  calibrateCovariance(detection);
  std::variant<DetectionType> detection_variant{ detection };
  std::visit(alignment_visitor, detection_variant, std::variant<units::time::second_t>(time));
  detection = std::get<DetectionType>(detection_variant);
};

/**
 * @brief Predict detection to a specific time stamp
 *
 * @param detection Detection being predicted
 * @param time Prediction time
 * @param alignment_visitor Visitor with implementation for predicting detection
 * @return New predicted detection
 */
template <typename DetectionType, typename AlignmentVisitor>
auto predictToTime(const DetectionType& detection, units::time::second_t time,
                   const AlignmentVisitor& alignment_visitor) -> DetectionType
{
  calibrateCovariance(detection);
  std::variant<DetectionType> detection_variant{ detection };
  std::visit(alignment_visitor, detection_variant, std::variant<units::time::second_t>(time));
  DetectionType new_predicted_detection = std::get<DetectionType>(detection_variant);
  return new_predicted_detection;
};

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
