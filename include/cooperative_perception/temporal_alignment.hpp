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
constexpr Visitor kStatePropagator{ [](auto& object, units::time::second_t time) {
  object.state = nextState(object.state, time - object.timestamp);
  object.timestamp = time;
} };

/**
 * @brief Get predicted Detection for specified time
 *
 * @param object DetectionType being predicted
 * @param time Prediction time
 * @return DetectionType whose state corresponds to the specified time
 */
auto objectAtTime(const DetectionType& object, units::time::second_t time) -> DetectionType
{
  DetectionType new_object{ object };

  std::visit(kStatePropagator, new_object, std::variant<units::time::second_t>(time));

  return new_object;
};

/**
 * @brief Get predicted Detections for specified time
 *
 * @param objects List of DetectionTypes being predicted
 * @param time Prediction time
 * @return List of DetectionTypes, each of whose state corresponds to the specified time
 */
auto objectsAtTime(const DetectionList& objects, units::time::second_t time) -> DetectionList
{
  DetectionList new_objects{ objects };

  std::transform(std::cbegin(new_objects), std::cend(new_objects), std::begin(new_objects),
                 [time](const DetectionType& object) { return objectAtTime(object, time); });

  return new_objects;
}

/**
 * @brief Temporally align object to a specific time step
 *
 * @param object DetectionType being predicted
 * @param time Prediction time
 * @return None, object is updated in place
 */
template <typename Detection>
auto alignToTime(Detection& object, units::time::second_t time) -> void
{
  calibrateCovariance(object);
  const auto alpha{ 1.0 };
  const auto beta{ 2.0 };
  const auto kappa{ 1.0 };
  auto [state, covariance] =
      unscentedKalmanFilterPredict(object.state, object.covariance, time - object.timestamp, alpha, kappa, beta);
  object.state = state;
  object.covariance = covariance;
  object.timestamp = time;
};

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
