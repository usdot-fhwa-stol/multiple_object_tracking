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
#include "cooperative_perception/detection.hpp"
#include "cooperative_perception/visitor.hpp"

namespace cooperative_perception
{
/**
 * @brief State propagation visitor
 *
 * When called, this visitor will propagate the visited object's state vector and update its timestamp.
 */
constexpr Visitor state_propagation_visitor{ [](auto& object, units::time::second_t time) {
  object.state = nextState(object.state, time - object.timestamp);
  object.timestamp = time;
} };

/**
 * @brief Get predicted object for specified time
 *
 * @param object DetectionType being predicted
 * @param time Prediction time
 * @return DetectionType whose state corresponds to the specified time
 */
auto objectAtTime(const DetectionType& object, units::time::second_t time) -> DetectionType
{
  DetectionType new_object{ object };

  std::visit(state_propagation_visitor, new_object, std::variant<units::time::second_t>(time));

  return new_object;
}

/**
 * @brief Get predicted objects for specified time
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
 * @brief Propagate the object to a specific time stamp.
 *
 * This function calibrates the covariance of the object and applies an prediction visitor to propagate the object's
 * state to the specified time stamp.
 *
 * @param[in,out] object The object being propagated.
 * @param[in] time The propagation time.
 * @param[in] prediction_visitor The visitor with the implementation for propagating the object.
 */
template <typename ObjectVariant, typename PredictionVisitor>
auto propagateToTime(ObjectVariant& object, units::time::second_t time, const PredictionVisitor& prediction_visitor)
    -> void
{
  calibrateCovariance(object);
  std::visit(prediction_visitor, object, std::variant<units::time::second_t>(time));
}

/**
 * @brief Predict the object's state to a specific time stamp and return a new object.
 *
 * This function creates a copy of the input object, calibrates the covariance of the new object, and applies an
 * prediction visitor to predict the state to the specified time stamp. The new predicted object is then returned.
 *
 * @param[in] object The object being predicted.
 * @param[in] time The prediction time.
 * @param[in] prediction_visitor The visitor with the implementation for predicting the object.
 * @return The new object with the predicted state.
 */
template <typename ObjectVariant, typename PredictionVisitor>
auto predictToTime(const ObjectVariant& object, units::time::second_t time, const PredictionVisitor& prediction_visitor)
    -> ObjectVariant
{
  ObjectVariant new_object = object;
  calibrateCovariance(new_object);
  std::visit(prediction_visitor, new_object, std::variant<units::time::second_t>(time));
  return new_object;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
