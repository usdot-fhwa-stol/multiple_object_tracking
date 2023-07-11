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
 * @brief Visitor for performing prediction using Unscented Kalman Filter (UKF).
 *
 * The `UkfPredictionVisitor` class is a callable visitor that performs prediction on an object using the Unscented
 * Kalman Filter. It applies the prediction algorithm to update the state and covariance of the object based on the
 * provided parameters.
 */
class UkfPredictionVisitor
{
public:
  /**
   * @brief Constructs a `UkfPredictionVisitor` with the specified parameters.
   *
   * @param[in] alpha The scaling parameter for sigma points.
   * @param[in] beta The secondary scaling parameter for sigma points.
   * @param[in] kappa A tuning parameter affecting how the points are sampled.
   */
  explicit UkfPredictionVisitor(float alpha, float beta, float kappa) : alpha_(alpha), beta_(beta), kappa_(kappa)
  {
  }

  /**
   * @brief Performs prediction on the specified object using the provided time stamp.
   *
   * This function applies the Unscented Kalman Filter prediction algorithm to update the state and covariance of the
   * given object based on the time difference between the object's timestamp and the provided time stamp. The
   * prediction results are stored in the object itself.
   *
   * @tparam ObjectType The type of the object being predicted.
   * @param[in,out] object The object to be predicted.
   * @param[in] time The time stamp for prediction.
   */
  template <typename ObjectType>
  auto operator()(ObjectType& object, units::time::second_t time) const -> void
  {
    const auto [state, covariance] =
        unscentedKalmanFilterPredict(object.state, object.covariance, time - object.timestamp, alpha_, kappa_, beta_);
    object.state = state;
    object.covariance = covariance;
    object.timestamp = time;
  }

private:
  float alpha_;
  float beta_;
  float kappa_;
};

/**
 * @brief Propagate the object to a specific time stamp.
 *
 * This function calibrates the covariance of the object and applies an alignment visitor to propagate the object's
 * state to the specified time stamp.
 *
 * @param[in,out] object The object being propagated.
 * @param[in] time The propagation time.
 * @param[in] alignment_visitor The visitor with the implementation for propagating the object.
 */
template <typename ObjectType, typename AlignmentVisitor>
void propagateToTime(ObjectType& object, units::time::second_t time, const AlignmentVisitor& alignment_visitor)
{
  calibrateCovariance(object);
  alignment_visitor(object, time);
}

/**
 * @brief Predict the object's state to a specific time stamp and return a new object.
 *
 * This function creates a copy of the input object, calibrates the covariance of the new object, and applies an
 * alignment visitor to predict the state to the specified time stamp. The new predicted object is then returned.
 *
 * @param[in] object The object being predicted.
 * @param[in] time The prediction time.
 * @param[in] alignment_visitor The visitor with the implementation for predicting the object.
 * @return The new object with the predicted state.
 */
template <typename ObjectType, typename AlignmentVisitor>
auto predictToTime(const ObjectType& object, units::time::second_t time, const AlignmentVisitor& alignment_visitor)
    -> ObjectType
{
  ObjectType new_object = object;
  calibrateCovariance(new_object);
  alignment_visitor(new_object, time);
  return new_object;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
