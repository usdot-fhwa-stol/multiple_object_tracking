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

#ifndef COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
#define COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP

#include <variant>
#include <units.h>
#include "cooperative_perception/covariance_calibration.hpp"
#include "cooperative_perception/unscented_transform.hpp"
#include "cooperative_perception/detection.hpp"
#include "cooperative_perception/visitor.hpp"

namespace cooperative_perception
{
/**
 * @brief State propagation visitor
 *
 * When called, this visitor will propagate the visited object's state vector and update its timestamp.
 *
 * @param[in,out] object Object whose state will be propagated
 * @param[in] time Time stamp to which the object's state will be propagated
 */
constexpr Visitor kStatePropagator{ [](auto& object, units::time::second_t time) {
  object.state = nextState(object.state, time - object.timestamp);
  object.timestamp = time;
} };

/**
 * @brief Temporally align detection to a specific time step
 *
 * @param[in,out] detection DetectionType being predicted
 * @param[in] time Prediction time
 * @return void; detection is updated in place
 */
template <typename DetectionType>
auto alignToTime(DetectionType& detection, units::time::second_t time) -> void
{
  calibrateCovariance(detection);
  auto [state, covariance] =
      computeUnscentedTransform(detection.state, detection.covariance, time - detection.timestamp);
  detection.state = state;
  detection.covariance = covariance;
  detection.timestamp = time;
};

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
