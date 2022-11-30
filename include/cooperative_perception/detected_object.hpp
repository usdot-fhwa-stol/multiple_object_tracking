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

#ifndef COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP
#define COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP

#include <variant>
#include <boost/container/static_vector.hpp>
#include <units.h>

#include "cooperative_perception/ctrv_model.hpp"

namespace cooperative_perception
{
template <typename StateType, typename CovarianceType>
struct DetectedObject
{
  units::time::second_t timestamp;
  StateType state;
  CovarianceType covariance;
};

using VehicleObject = DetectedObject<CtrvState, CtrvStateCovariance>;

using DetectedObjectType = std::variant<VehicleObject>;

using DetectedObjectList = boost::container::static_vector<DetectedObjectType, 200>;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP
