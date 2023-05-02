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

#ifndef COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP
#define COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP

#include <variant>
#include <boost/container/static_vector.hpp>
#include <units.h>

#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/ctra_model.hpp"

namespace cooperative_perception
{
/**
 * @brief Detected object
 *
 * @tparam StateType State vector type for object's motion model
 * @tparam CovarianceType Covariance matrix type for object's motion model
 */
template <typename StateType, typename CovarianceType>
struct DetectedObject
{
  units::time::second_t timestamp;
  StateType state;
  CovarianceType covariance;
};

/**
 * @brief DetectedObject specialization for a vehicle using the CTRV motion model
 */
using CtrvVehicleObject = DetectedObject<CtrvState, CtrvStateCovariance>;

/**
 * @brief DetectedObject specialization for a vehicle using the CTRA motion model.
 */
using CtraVehicleObject = DetectedObject<CtraState, CtraStateCovariance>;

/**
 * @brief Aggregation of all DetectedObject specializations
 *
 * This type contains all the DetectedObject types supported by the library. It can be used to refer
 * generically to any DetectedObject and allows all DetectedObject types to be stored in the same container.
 */
using DetectedObjectType = std::variant<CtrvVehicleObject, CtraVehicleObject>;

/**
 * @brief Maximum number of DetectedObjects that can be stored at a time
 *
 * To help ensure the system executes in a timely manner, the number of concurrent DetectedObjects that are being
 * tracked is limited.
 */
inline constexpr auto kMaxDetectedObjects{ 200U };

/**
 * @brief Container for DetectedObject variables
 */
using DetectedObjectList = boost::container::static_vector<DetectedObjectType, kMaxDetectedObjects>;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP
