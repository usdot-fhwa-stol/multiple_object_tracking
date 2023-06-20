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

#ifndef COOPERATIVE_PERCEPTION_DETECTION_HPP
#define COOPERATIVE_PERCEPTION_DETECTION_HPP

#include <variant>
#include <map>
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
struct Detection
{
  units::time::second_t timestamp;
  StateType state;
  CovarianceType covariance;
  std::string uuid;
};

/**
 * @brief Detection specialization for a vehicle using the CTRV motion model
 */
using CtrvVehicleObject = Detection<CtrvState, CtrvStateCovariance>;

/**
 * @brief Detection specialization for a vehicle using the CTRA motion model.
 */
using CtraVehicleObject = Detection<CtraState, CtraStateCovariance>;

/**
 * @brief Aggregation of all Detection specializations
 *
 * This type contains all the Detection types supported by the library. It can be used to refer
 * generically to any Detection and allows all Detection types to be stored in the same container.
 */
using DetectionType = std::variant<CtrvVehicleObject, CtraVehicleObject>;

/**
 * @brief Maximum number of Detections that can be stored at a time
 *
 * To help ensure the system executes in a timely manner, the number of concurrent Detections that are being
 * tracked is limited.
 */
inline constexpr auto kMaxDetections{ 200U };

/**
 * @brief Container for Detection variables
 */
using DetectionList = boost::container::static_vector<DetectionType, kMaxDetections>;

/**
 * @brief A data store that caches the most recent Detections
 *
 *  BSM and SDSM messages report a TemporaryID, which may be assumed to be unique during
 *  a CDA CP interaction. This TemporaryID will be treated as a unique string that is associated
 *  with the most recent Detection with that ID.
 */
using DetectionCache = std::map<std::string, DetectionType>;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_DETECTION_HPP
