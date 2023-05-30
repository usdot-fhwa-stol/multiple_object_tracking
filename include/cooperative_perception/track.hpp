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

#ifndef COOPERATIVE_PERCEPTION_TRACK_HPP
#define COOPERATIVE_PERCEPTION_TRACK_HPP

#include <variant>
#include <boost/container/static_vector.hpp>
#include <units.h>

#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/ctra_model.hpp"

namespace cooperative_perception
{
/**
 * @brief Track status
 *
 * Tracks are tentative until they have been perceived beyond a threshold, at
 * which point they become confirmed.
 */
enum class TrackStatus
{
  kConfirmed,
  kTentative
};

/**
 * @brief Tracked object (simply, Track)
 *
 * @tparam StateType State vector type for object's motion model
 * @tparam CovarianceType Covariance matrix type for object's motion model
 */
template <typename StateType, typename CovarianceType>
struct Track
{
  units::time::second_t timestamp;
  StateType state;
  CovarianceType covariance;
  TrackStatus status;
  std::string uuid;
  std::vector<std::string> associated_object_ids;
};

/**
 * @brief Track specialization for a vehicle using the CTRV motion model
 */
using CtrvTrack = Track<CtrvState, CtrvStateCovariance>;

/**
 * @brief Track specialization for a vehicle using the CTRA motion model.
 */
using CtraTrack = Track<CtraState, CtraStateCovariance>;

/**
 * @brief Aggregation of all Track specializations
 *
 * This type contains all the Track types supported by the library. It can be used to refer
 * generically to any Track and allows all Track types to be stored in the same container.
 */
using TrackType = std::variant<CtrvTrack, CtraTrack>;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_HPP
