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

#include <units.h>

#include <variant>

#include "cooperative_perception/ctra_model.hpp"
#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/detection.hpp"

namespace cooperative_perception
{
/**
 * @brief Tracked object (simply, Track)
 *
 * @tparam StateType State vector type for track's motion model
 * @tparam CovarianceType Covariance matrix type for track's motion model
 */
template <typename StateType, typename CovarianceType>
struct Track
{
  units::time::second_t timestamp;
  StateType state;
  CovarianceType covariance;
  std::string uuid;

  static auto from_detection(const Detection<StateType, CovarianceType> & detection)
    -> Track<StateType, CovarianceType>
  {
    return {detection.timestamp, detection.state, detection.covariance, detection.uuid};
  }
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
using TrackVariant = std::variant<CtrvTrack, CtraTrack>;

template <typename Track, typename Detection>
auto make_track(const Detection & detection) -> Track
{
  return Track::from_detection(detection);
}

template <typename Track, typename... Alternatives>
auto make_track(const std::variant<Alternatives...> & detection) -> Track
{
  return std::visit([](const auto & d) { return make_track<Track>(d); }, detection);
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_HPP
