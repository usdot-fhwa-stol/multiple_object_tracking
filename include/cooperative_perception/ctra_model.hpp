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
#ifndef COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP
#define COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP

#include <units.h>
#include "cooperative_perception/units.hpp"

namespace cooperative_perception
{
struct CtraState
{
  units::length::meter_t position_x;
  units::length::meter_t position_y;
  units::velocity::meters_per_second_t velocity;
  units::angle::radian_t yaw;
  units::angular_velocity::radians_per_second_t yaw_rate;
  units::acceleration::meters_per_second_squared_t acceleration;
};

/** Calculate next CTRA state based on current state and time step
 *
 * @param[in] state Current CTRA state
 * @param[in] time_step Propagation time duration
 * @return CTRA state at end of time step
 */
auto nextState(const CtraState& state, units::time::second_t time_step) -> CtraState;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP
