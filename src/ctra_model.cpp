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

#include <cmath>
#include <units.h>
#include "cooperative_perception/ctra_model.hpp"

#include "cooperative_perception/utils.hpp"
#include "cooperative_perception/units.hpp"

namespace cooperative_perception
{
auto nextState(const CtraState& state, units::time::second_t time_step) -> CtraState
{
  using namespace units::literals;

  units::length::meter_t delta_pos_x;
  units::length::meter_t delta_pos_y;

  auto velocity_new = state.velocity + state.acceleration * time_step;
  auto yaw_new = state.yaw + state.yaw_rate * time_step;

  delta_pos_x =
      (1 / (state.yaw_rate * state.yaw_rate)) *
      (velocity_new * state.yaw_rate * units::math::sin(yaw_new) + state.acceleration * units::math::cos(yaw_new) -
       state.velocity * state.yaw_rate * units::math::sin(state.yaw) -
       state.acceleration * units::math::cos(state.yaw));

  delta_pos_y =
      (1 / (state.yaw_rate * state.yaw_rate)) *
      (-velocity_new * state.yaw_rate * units::math::cos(yaw_new) + state.acceleration * units::math::sin(yaw_new) +
       state.velocity * state.yaw_rate * units::math::cos(state.yaw) -
       state.acceleration * units::math::sin(state.yaw));

  return CtraState{ state.position_x + delta_pos_x,
                    state.position_y + delta_pos_y,
                    velocity_new,
                    yaw_new,
                    state.yaw_rate,
                    state.acceleration };
}

auto nextState(const CtraState& state, units::time::second_t time_step, const CtraProcessNoise& noise) -> CtraState
{
  auto next_state{ nextState(state, time_step) };
  const auto time_step_sq{ units::math::pow<2>(time_step) };
  constexpr auto one_half{ 1.0 / 2.0 };

  next_state.position_x += one_half * noise.linear_acceleration * units::math::cos(state.yaw) * time_step_sq;
  next_state.position_y += one_half * noise.linear_acceleration * units::math::sin(state.yaw) * time_step_sq;
  next_state.velocity += noise.linear_acceleration * time_step;
  next_state.yaw += one_half * noise.angular_acceleration * time_step_sq;
  next_state.yaw_rate += noise.angular_acceleration * time_step;
  next_state.acceleration += noise.linear_acceleration * time_step;

  return next_state;
}

}  // namespace cooperative_perception
