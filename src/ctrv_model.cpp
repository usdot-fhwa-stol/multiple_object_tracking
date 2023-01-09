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
#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/utils.hpp"
#include "cooperative_perception/units.hpp"

namespace cooperative_perception
{
auto nextState(const CtrvState& state, units::time::second_t time_step) -> CtrvState
{
  using namespace units::literals;

  CtrvState next_state{ state };

  const auto delta_yaw{ state.yaw_rate * time_step };

  if (utils::almostEqual(units::unit_cast<double>(state.yaw_rate), 0.0))
  {
    // Yaw rate of zero (no turning) is a special case. The general case is invalid because it divides by the raw rate.
    // You can't divide by zero.
    next_state.position_x += state.velocity * units::math::cos(state.yaw) * time_step;
    next_state.position_y += state.velocity * units::math::sin(state.yaw) * time_step;
  }
  else
  {
    const auto vel_over_yaw_rate{ state.velocity / state.yaw_rate };
    next_state.position_x +=
        vel_over_yaw_rate * (units::math::sin(state.yaw + delta_yaw) - units::math::sin(state.yaw)) * 1_rad;
    next_state.position_y -=
        vel_over_yaw_rate * (units::math::cos(state.yaw + delta_yaw) - units::math::cos(state.yaw)) * 1_rad;
  }

  next_state.yaw += delta_yaw;

  return next_state;
}

auto nextState(const CtrvState& state, units::time::second_t time_step, const CtrvProcessNoise& noise) -> CtrvState
{
  auto next_state{ nextState(state, time_step) };

  const auto time_step_sq{ units::math::pow<2>(time_step) };
  constexpr auto one_half{ 1.0 / 2.0 };

  next_state.position_x += one_half * noise.linear_acceleration * units::math::cos(state.yaw) * time_step_sq;
  next_state.position_y += one_half * noise.linear_acceleration * units::math::sin(state.yaw) * time_step_sq;
  next_state.velocity += noise.linear_acceleration * time_step;
  next_state.yaw += one_half * noise.angular_acceleration * time_step_sq;
  next_state.yaw_rate += noise.angular_acceleration * time_step;

  return next_state;
}

}  // namespace cooperative_perception
