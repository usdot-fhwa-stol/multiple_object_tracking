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

#include "cooperative_perception/ctra_model.hpp"

#include <units.h>

#include <cmath>

#include "cooperative_perception/units.hpp"
#include "cooperative_perception/utils.hpp"

namespace cooperative_perception
{
auto get_next_state(const CtraState & state, units::time::second_t time_step) -> CtraState
{
  using namespace units::literals;

  units::length::meter_t delta_pos_x;
  units::length::meter_t delta_pos_y;

  auto velocity_new = state.velocity + state.acceleration * time_step;
  auto yaw_new = state.yaw + Angle(state.yaw_rate * time_step);

  // Note: units library overloads operator== to use almost-equal semantics
  if (state.yaw_rate == units::angular_velocity::radians_per_second_t{0.0}) {
    // Yaw rate of zero (no turning) is a special case. The general case is invalid because it divides by the raw rate.
    // You can't divide by zero.
    delta_pos_x = (state.velocity * time_step + (state.acceleration * time_step * time_step) / 2) *
                  units::math::cos(state.yaw.get_angle());
    delta_pos_y = (state.velocity * time_step + (state.acceleration * time_step * time_step) / 2) *
                  units::math::sin(state.yaw.get_angle());
  } else {
    delta_pos_x = (1_rad / (state.yaw_rate * state.yaw_rate)) *
                  (velocity_new * state.yaw_rate * units::math::sin(yaw_new.get_angle()) +
                   state.acceleration * units::math::cos(yaw_new.get_angle()) * 1_rad -
                   state.velocity * state.yaw_rate * units::math::sin(state.yaw.get_angle()) -
                   state.acceleration * units::math::cos(state.yaw.get_angle()) * 1_rad);

    delta_pos_y = (1_rad / (state.yaw_rate * state.yaw_rate)) *
                  (-velocity_new * state.yaw_rate * units::math::cos(yaw_new.get_angle()) +
                   state.acceleration * units::math::sin(yaw_new.get_angle()) * 1_rad +
                   state.velocity * state.yaw_rate * units::math::cos(state.yaw.get_angle()) -
                   state.acceleration * units::math::sin(state.yaw.get_angle()) * 1_rad);
  }
  return CtraState{
    state.position_x + delta_pos_x,
    state.position_y + delta_pos_y,
    velocity_new,
    yaw_new,
    state.yaw_rate,
    state.acceleration};
}

auto get_next_state(
  const CtraState & state, units::time::second_t time_step, const CtraProcessNoise & noise)
  -> CtraState
{
  auto next_state{get_next_state(state, time_step)};
  const auto time_step_sq{units::math::pow<2>(time_step)};
  constexpr auto one_half{1.0 / 2.0};

  next_state.position_x +=
    one_half * noise.linear_acceleration * units::math::cos(state.yaw.get_angle()) * time_step_sq;
  next_state.position_y +=
    one_half * noise.linear_acceleration * units::math::sin(state.yaw.get_angle()) * time_step_sq;
  next_state.velocity += noise.linear_acceleration * time_step;
  next_state.yaw += Angle(one_half * noise.angular_acceleration * time_step_sq);
  next_state.yaw_rate += noise.angular_acceleration * time_step;
  next_state.acceleration += noise.linear_acceleration;

  return next_state;
}

auto print_state(const CtraState & state) -> void
{
  std::cout << "CtraState: \n";
  std::cout << "x: " << state.position_x << "\n";
  std::cout << "y: " << state.position_y << "\n";
  std::cout << "velocity: " << state.velocity << "\n";
  std::cout << "yaw: " << state.yaw.get_angle() << "\n";
  std::cout << "yaw rate: " << state.yaw_rate << "\n";
  std::cout << "acceleration: " << state.acceleration << "\n";
}

}  // namespace cooperative_perception
