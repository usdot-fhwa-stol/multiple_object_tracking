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

#include <boost/container_hash/hash.hpp>
#include <boost/math/special_functions/next.hpp>
#include <functional>
#include <Eigen/Dense>
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

  static constexpr auto kNumVars{ 6 };

  static inline auto fromEigenVector(const Eigen::Vector<float, kNumVars>& vec) noexcept -> CtraState
  {
    return CtraState{ .position_x{ units::length::meter_t{ vec(0) } },
                      .position_y{ units::length::meter_t{ vec(1) } },
                      .velocity{ units::velocity::meters_per_second_t{ vec(2) } },
                      .yaw{ units::angle::radian_t{ vec(3) } },
                      .yaw_rate{ units::angular_velocity::radians_per_second_t{ vec(4) } },
                      .acceleration{ units::acceleration::meters_per_second_squared_t{ vec(5)} } };
  }
};

inline auto operator+=(CtraState& lhs, const CtraState& rhs) -> CtraState&
{
  lhs.position_x += rhs.position_x;
  lhs.position_y += rhs.position_y;
  lhs.velocity += rhs.velocity;
  lhs.yaw += rhs.yaw;
  lhs.yaw_rate += rhs.yaw_rate;
  lhs.acceleration += rhs.acceleration;

  return lhs;
}

inline auto operator-=(CtraState& lhs, const CtraState& rhs) -> CtraState&
{
  lhs.position_x -= rhs.position_x;
  lhs.position_y -= rhs.position_y;
  lhs.velocity -= rhs.velocity;
  lhs.yaw -= rhs.yaw;
  lhs.yaw_rate -= rhs.yaw_rate;
  lhs.acceleration -= rhs.acceleration;

  return lhs;
}

inline auto operator==(const CtraState& lhs, const CtraState& rhs) -> bool
{
  return lhs.position_x == rhs.position_x && lhs.position_y == rhs.position_y && lhs.velocity == rhs.velocity &&
         lhs.yaw == rhs.yaw && lhs.yaw_rate == rhs.yaw_rate && lhs.acceleration == rhs.acceleration;
}

inline auto operator+(CtraState lhs, const CtraState& rhs) -> CtraState
{
  lhs += rhs;
  return lhs;
}

inline auto operator-(CtraState lhs, const CtraState& rhs) -> CtraState
{
  lhs -= rhs;
  return lhs;
}

/** Calculate next CTRA state based on current state and time step
 *
 * @param[in] state Current CTRA state
 * @param[in] time_step Propagation time duration
 * @return CTRA state at end of time step
 */
auto nextState(const CtraState& state, units::time::second_t time_step) -> CtraState;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP
