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

/**
 * @brief State vector for the constant turn-rate and acceleration (CTRA) motion model
 */
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

   /**
   * @brief Number of elements in CTRA state vector
   */
  static constexpr auto kNumVars{ 6 };

  /**
   * @brief Convert an Eigen::Vector into a CtraState
   *
   * @param[in] vec Vector being converted
   * @return CtraState instance
   */
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

/**
 * @brief Add-assignment operator overload
 *
 * Adds two CtraState variables together and stores the result in the left-hand side operand.
 *
 * @param[in] lhs Left-hand side (lhs) of the add-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the add-assignment expression
 * @return Modified left-hand side operand
 */
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

/**
 * @brief Subtract-assignment operator overload
 *
 * Subtracts two CtraState variables together and stores the result in the left-hand side operand.
 *
 * @param[in] lhs Left-hand side (lhs) of the subtract-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the subtract-assignment expression
 * @return Modified left-hand side operand
 */
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

/**
 * @brief Compare true equality between two CtraStates
 *
 * This function was added to support unordered containers. It should not be used in general computations for the same
 * reasons that floating point values cannot be equated exactly.
 *
 * @param[in] lhs Left-hand side (lhs) of the equality expression
 * @param[in] rhs Right-hand side (rhs) of the equality expression
 * @return True if CtraStates are exactly equal, false otherwise
 */
inline auto operator==(const CtraState& lhs, const CtraState& rhs) -> bool
{
  return lhs.position_x == rhs.position_x && lhs.position_y == rhs.position_y && lhs.velocity == rhs.velocity &&
         lhs.yaw == rhs.yaw && lhs.yaw_rate == rhs.yaw_rate && lhs.acceleration == rhs.acceleration;
}

/**
 * @brief Addition operator overload
 *
 * Adds two CtraState variables together and returns the result in a new CtraState.
 *
 * @param[in] lhs Left-hand side (lhs) of the addition expression
 * @param[in] rhs Right-hand side (rhs) of the addition expression
 * @return Operation result
 */
inline auto operator+(CtraState lhs, const CtraState& rhs) -> CtraState
{
  lhs += rhs;
  return lhs;
}

/**
 * @brief Subtraction operator overload
 *
 * Subtracts two CtraState variables together and returns the result in a new CtraState.
 *
 * @param[in] lhs Left-hand side (lhs) of the subtraction expression
 * @param[in] rhs Right-hand side (rhs) of the subtraction expression
 * @return Operation result
 */
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
