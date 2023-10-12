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

#ifndef COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP
#define COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP

#include <units.h>

#include <Eigen/Dense>
#include <boost/container_hash/hash.hpp>
#include <boost/math/special_functions/next.hpp>
#include <cmath>
#include <functional>

#include "cooperative_perception/angle.hpp"
#include "cooperative_perception/dynamic_object.hpp"
#include "cooperative_perception/units.hpp"

/**
 * @brief State vector for the constant turn-rate and acceleration (CTRA) motion model
 */
namespace cooperative_perception
{
struct CtraState
{
  units::length::meter_t position_x{0.0};
  units::length::meter_t position_y{0.0};
  units::velocity::meters_per_second_t velocity{0.0};
  Angle yaw{units::angle::radian_t{0.0}};
  units::angular_velocity::radians_per_second_t yaw_rate{0.0};
  units::acceleration::meters_per_second_squared_t acceleration{0.0};

  /**
   * @brief Number of elements in CTRA state vector
   */
  static constexpr auto kNumVars{6};

  /**
   * @brief Convert an Eigen::Vector into a CtraState
   *
   * @param[in] vec Vector being converted
   * @return CtraState instance
   */
  static inline auto from_eigen_vector(const Eigen::Matrix<float, kNumVars, 1> & vec) noexcept
    -> CtraState
  {
    return CtraState{
      .position_x{units::length::meter_t{vec(0)}},
      .position_y{units::length::meter_t{vec(1)}},
      .velocity{units::velocity::meters_per_second_t{vec(2)}},
      .yaw{units::angle::radian_t{vec(3)}},
      .yaw_rate{units::angular_velocity::radians_per_second_t{vec(4)}},
      .acceleration{units::acceleration::meters_per_second_squared_t{vec(5)}}};
  }

  /**
   * @brief Convert a CtraState into an Eigen::Vector
   *
   * @param[in] ctra_state CtraState to be converted
   * @return Eigen::Vector representation of CTRA state
   */

  static inline auto to_eigen_vector(const CtraState & ctra_state) noexcept
    -> Eigen::Matrix<float, kNumVars, 1>
  {
    Eigen::Matrix<float, kNumVars, 1> ctra_vector;
    ctra_vector << units::unit_cast<float>(ctra_state.position_x),
      units::unit_cast<float>(ctra_state.position_y), units::unit_cast<float>(ctra_state.velocity),
      units::unit_cast<float>(ctra_state.yaw.get_angle()),
      units::unit_cast<float>(ctra_state.yaw_rate),
      units::unit_cast<float>(ctra_state.acceleration);

    return ctra_vector;
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
inline auto operator+=(CtraState & lhs, const CtraState & rhs) -> CtraState &
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
inline auto operator-=(CtraState & lhs, const CtraState & rhs) -> CtraState &
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
inline auto operator==(const CtraState & lhs, const CtraState & rhs) -> bool
{
  return lhs.position_x == rhs.position_x && lhs.position_y == rhs.position_y &&
         lhs.velocity == rhs.velocity && lhs.yaw == rhs.yaw && lhs.yaw_rate == rhs.yaw_rate &&
         lhs.acceleration == rhs.acceleration;
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
inline auto operator+(CtraState lhs, const CtraState & rhs) -> CtraState
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
inline auto operator-(CtraState lhs, const CtraState & rhs) -> CtraState
{
  lhs -= rhs;
  return lhs;
}

/**
 * @brief Covariance matrix for the CTRA motion model
 */
using CtraStateCovariance = Eigen::Matrix<float, CtraState::kNumVars, CtraState::kNumVars>;

/**
 * @brief Process noise vector for the CTRA motion model
 */
struct CtraProcessNoise
{
  units::acceleration::meters_per_second_squared_t linear_acceleration;
  units::angular_acceleration::radian_per_second_squared_t angular_acceleration;

  /**
   * @brief Number of elements in the process noise state vector
   */
  static constexpr auto kNumVars{2};

  /**
   * @brief Convert an Eigen::Vector into a CtraProcessNoise
   *
   * @param[in] vec Vector being converted
   * @return CtraProcessNoise instance
   */
  static inline auto from_eigen_vector(const Eigen::Matrix<float, kNumVars, 1> & vec) noexcept
    -> CtraProcessNoise
  {
    return CtraProcessNoise{
      .linear_acceleration{units::acceleration::meters_per_second_squared_t{vec(0)}},
      .angular_acceleration{units::angular_acceleration::radian_per_second_squared_t{vec(1)}}};
  }
};

/**
 * @brief Add-assignment operator overload
 *
 * Adds two CtraProcessNoise variables together and stores the result in the left-hand side operand.
 *
 * @param[in] lhs Left-hand side (lhs) of the add-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the add-assignment expression
 * @return Modified left-hand side operand
 */
inline auto operator+=(CtraProcessNoise & lhs, const CtraProcessNoise & rhs) -> CtraProcessNoise &
{
  lhs.linear_acceleration += rhs.linear_acceleration;
  lhs.angular_acceleration += rhs.angular_acceleration;

  return lhs;
}

/**
 * @brief Subtract-assignment operator overload
 *
 * Subtracts two CtraProcessNoise variables together and stores the result in the left-hand side operand.
 *
 * @param[in] lhs Left-hand side (lhs) of the subtract-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the subtract-assignment expression
 * @return Modified left-hand side operand
 */
inline auto operator-=(CtraProcessNoise & lhs, const CtraProcessNoise & rhs) -> CtraProcessNoise &
{
  lhs.linear_acceleration -= rhs.linear_acceleration;
  lhs.angular_acceleration -= rhs.angular_acceleration;

  return lhs;
}

/**
 * @brief Compare true equality between two CtraProcessNoises
 *
 * This function was added to support unordered containers. It should not be used in general computations for the same
 * reasons that floating point values cannot be equated exactly.
 *
 * @param[in] lhs Left-hand side (lhs) of the equality expression
 * @param[in] rhs Right-hand side (rhs) of the equality expression
 * @return True if CtraProcessNoise are exactly equal, false otherwise
 */
inline auto operator==(const CtraProcessNoise & lhs, const CtraProcessNoise & rhs) -> bool
{
  return lhs.linear_acceleration == rhs.linear_acceleration &&
         lhs.angular_acceleration == rhs.angular_acceleration;
}

/**
 * @brief Addition operator overload
 *
 * Adds two CtraProcessNoise variables together and returns the result in a new CtraProcessNoise.
 *
 * @param[in] lhs Left-hand side (lhs) of the addition expression
 * @param[in] rhs Right-hand side (rhs) of the addition expression
 * @return Operation result
 */
inline auto operator+(CtraProcessNoise lhs, const CtraProcessNoise & rhs) -> CtraProcessNoise
{
  lhs += rhs;
  return lhs;
}

/**
 * @brief Subtraction operator overload
 *
 * Subtracts two CtraProcessNoise variables together and returns the result in a new CtraProcessNoise.
 *
 * @param[in] lhs Left-hand side (lhs) of the subtraction expression
 * @param[in] rhs Right-hand side (rhs) of the subtraction expression
 * @return Operation result
 */
inline auto operator-(CtraProcessNoise lhs, const CtraProcessNoise & rhs) -> CtraProcessNoise
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
auto get_next_state(const CtraState & state, units::time::second_t time_step) -> CtraState;

/** Calculate next CTRA state based on current state, time step, and process noise
 *
 * @param[in] state Current CTRA state
 * @param[in] time_step Propagation time duration
 * @param[in] linear_accel_noise Linear acceleration process noise
 * @param[in] angular_accel_noise Angular acceleration process noise
 */
auto get_next_state(
  const CtraState & state, units::time::second_t time_step, const CtraProcessNoise & noise)
  -> CtraState;

inline auto euclidean_distance(const CtraState & lhs, const CtraState & rhs) -> float
{
  const Eigen::Vector3f lhs_pose = Eigen::Vector3f{
    units::unit_cast<float>(lhs.position_x), units::unit_cast<float>(lhs.position_y),
    units::unit_cast<float>(lhs.yaw.get_angle())};
  const Eigen::Vector3f rhs_pose = Eigen::Vector3f{
    units::unit_cast<float>(rhs.position_x), units::unit_cast<float>(rhs.position_y),
    units::unit_cast<float>(rhs.yaw.get_angle())};

  const Eigen::VectorXf diff = lhs_pose - rhs_pose;

  return std::sqrt(diff.transpose() * diff);
}

inline auto mahalanobis_distance(CtraState mean, CtraStateCovariance covariance, CtraState point)
  -> float
{
  const Eigen::Vector3f mean_pose = Eigen::Vector3f{
    units::unit_cast<float>(mean.position_x), units::unit_cast<float>(mean.position_y),
    units::unit_cast<float>(mean.yaw.get_angle())};
  const Eigen::Vector3f point_pose = Eigen::Vector3f{
    units::unit_cast<float>(point.position_x), units::unit_cast<float>(point.position_y),
    units::unit_cast<float>(point.yaw.get_angle())};

  const Eigen::VectorXf diff = mean_pose - point_pose;

  Eigen::Matrix3f pose_cov;
  pose_cov << covariance(0, 0), covariance(0, 1), covariance(0, 3), covariance(1, 0),
    covariance(1, 1), covariance(1, 3), covariance(3, 0), covariance(3, 1), covariance(3, 3);

  return std::sqrt(diff.transpose() * pose_cov.inverse() * diff);
}

/**
 * @brief Prints the values of a CtraState object to the console
 *
 * This function prints the values of a CtraState object to the console in a user-friendly format.
 * The values printed are: position_x, position_y, velocity, yaw angle, yaw rate, and acceleration.
 *
 * @param[in] state The CtraState object to print
 * @return None
 */
auto print_state(const CtraState & state) -> void;

/**
 * @brief Detection specialization for a the CTRA motion model
 */
using CtraDetection = Detection<CtraState, CtraStateCovariance>;

/**
 * @brief Track specialization for a vehicle using the CTRA motion model.
 */
using CtraTrack = Track<CtraState, CtraStateCovariance>;

namespace utils
{
/**
 * @brief Rounds CtraState vector elements to the nearest decimal place
 *
 * @param[in] state CtraState being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths (0.001)
 * @return Rounded CtraState
 */
inline auto round_to_decimal_place(const CtraState & state, std::size_t decimal_place) -> CtraState
{
  const auto multiplier{std::pow(10, decimal_place)};

  CtraState rounded_state{
    units::math::round(state.position_x * multiplier) / multiplier,
    units::math::round(state.position_y * multiplier) / multiplier,
    units::math::round(state.velocity * multiplier) / multiplier,
    units::math::round(state.yaw.get_angle() * multiplier) / multiplier,
    units::math::round(state.yaw_rate * multiplier) / multiplier,
    units::math::round(state.acceleration * multiplier) / multiplier};

  return rounded_state;
}

/**
 * @brief Rounds CtraProcessNoise vector elements to the nearest decimal place
 *
 * @param[in] state CtraProcessNoise being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths (0.001)
 * @return Rounded CtraProcessNoise
 */
inline auto round_to_decimal_place(const CtraProcessNoise & noise, std::size_t decimal_place)
  -> CtraProcessNoise
{
  const auto multiplier{std::pow(10, decimal_place)};

  return CtraProcessNoise{
    units::math::round(noise.linear_acceleration * multiplier) / multiplier,
    units::math::round(noise.angular_acceleration * multiplier) / multiplier};
}

}  // namespace utils

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP
