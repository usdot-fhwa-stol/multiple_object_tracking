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

#ifndef COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
#define COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP

#include <boost/container_hash/hash.hpp>
#include <boost/math/special_functions/next.hpp>
#include <functional>
#include <Eigen/Dense>
#include <units.h>
#include <math.h>
#include "cooperative_perception/angle.hpp"
#include "cooperative_perception/units.hpp"

/**
 * @brief State vector for the constant turn-rate and velocity (CTRV) motion model
 */
namespace cooperative_perception
{
struct CtrvState
{
  units::length::meter_t position_x;
  units::length::meter_t position_y;
  units::velocity::meters_per_second_t velocity;
  Angle yaw;
  units::angular_velocity::radians_per_second_t yaw_rate;
  /**
   * @brief Number of elements in CTRV state vector
   */
  static constexpr auto kNumVars{ 5 };

  /**
   * @brief Convert an Eigen::Vector into a CtrvState
   *
   * @param[in] vec Vector being converted
   * @return CtrvState instance
   */
  static inline auto fromEigenVector(const Eigen::Vector<float, kNumVars>& vec) noexcept -> CtrvState
  {
    return CtrvState{ .position_x{ units::length::meter_t{ vec(0) } },
                      .position_y{ units::length::meter_t{ vec(1) } },
                      .velocity{ units::velocity::meters_per_second_t{ vec(2) } },
                      .yaw{ units::angle::radian_t{ vec(3) } },
                      .yaw_rate{ units::angular_velocity::radians_per_second_t{ vec(4) } } };
  }

  /**
   * @brief Convert a CtrvState into an Eigen::Vector
   *
   * @param[in] ctrv_state CtrvState to be converted
   * @return CtrvState instance
   */
  static inline auto toEigenVector(const CtrvState& ctrv_state) noexcept -> Eigen::Vector<float, kNumVars>
  {
    return Eigen::Vector<float, kNumVars>{ units::unit_cast<float>(ctrv_state.position_x),
                                           units::unit_cast<float>(ctrv_state.position_y),
                                           units::unit_cast<float>(ctrv_state.velocity),
                                           units::unit_cast<float>(ctrv_state.yaw.get_angle()),
                                           units::unit_cast<float>(ctrv_state.yaw_rate) };
  }
};

/**
 * @brief Add-assignment operator overload
 *
 * Adds two CtrvState variables together and stores the result in the left-hand side operand.
 *
 * @param[in] lhs Left-hand side (lhs) of the add-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the add-assignment expression
 * @return Modified left-hand side operand
 */
inline auto operator+=(CtrvState& lhs, const CtrvState& rhs) -> CtrvState&
{
  lhs.position_x += rhs.position_x;
  lhs.position_y += rhs.position_y;
  lhs.velocity += rhs.velocity;
  lhs.yaw += rhs.yaw;
  lhs.yaw_rate += rhs.yaw_rate;

  return lhs;
}

/**
 * @brief Subtract-assignment operator overload
 *
 * Subtracts two CtrvState variables together and stores the result in the left-hand side operand.
 *
 * @param[in] lhs Left-hand side (lhs) of the subtract-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the subtract-assignment expression
 * @return Modified left-hand side operand
 */
inline auto operator-=(CtrvState& lhs, const CtrvState& rhs) -> CtrvState&
{
  lhs.position_x -= rhs.position_x;
  lhs.position_y -= rhs.position_y;
  lhs.velocity -= rhs.velocity;
  lhs.yaw -= rhs.yaw;
  lhs.yaw_rate -= rhs.yaw_rate;

  return lhs;
}

/**
 * @brief Compare true equality between two CtrvStates
 *
 * This function was added to support unordered containers. It should not be used in general computations for the same
 * reasons that floating point values cannot be equated exactly.
 *
 * @param[in] lhs Left-hand side (lhs) of the equality expression
 * @param[in] rhs Right-hand side (rhs) of the equality expression
 * @return True if CtrvStates are exactly equal, false otherwise
 */
inline auto operator==(const CtrvState& lhs, const CtrvState& rhs) -> bool
{
  return lhs.position_x == rhs.position_x && lhs.position_y == rhs.position_y && lhs.velocity == rhs.velocity &&
         lhs.yaw == rhs.yaw && lhs.yaw_rate == rhs.yaw_rate;
}

/**
 * @brief Addition operator overload
 *
 * Adds two CtrvState variables together and returns the result in a new CtrvState.
 *
 * @param[in] lhs Left-hand side (lhs) of the addition expression
 * @param[in] rhs Right-hand side (rhs) of the addition expression
 * @return Operation result
 */
inline auto operator+(CtrvState lhs, const CtrvState& rhs) -> CtrvState
{
  lhs += rhs;
  return lhs;
}

/**
 * @brief Subtraction operator overload
 *
 * Subtracts two CtrvState variables together and returns the result in a new CtrvState.
 *
 * @param[in] lhs Left-hand side (lhs) of the subtraction expression
 * @param[in] rhs Right-hand side (rhs) of the subtraction expression
 * @return Operation result
 */
inline auto operator-(CtrvState lhs, const CtrvState& rhs) -> CtrvState
{
  lhs -= rhs;
  return lhs;
}

/**
 * @brief Multiply-assign operator overload
 *
 * Multiplies each member variable of a CtrvState variable by a scalar and returns the modified CtrvState.
 *
 * @param[in,out] lhs Left-hand side (lhs) of the multiplication expression, which will be modified and returned.
 * @param[in] rhs Scalar value to multiply each member variable of lhs with
 * @return Modified lhs
 */
inline auto operator*=(CtrvState& lhs, float rhs) noexcept -> CtrvState&
{
  lhs.position_x *= rhs;
  lhs.position_y *= rhs;
  lhs.velocity *= rhs;
  lhs.yaw *= rhs;
  lhs.yaw_rate *= rhs;

  return lhs;
}

/**
 * @brief Multiplication operator overload
 *
 * Multiplies a CtrvState variable with a scalar float value and returns the result in a new CtrvState.
 *
 * @param[in] lhs Left-hand side (lhs) of the multiplication expression
 * @param[in] rhs Right-hand side (rhs) of the multiplication expression
 * @return Operation result
 */
inline auto operator*(CtrvState lhs, float rhs) noexcept -> CtrvState
{
  return lhs *= rhs;
}

/**
 * @brief Multiplication operator overload
 *
 * Multiplies a scalar value with a CtrvState variable and returns the result in a new CtrvState.
 *
 * @param[in] lhs Scalar value to multiply
 * @param[in] rhs CtrvState variable to multiply
 * @return Operation result
 */
inline auto operator*(float lhs, CtrvState rhs) noexcept -> CtrvState
{
  return rhs *= lhs;
}

/**
 * @brief Covariance matrix for the CTRV motion model
 */
using CtrvStateCovariance = Eigen::MatrixXf;

/**
 * @brief Process noise vector for the CTRV motion model
 */
struct CtrvProcessNoise
{
  units::acceleration::meters_per_second_squared_t linear_acceleration;
  units::angular_acceleration::radian_per_second_squared_t angular_acceleration;

  /**
   * @brief Number of elements in the process noise state vector
   */
  static constexpr auto kNumVars{ 2 };

  /**
   * @brief Convert an Eigen::Vector into a CtrvProcessNoise
   *
   * @param[in] vec Vector being converted
   * @return CtrvProcessNoise instance
   */
  static inline auto fromEigenVector(const Eigen::Vector<float, kNumVars>& vec) noexcept -> CtrvProcessNoise
  {
    return CtrvProcessNoise{ .linear_acceleration{ units::acceleration::meters_per_second_squared_t{ vec(0) } },
                             .angular_acceleration{
                                 units::angular_acceleration::radian_per_second_squared_t{ vec(1) } } };
  }
};

/**
 * @brief Add-assignment operator overload
 *
 * Adds two CtrvProcessNoise variables together and stores the result in the left-hand side operand.
 *
 * @param[in] lhs Left-hand side (lhs) of the add-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the add-assignment expression
 * @return Modified left-hand side operand
 */
inline auto operator+=(CtrvProcessNoise& lhs, const CtrvProcessNoise& rhs) -> CtrvProcessNoise&
{
  lhs.linear_acceleration += rhs.linear_acceleration;
  lhs.angular_acceleration += rhs.angular_acceleration;

  return lhs;
}

/**
 * @brief Subtract-assignment operator overload
 *
 * Subtracts two CtrvProcessNoise variables together and stores the result in the left-hand side operand.
 *
 * @param[in] lhs Left-hand side (lhs) of the subtract-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the subtract-assignment expression
 * @return Modified left-hand side operand
 */
inline auto operator-=(CtrvProcessNoise& lhs, const CtrvProcessNoise& rhs) -> CtrvProcessNoise&
{
  lhs.linear_acceleration -= rhs.linear_acceleration;
  lhs.angular_acceleration -= rhs.angular_acceleration;

  return lhs;
}

/**
 * @brief Compare true equality between two CtrvProcessNoises
 *
 * This function was added to support unordered containers. It should not be used in general computations for the same
 * reasons that floating point values cannot be equated exactly.
 *
 * @param[in] lhs Left-hand side (lhs) of the equality expression
 * @param[in] rhs Right-hand side (rhs) of the equality expression
 * @return True if CtrvProcessNoise are exactly equal, false otherwise
 */
inline auto operator==(const CtrvProcessNoise& lhs, const CtrvProcessNoise& rhs) -> bool
{
  return lhs.linear_acceleration == rhs.linear_acceleration && lhs.angular_acceleration == rhs.angular_acceleration;
}

/**
 * @brief Addition operator overload
 *
 * Adds two CtrvProcessNoise variables together and returns the result in a new CtrvProcessNoise.
 *
 * @param[in] lhs Left-hand side (lhs) of the addition expression
 * @param[in] rhs Right-hand side (rhs) of the addition expression
 * @return Operation result
 */
inline auto operator+(CtrvProcessNoise lhs, const CtrvProcessNoise& rhs) -> CtrvProcessNoise
{
  lhs += rhs;
  return lhs;
}

/**
 * @brief Subtraction operator overload
 *
 * Subtracts two CtrvProcessNoise variables together and returns the result in a new CtrvProcessNoise.
 *
 * @param[in] lhs Left-hand side (lhs) of the subtraction expression
 * @param[in] rhs Right-hand side (rhs) of the subtraction expression
 * @return Operation result
 */
inline auto operator-(CtrvProcessNoise lhs, const CtrvProcessNoise& rhs) -> CtrvProcessNoise
{
  lhs -= rhs;
  return lhs;
}

/** Calculate next CTRV state based on current state and time step
 *
 * @param[in] state Current CTRV state
 * @param[in] time_step Propagation time duration
 * @return CTRV state at end of time step
 */
auto nextState(const CtrvState& state, units::time::second_t time_step) -> CtrvState;

/** Calculate next CTRV state based on current state, time step, and process noise
 *
 * @param[in] state Current CTRV state
 * @param[in] time_step Propagation time duration
 * @param[in] linear_accel_noise Linear acceleration process noise
 * @param[in] angular_accel_noise Angular acceleration process noise
 */
auto nextState(const CtrvState& state, units::time::second_t time_step, const CtrvProcessNoise& noise) -> CtrvState;

inline auto euclidean_distance(CtrvState lhs, CtrvState rhs) -> float
{
  const Eigen::VectorXf diff = CtrvState::toEigenVector(lhs) - CtrvState::toEigenVector(rhs);

  return std::sqrt(diff.transpose() * diff);
}

inline auto mahalanobis_distance(CtrvState mean, CtrvStateCovariance covariance, CtrvState point) -> float
{
  const Eigen::VectorXf diff = CtrvState::toEigenVector(point) - CtrvState::toEigenVector(mean);

  return std::sqrt(diff.transpose() * covariance.inverse() * diff);
}

/**
 * @brief Prints the values of a CtrvState object to the console
 *
 * This function prints the values of a CtrvState object to the console in a user-friendly format.
 * The values printed are: position_x, position_y, velocity, yaw angle, and yaw rate.
 *
 * @param[in] state The CtrvState object to print
 * @return None
 */
auto printState(const CtrvState& state) -> void;

namespace utils
{
/**
 * @brief Compares the almost-equality of two CtrvStates
 *
 * @param[in] lhs Left-hand side (lhs) of the almost-equal expression
 * @param[in] rhs Right-hand side (rhs) of the almost-equal expression
 * @param[in] ulp_tol Units of least precision (ULP) tolerance.
 *            Distance between integer representation of each vector element must be less than this.
 * @return True if CtrvState are almost-equal, false otherwise
 */
inline auto almostEqual(const CtrvState& lhs, const CtrvState& rhs, std::size_t ulp_tol = 4) -> bool
{
  const auto dist_x{ boost::math::float_distance(units::unit_cast<double>(lhs.position_x),
                                                 units::unit_cast<double>(rhs.position_x)) };
  const auto dist_y{ boost::math::float_distance(units::unit_cast<double>(lhs.position_y),
                                                 units::unit_cast<double>(rhs.position_y)) };
  const auto dist_vel{ boost::math::float_distance(units::unit_cast<double>(lhs.velocity),
                                                   units::unit_cast<double>(rhs.velocity)) };
  const auto dist_yaw{ boost::math::float_distance(units::unit_cast<double>(lhs.yaw.get_angle()),
                                                   units::unit_cast<double>(rhs.yaw.get_angle())) };
  const auto dist_yaw_rate{ boost::math::float_distance(units::unit_cast<double>(lhs.yaw_rate),
                                                        units::unit_cast<double>(rhs.yaw_rate)) };

  return std::abs(dist_x) <= ulp_tol && std::abs(dist_y) <= ulp_tol && std::abs(dist_vel) <= ulp_tol &&
         std::abs(dist_yaw) <= ulp_tol && std::abs(dist_yaw_rate) <= ulp_tol;
}

/**
 * @brief Rounds CtrvState vector elements to the nearest decimal place
 *
 * @param[in] state CtrvState being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths
 * (0.001)
 * @return Rounded CtrvState
 */
inline auto roundToDecimalPlace(const CtrvState& state, std::size_t decimal_place) -> CtrvState
{
  const auto multiplier{ std::pow(10, decimal_place) };

  CtrvState rounded_state{ units::math::round(state.position_x * multiplier) / multiplier,
                           units::math::round(state.position_y * multiplier) / multiplier,
                           units::math::round(state.velocity * multiplier) / multiplier,
                           units::math::round(state.yaw.get_angle() * multiplier) / multiplier,
                           units::math::round(state.yaw_rate * multiplier) / multiplier };

  return rounded_state;
}

/**
 * @brief Rounds CtrvCovariance matrix elements to the nearest decimal place
 *
 * @param[in] matrix CtrvCovariance matrix being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths
 * (0.001)
 * @return Rounded CtrvCovariance matrix
 */
inline auto roundToDecimalPlace(CtrvStateCovariance& matrix, std::size_t decimal_place) -> Eigen::MatrixXf
{
  const auto multiplier{ std::pow(10, decimal_place) };
  Eigen::MatrixXf out_matrix(matrix.rows(), matrix.cols());
  for (int i = 0; i < matrix.rows(); ++i)
  {
    for (int j = 0; j < matrix.cols(); ++j)
    {
      out_matrix(i, j) = round(matrix(i, j) * multiplier) / multiplier;
    }
  }
  return out_matrix;
}

/**
 * @brief Compares the almost-equality of two CtrvProcessNoises
 *
 * @param[in] lhs Left-hand side (lhs) of the almost-equal expression
 * @param[in] rhs Right-hand side (rhs) of the almost-equal expression
 * @param[in] ulp_tol Units of least precision (ULP) tolerance.
 *            Distance between integer representation of each vector element must be less than this.
 * @return True if CtrvProcessNoise are almost-equal, false otherwise
 */
inline auto almostEqual(const CtrvProcessNoise& lhs, const CtrvProcessNoise& rhs, std::size_t ulp_tol = 4) -> bool
{
  const auto dist_lin_accel{ boost::math::float_distance(units::unit_cast<double>(lhs.linear_acceleration),
                                                         units::unit_cast<double>(rhs.linear_acceleration)) };
  const auto dist_ang_accel{ boost::math::float_distance(units::unit_cast<double>(lhs.angular_acceleration),
                                                         units::unit_cast<double>(rhs.angular_acceleration)) };

  return std::abs(dist_lin_accel) <= ulp_tol && std::abs(dist_ang_accel) <= ulp_tol;
}

/**
 * @brief Rounds CtrvProcessNoise vector elements to the nearest decimal place
 *
 * @param[in] state CtrvProcessNoise being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths
 * (0.001)
 * @return Rounded CtrvProcessNoise
 */
inline auto roundToDecimalPlace(const CtrvProcessNoise& noise, std::size_t decimal_place) -> CtrvProcessNoise
{
  const auto multiplier{ std::pow(10, decimal_place) };

  return CtrvProcessNoise{ units::math::round(noise.linear_acceleration * multiplier) / multiplier,
                           units::math::round(noise.angular_acceleration * multiplier) / multiplier };
}

}  // namespace utils

}  // namespace cooperative_perception

namespace std
{
/**
 * @brief std::hash specialization for CtrvState
 *
 * This specialization is necessary to use CtrvState in unordered containers (e.g., std::unordered_set)
 */
template <>
struct hash<cooperative_perception::CtrvState>
{
  /**
   * @brief Call operator overload
   *
   * Computes the has of CtrvState when called
   *
   * @param[in] state CtrvState being hashed
   * @return hash corresponding to the CtrvState
   */
  std::size_t operator()(const cooperative_perception::CtrvState& state) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, units::unit_cast<double>(state.position_x));
    boost::hash_combine(seed, units::unit_cast<double>(state.position_y));
    boost::hash_combine(seed, units::unit_cast<double>(state.velocity));
    boost::hash_combine(seed, units::unit_cast<double>(state.yaw.get_angle()));
    boost::hash_combine(seed, units::unit_cast<double>(state.yaw_rate));

    return seed;
  }
};

/**
 * @brief std::hash specialization for CtrvProcessNoise
 *
 * This specialization is necessary to use CtrvProcessNoise in unordered containers (e.g., std::unordered_set)
 */
template <>
struct hash<cooperative_perception::CtrvProcessNoise>
{
  /**
   * @brief Call operator overload
   *
   * Computes the has of CtrvProcessNoise when called
   *
   * @param[in] state CtrvProcessNoise being hashed
   * @return hash corresponding to the CtrvProcessNoise
   */
  std::size_t operator()(const cooperative_perception::CtrvProcessNoise& process_noise) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, units::unit_cast<double>(process_noise.linear_acceleration));
    boost::hash_combine(seed, units::unit_cast<double>(process_noise.angular_acceleration));

    return seed;
  }
};
}  // namespace std

#endif  // COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
