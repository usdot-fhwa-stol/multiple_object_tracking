#ifndef COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
#define COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP

#include <boost/container_hash/hash.hpp>
#include <boost/math/special_functions/next.hpp>
#include <functional>
#include <Eigen/Dense>
#include <units.h>
#include "cooperative_perception/units.hpp"

namespace cooperative_perception
{
struct CtrvState
{
  units::length::meter_t position_x;
  units::length::meter_t position_y;
  units::velocity::meters_per_second_t velocity;
  units::angle::radian_t yaw;
  units::angular_velocity::radians_per_second_t yaw_rate;

  CtrvState& operator+=(const CtrvState& rhs)
  {
    position_x += rhs.position_x;
    position_y += rhs.position_y;
    velocity += rhs.velocity;
    yaw += rhs.yaw;
    yaw_rate += rhs.yaw_rate;

    return *this;
  }

  CtrvState& operator-=(const CtrvState& rhs)
  {
    position_x -= rhs.position_x;
    position_y -= rhs.position_y;
    velocity -= rhs.velocity;
    yaw -= rhs.yaw;
    yaw_rate -= rhs.yaw_rate;

    return *this;
  }
};

inline auto operator==(const CtrvState& lhs, const CtrvState& rhs) -> bool
{
  return lhs.position_x == rhs.position_x && lhs.position_y == rhs.position_y && lhs.velocity == rhs.velocity &&
         lhs.yaw == rhs.yaw && lhs.yaw_rate == rhs.yaw_rate;
}

inline auto operator+(CtrvState lhs, const CtrvState& rhs) -> CtrvState
{
  lhs += rhs;
  return lhs;
}

inline auto operator-(CtrvState lhs, const CtrvState& rhs) -> CtrvState
{
  lhs -= rhs;
  return lhs;
}

std::size_t hash_value(const CtrvState& state);

using CtrvStateCovariance = Eigen::Matrix<float, 5, 5>;

/** Calculate next CTRV state based on current state and time step
 *
 * @param[in] state Current CTRV state
 * @param[in] time_step Propagation time duration
 * @return CTRV state at end of time step
 */
auto nextState(const CtrvState& state, units::time::second_t time_step) -> CtrvState;

auto nextState(const CtrvState& state, units::time::second_t time_step,
               units::acceleration::meters_per_second_squared_t linear_accel_noise,
               units::angular_acceleration::radian_per_second_squared_t angular_accel_noise) -> CtrvState;

namespace utils
{
inline auto almostEqual(const CtrvState& lhs, const CtrvState& rhs) -> bool
{
  static constexpr auto kUlpTol{ 4U };
  const auto dist_x{ boost::math::float_distance(units::unit_cast<double>(lhs.position_x),
                                                 units::unit_cast<double>(rhs.position_x)) };
  const auto dist_y{ boost::math::float_distance(units::unit_cast<double>(lhs.position_y),
                                                 units::unit_cast<double>(rhs.position_y)) };
  const auto dist_vel{ boost::math::float_distance(units::unit_cast<double>(lhs.velocity),
                                                   units::unit_cast<double>(rhs.velocity)) };
  const auto dist_yaw{ boost::math::float_distance(units::unit_cast<double>(lhs.yaw),
                                                   units::unit_cast<double>(rhs.yaw)) };
  const auto dist_yaw_rate{ boost::math::float_distance(units::unit_cast<double>(lhs.yaw_rate),
                                                        units::unit_cast<double>(rhs.yaw_rate)) };

  return std::abs(dist_x) <= kUlpTol && std::abs(dist_y) <= kUlpTol && std::abs(dist_vel) <= kUlpTol &&
         std::abs(dist_yaw) <= kUlpTol && std::abs(dist_yaw_rate) <= kUlpTol;
}

inline auto roundToDecimalPlaces(const CtrvState& state, std::size_t decimal_places) -> CtrvState
{
  const auto multiplier{ std::pow(10, decimal_places) };

  CtrvState rounded_state{ units::math::round(state.position_x * multiplier) / multiplier,
                           units::math::round(state.position_y * multiplier) / multiplier,
                           units::math::round(state.velocity * multiplier) / multiplier,
                           units::math::round(state.yaw * multiplier) / multiplier,
                           units::math::round(state.yaw_rate * multiplier) / multiplier };

  return rounded_state;
}

}  // namespace utils

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
