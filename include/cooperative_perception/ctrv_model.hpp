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

  static constexpr auto kNumVars{ 5 };

  static inline auto fromEigenVector(const Eigen::Vector<float, kNumVars>& vec) noexcept -> CtrvState
  {
    return CtrvState{ .position_x{ units::length::meter_t{ vec(0) } },
                      .position_y{ units::length::meter_t{ vec(1) } },
                      .velocity{ units::velocity::meters_per_second_t{ vec(2) } },
                      .yaw{ units::angle::radian_t{ vec(3) } },
                      .yaw_rate{ units::angular_velocity::radians_per_second_t{ vec(4) } } };
  }
};

inline auto operator+=(CtrvState& lhs, const CtrvState& rhs) -> CtrvState&
{
  lhs.position_x += rhs.position_x;
  lhs.position_y += rhs.position_y;
  lhs.velocity += rhs.velocity;
  lhs.yaw += rhs.yaw;
  lhs.yaw_rate += rhs.yaw_rate;

  return lhs;
}

inline auto operator-=(CtrvState& lhs, const CtrvState& rhs) -> CtrvState&
{
  lhs.position_x -= rhs.position_x;
  lhs.position_y -= rhs.position_y;
  lhs.velocity -= rhs.velocity;
  lhs.yaw -= rhs.yaw;
  lhs.yaw_rate -= rhs.yaw_rate;

  return lhs;
}

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

using CtrvStateCovariance = Eigen::Matrix<float, 5, 5>;

struct CtrvProcessNoise
{
  units::acceleration::meters_per_second_squared_t linear_acceleration;
  units::angular_acceleration::radian_per_second_squared_t angular_acceleration;

  static constexpr auto kNumVars{ 2 };

  static inline auto fromEigenVector(const Eigen::Vector<float, kNumVars>& vec) noexcept -> CtrvProcessNoise
  {
    return CtrvProcessNoise{ .linear_acceleration{ units::acceleration::meters_per_second_squared_t{ vec(0) } },
                             .angular_acceleration{
                                 units::angular_acceleration::radian_per_second_squared_t{ vec(1) } } };
  }
};

inline auto operator+=(CtrvProcessNoise& lhs, const CtrvProcessNoise& rhs) -> CtrvProcessNoise&
{
  lhs.linear_acceleration += rhs.linear_acceleration;
  lhs.angular_acceleration += rhs.angular_acceleration;

  return lhs;
}

inline auto operator-=(CtrvProcessNoise& lhs, const CtrvProcessNoise& rhs) -> CtrvProcessNoise&
{
  lhs.linear_acceleration -= rhs.linear_acceleration;
  lhs.angular_acceleration -= rhs.angular_acceleration;

  return lhs;
}

inline auto operator==(const CtrvProcessNoise& lhs, const CtrvProcessNoise& rhs) -> bool
{
  return lhs.linear_acceleration == rhs.linear_acceleration && lhs.angular_acceleration == rhs.angular_acceleration;
}

inline auto operator+(CtrvProcessNoise lhs, const CtrvProcessNoise& rhs) -> CtrvProcessNoise
{
  lhs += rhs;
  return lhs;
}

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

auto nextState(const CtrvState& state, units::time::second_t time_step, const CtrvProcessNoise& noise) -> CtrvState;

namespace utils
{
inline auto almostEqual(const CtrvState& lhs, const CtrvState& rhs, std::size_t ulp_tol = 4) -> bool
{
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

  return std::abs(dist_x) <= ulp_tol && std::abs(dist_y) <= ulp_tol && std::abs(dist_vel) <= ulp_tol &&
         std::abs(dist_yaw) <= ulp_tol && std::abs(dist_yaw_rate) <= ulp_tol;
}

inline auto roundToDecimalPlace(const CtrvState& state, std::size_t decimal_place) -> CtrvState
{
  const auto multiplier{ std::pow(10, decimal_place) };

  CtrvState rounded_state{ units::math::round(state.position_x * multiplier) / multiplier,
                           units::math::round(state.position_y * multiplier) / multiplier,
                           units::math::round(state.velocity * multiplier) / multiplier,
                           units::math::round(state.yaw * multiplier) / multiplier,
                           units::math::round(state.yaw_rate * multiplier) / multiplier };

  return rounded_state;
}

inline auto almostEqual(const CtrvProcessNoise& lhs, const CtrvProcessNoise& rhs, std::size_t ulp_tol = 4) -> bool
{
  const auto dist_lin_accel{ boost::math::float_distance(units::unit_cast<double>(lhs.linear_acceleration),
                                                         units::unit_cast<double>(rhs.linear_acceleration)) };
  const auto dist_ang_accel{ boost::math::float_distance(units::unit_cast<double>(lhs.angular_acceleration),
                                                         units::unit_cast<double>(rhs.angular_acceleration)) };

  return std::abs(dist_lin_accel) <= ulp_tol && std::abs(dist_ang_accel) <= ulp_tol;
}

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
template <>
struct hash<cooperative_perception::CtrvState>
{
  std::size_t operator()(const cooperative_perception::CtrvState& state) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, units::unit_cast<double>(state.position_x));
    boost::hash_combine(seed, units::unit_cast<double>(state.position_y));
    boost::hash_combine(seed, units::unit_cast<double>(state.velocity));
    boost::hash_combine(seed, units::unit_cast<double>(state.yaw));
    boost::hash_combine(seed, units::unit_cast<double>(state.yaw_rate));

    return seed;
  }
};

template <>
struct hash<cooperative_perception::CtrvProcessNoise>
{
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
