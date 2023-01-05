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

#ifndef COOPERATIVE_PERCEPTION_ANGLE_HPP
#define COOPERATIVE_PERCEPTION_ANGLE_HPP

#include <complex>
#include <units.h>
#include <boost/math/constants/constants.hpp>
#include "cooperative_perception/utils.hpp"

namespace cooperative_perception
{
/**
 * @brief Angle type for representing and manipulating angles
 *
 * The angle class is designed to support circular mean for use in probabilistic sampling as part of the UKF prediction
 * step. https://en.wikipedia.org/wiki/Circular_mean To support the 0,2pi range required by circular mean the classes
 * stores the angle in the complex plane. https://en.wikipedia.org/wiki/Polar_coordinate_system#Complex_numbers
 */
class Angle
{
private:
  std::complex<double> value_{};

public:
  Angle(units::angle::radian_t angle_val) noexcept : value_{ units::math::cos(angle_val), units::math::sin(angle_val) }
  {
  }

  auto set_angle(units::angle::radian_t angle_val) noexcept -> void
  {
    value_.real(units::math::cos(angle_val));
    value_.imag(units::math::sin(angle_val));
  }

  auto get_angle() const noexcept -> units::angle::radian_t
  {
    auto phase_angle = std::arg(value_);
    if (phase_angle < 0.0)
    {
      phase_angle += 2.0 * boost::math::double_constants::pi;
    }
    return units::angle::radian_t{ phase_angle };
  }

  friend auto operator==(const Angle& lhs, const Angle& rhs) -> bool
  {
    return lhs.value_ == rhs.value_;
  }
};

inline auto operator+=(Angle& lhs, const Angle& rhs) noexcept -> Angle&
{
  auto angle_sum = lhs.get_angle() + rhs.get_angle();
  lhs.set_angle(angle_sum);
  return lhs;
}

inline auto operator-=(Angle& lhs, const Angle& rhs) noexcept -> Angle&
{
  auto angle_sub = lhs.get_angle() - rhs.get_angle();
  lhs.set_angle(angle_sub);
  return lhs;
}

inline auto operator+(Angle lhs, const Angle& rhs) noexcept -> Angle
{
  lhs += rhs;
  return lhs;
}

inline auto operator-(Angle lhs, const Angle& rhs) noexcept -> Angle
{
  lhs -= rhs;
  return lhs;
}

inline auto operator*=(Angle& lhs, double rhs) noexcept -> Angle&
{
  lhs.set_angle(rhs * lhs.get_angle());
  return lhs;
}

inline auto operator*(Angle lhs, double rhs) noexcept -> Angle
{
  lhs *= rhs;
  return lhs;
}

inline auto operator*(double lhs, Angle rhs) noexcept -> Angle
{
  rhs *= lhs;
  return rhs;
}

inline auto operator/=(Angle& lhs, double rhs) noexcept -> Angle&
{
  lhs.set_angle(lhs.get_angle() / rhs);
  return lhs;
}

inline auto operator/(Angle lhs, double rhs) -> Angle
{
  lhs /= rhs;
  return lhs;
}

namespace utils
{
auto almostEqual(const Angle& lhs, const Angle& rhs) -> bool
{
  // Despite its appearance, this operation checks for almost-equality. The get_angle() function returns a
  // units::angle::radian_t type. The operator==(...) overload checks for almost-equality. See
  // https://github.com/nholthaus/units/issues/118 for a discussion.
  return lhs.get_angle() == rhs.get_angle();
}
}  // namespace utils

}  // namespace cooperative_perception
#endif  // COOPERATIVE_PERCEPTION_ANGLE_HPP
