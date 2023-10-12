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

#ifndef COOPERATIVE_PERCEPTION_UNITS_HPP
#define COOPERATIVE_PERCEPTION_UNITS_HPP

#include <units.h>

namespace units
{
/**
 * @brief Add angular acceleration to the units library
 *
 * The library does not come with an angular acceleration unit, so we must add one.
 */
UNIT_ADD(angular_acceleration, radian_per_second_squared, radians_per_second_squared, rad_per_s_sq,
         units::compound_unit<units::angular_velocity::radians_per_second, units::inverse<units::time::seconds>>)

}  // namespace units

namespace cooperative_perception
{
template <typename Unit>
constexpr auto remove_units(const units::unit_t<Unit> & value) noexcept
{
  return units::unit_cast<typename units::unit_t<Unit>::underlying_type>(value);
}
}  // namespace cooperative_perception

#endif
