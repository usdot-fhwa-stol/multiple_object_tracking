#ifndef COOPERATIVE_PERCEPTION_UNITS_HPP
#define COOPERATIVE_PERCEPTION_UNITS_HPP

#include <units.h>

namespace units
{

UNIT_ADD(angular_acceleration, radian_per_second_squared, radians_per_second_squared, rad_per_s_sq,
         units::compound_unit<units::angular_velocity::radians_per_second, units::inverse<units::time::seconds>>)

}  // namespace units

#endif
