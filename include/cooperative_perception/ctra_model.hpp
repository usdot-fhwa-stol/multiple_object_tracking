#ifndef COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP
#define COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP

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
};

auto nextState(const CtraState& state, units::time::second_t time_step) -> CtraState;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CTRA_MODEL_HPP