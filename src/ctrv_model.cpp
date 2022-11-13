#include <cmath>
#include <units.h>
#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/utils.hpp"

namespace cooperative_perception
{

auto nextState(const CtrvState& state, units::time::second_t time_step) -> CtrvState
{
  units::length::meter_t delta_pos_x;
  units::length::meter_t delta_pos_y;

  if (utils::almostEqual(units::unit_cast<double>(state.yaw_rate), 0.0))
  {
    delta_pos_x = state.velocity * units::math::cos(state.yaw) * time_step;
    delta_pos_y = state.velocity * units::math::sin(state.yaw) * time_step;
  }
  else
  {
    delta_pos_x = state.velocity / state.yaw_rate * units::angle::radian_t(1) *
                  (units::math::sin(state.yaw + state.yaw_rate * time_step) - units::math::sin(state.yaw));
    delta_pos_y = state.velocity / state.yaw_rate * units::angle::radian_t(1) *
                  (-units::math::cos(state.yaw + state.yaw_rate * time_step) + units::math::cos(state.yaw));
  }

  using namespace units::literals;

  const auto delta_vel{ 0_m / 1_s };
  const units::angle::radian_t delta_yaw{ state.yaw_rate * time_step };
  const auto delta_yaw_rate{ 0_rad / 1_s };

  return CtrvState{ state.position_x + delta_pos_x, state.position_y + delta_pos_y, state.velocity + delta_vel,
                    state.yaw + delta_yaw, state.yaw_rate + delta_yaw_rate };
}

}  // namespace cooperative_perception
