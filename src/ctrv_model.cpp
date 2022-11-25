#include <cmath>
#include <units.h>
#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/utils.hpp"
#include "cooperative_perception/units.hpp"

namespace cooperative_perception
{
auto nextState(const CtrvState& state, units::time::second_t time_step) -> CtrvState
{
  using namespace units::literals;

  units::length::meter_t delta_pos_x;
  units::length::meter_t delta_pos_y;

  if (utils::almostEqual(units::unit_cast<double>(state.yaw_rate), 0.0))
  {
    delta_pos_x = state.velocity * units::math::cos(state.yaw) * time_step;
    delta_pos_y = state.velocity * units::math::sin(state.yaw) * time_step;
  }
  else
  {
    delta_pos_x = state.velocity / state.yaw_rate *
                  (units::math::sin(state.yaw + state.yaw_rate * time_step) - units::math::sin(state.yaw)) * 1_rad;
    delta_pos_y = state.velocity / state.yaw_rate *
                  (-units::math::cos(state.yaw + state.yaw_rate * time_step) + units::math::cos(state.yaw)) * 1_rad;
  }

  const auto delta_vel{ 0_mps };
  const units::angle::radian_t delta_yaw{ state.yaw_rate * time_step };
  const auto delta_yaw_rate{ 0_rad_per_s };

  return CtrvState{ state.position_x + delta_pos_x, state.position_y + delta_pos_y, state.velocity + delta_vel,
                    state.yaw + delta_yaw, state.yaw_rate + delta_yaw_rate };
}

auto nextState(const CtrvState& state, units::time::second_t time_step, const CtrvProcessNoise& noise) -> CtrvState
{
  auto next_state{ nextState(state, time_step) };

  const units::length::meter_t delta_x{ 1.0 / 2.0 * noise.linear_acceleration * units::math::cos(state.yaw) *
                                        units::math::pow<2>(time_step) };
  const units::length::meter_t delta_y{ 1.0 / 2.0 * noise.linear_acceleration * units::math::sin(state.yaw) *
                                        units::math::pow<2>(time_step) };
  const units::velocity::meters_per_second_t delta_velocity{ noise.linear_acceleration * time_step };
  const units::angle::radian_t delta_yaw{ 1.0 / 2.0 * noise.angular_acceleration * units::math::pow<2>(time_step) };
  const units::angular_velocity::radians_per_second_t delta_yaw_rate{ noise.angular_acceleration * time_step };

  next_state.position_x += delta_x;
  next_state.position_y += delta_y;
  next_state.velocity += delta_velocity;
  next_state.yaw += delta_yaw;
  next_state.yaw_rate += delta_yaw_rate;

  return next_state;
}

}  // namespace cooperative_perception
