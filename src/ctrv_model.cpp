#include <cmath>
#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/utils.hpp"

namespace cooperative_perception
{

auto nextState(const CtrvState& state, float time_step) -> CtrvState
{
  const auto pos_x{ state[0] };
  const auto pos_y{ state[1] };
  const auto vel{ state[2] };
  const auto yaw{ state[3] };
  const auto yaw_rate{ state[4] };

  auto delta_pos_x{ 0.0F };
  auto delta_pos_y{ 0.0F };

  if (utils::almostEqual(yaw_rate, 0.0))
  {
    delta_pos_x = vel * std::cos(yaw) * time_step;
    delta_pos_y = vel * std::sin(yaw) * time_step;
  }
  else
  {
    delta_pos_x = vel / yaw_rate * (std::sin(yaw + yaw_rate * time_step) - std::sin(yaw));
    delta_pos_y = vel / yaw_rate * (-std::cos(yaw + yaw_rate * time_step) + std::cos(yaw));
  }

  const auto delta_vel{ 0 };
  const auto delta_yaw{ yaw_rate * time_step };
  const auto delta_yaw_rate{ 0 };

  CtrvState next_state;
  next_state << pos_x + delta_pos_x, pos_y + delta_pos_y, vel + delta_vel, yaw + delta_yaw, yaw_rate + delta_yaw_rate;

  return next_state;
}

}  // namespace cooperative_perception
