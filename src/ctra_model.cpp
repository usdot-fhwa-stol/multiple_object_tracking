#include <cmath>
#include <units.h>
#include "cooperative_perception/ctra_model.hpp"
#include "cooperative_perception/utils.hpp"
#include "cooperative_perception/units.hpp"

namespace cooperative_perception
{

auto nextState(const CtraState& state, units::time::second_t time_step) -> CtraState
{
    using namespace units::literals;

    units::length::meter_t delta_pos_x;
    units::length::meter_t delta_pos_y;

    auto velocity_new = state.velocity + state.acceleration * time_step;
    auto yaw_new =  state.yaw + state.yaw_rate * time_step;

    delta_pos_x =   (1 / (state.yaw_rate * state.yaw_rate)) * 
                    (velocity_new * state.yaw_rate * units::math::sin(yaw_new) + state.acceleration * units::math::cos(yaw_new) - 
                    state.velocity * state.yaw_rate * units::math::sin(state.yaw) - state.acceleration * units::math::cos(state.yaw));

    delta_pos_y =   (1 / (state.yaw_rate * state.yaw_rate)) * 
                    (-velocity_new * state.yaw_rate * units::math::cos(yaw_new) + state.acceleration * units::math::sin(yaw_new) + 
                    state.velocity * state.yaw_rate * units::math::cos(state.yaw) - state.acceleration * units::math::sin(state.yaw));

    return CtraState{ state.position_x + delta_pos_x, state.position_y + delta_pos_y, velocity_new, yaw_new, state.yaw_rate, state.acceleration};
}

}  // namespace cooperative_perception