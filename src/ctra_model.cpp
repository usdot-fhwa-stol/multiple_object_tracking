#include <cmath>
#include <units.h>
#include "cooperative_perception/ctra_model.hpp"
#include "cooperative_perception/utils.hpp"
#include "cooperative_perception/units.hpp"

namespace cooperative_perception
{

auto nextState(const CtraState& state, units::time::second_t time_step) -> CtraState
{

    return state;
}

}  // namespace cooperative_perception