#ifndef COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
#define COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP

#include <Eigen/Dense>
#include <units.h>

namespace cooperative_perception
{

struct CtrvState
{
  units::length::meter_t position_x;
  units::length::meter_t position_y;
  units::velocity::meters_per_second_t velocity;
  units::angle::radian_t yaw;
  units::angular_velocity::radians_per_second_t yaw_rate;
};

/** Calculate next CTRV state based on current state and time step
 *
 * @param[in] state Current CTRV state
 * @param[in] time_step Propagation time duration
 * @return CTRV state at end of time step
 */
auto nextState(const CtrvState& state, units::time::second_t time_step) -> CtrvState;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
