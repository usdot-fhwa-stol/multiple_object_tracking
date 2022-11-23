#ifndef COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
#define COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP

#include <Eigen/Dense>
#include <units.h>
#include "cooperative_perception/units.hpp"

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

using CtrvStateCovariance = Eigen::Matrix<float, 5, 5>;

/** Calculate next CTRV state based on current state and time step
 *
 * @param[in] state Current CTRV state
 * @param[in] time_step Propagation time duration
 * @return CTRV state at end of time step
 */
auto nextState(const CtrvState& state, units::time::second_t time_step) -> CtrvState;

auto nextState(const CtrvState& state, units::time::second_t time_step,
               units::acceleration::meters_per_second_squared_t linear_accel_noise,
               units::angular_acceleration::radian_per_second_squared_t angular_accel_noise) -> CtrvState;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
