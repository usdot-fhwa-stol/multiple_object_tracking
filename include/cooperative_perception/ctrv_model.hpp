#ifndef COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
#define COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP

#include <Eigen/Dense>

namespace cooperative_perception
{

constexpr auto kCtrvStateNumVars{ 5 };

using CtrvState = Eigen::Matrix<float, kCtrvStateNumVars, 1>;

/** Calculate next CTRV state based on current state and time step
 *
 * @param[in] state Current CTRV state
 * @param[in] time_step Propagation time duration
 * @return CTRV state at end of time step
 */
auto nextState(const CtrvState& state, float time_step) -> CtrvState;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CTRV_MODEL_HPP
