#ifndef COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP
#define COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP

#include <boost/container_hash/hash.hpp>
#include <unordered_set>
#include "cooperative_perception/ctrv_model.hpp"

namespace cooperative_perception
{

auto sampleStateDistribution(const CtrvState& state, const CtrvStateCovariance covariance, std::size_t num_points,
                             float lambda) -> std::unordered_set<CtrvState, boost::hash<CtrvState>>;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP
