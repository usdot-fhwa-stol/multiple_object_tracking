#ifndef COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP
#define COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP

#include <variant>
#include <boost/container/static_vector.hpp>
#include <units.h>

#include "cooperative_perception/ctrv_model.hpp"

namespace cooperative_perception
{

template <typename StateType, typename CovarianceType>
struct DetectedObject
{
  units::time::second_t timestamp;
  StateType state;
  CovarianceType covariance;
};

using VehicleObject = DetectedObject<CtrvState, CtrvStateCovariance>;

using DetectedObjectType = std::variant<VehicleObject>;

using DetectedObjectList = boost::container::static_vector<DetectedObjectType, 200>;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_DETECTED_OBJECT_HPP
