#ifndef COOPERATIVE_PERCEPTION_DYNAMIC_OBJECT_HPP
#define COOPERATIVE_PERCEPTION_DYNAMIC_OBJECT_HPP

#include <units.h>

#include "cooperative_perception/uuid.hpp"

namespace cooperative_perception
{
template <typename State, typename StateCovariance, typename Tag>
struct DynamicObject
{
  using state_type = State;
  using state_covariance_type = StateCovariance;
  using tag_type = Tag;

  units::time::second_t timestamp;
  State state;
  StateCovariance covariance;
  std::string uuid;
};

template <typename Object>
auto get_timestamp(const Object & object) -> units::time::second_t
{
  return object.timestamp;
}

template <typename... Alternatives>
auto get_timestamp(const std::variant<Alternatives...> & object) -> units::time::second_t
{
  return std::visit([](const auto & o) { return get_timestamp(o); }, object);
}

template <typename State, typename StateCovariance>
using Detection = DynamicObject<State, StateCovariance, struct DetectionTag>;

template <typename State, typename StateCovariance>
using Track = DynamicObject<State, StateCovariance, struct TrackTag>;

template <typename Track, typename Detection>
auto make_track(const Detection & detection) -> Track
{
  return {detection.timestamp, detection.state, detection.covariance, detection.uuid};
}

template <typename Track, typename... Alternatives>
auto make_track(const std::variant<Alternatives...> & detection) -> Track
{
  return std::visit([](const auto & d) { return make_track<Track>(d); }, detection);
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_DYNAMIC_OBJECT_HPP
