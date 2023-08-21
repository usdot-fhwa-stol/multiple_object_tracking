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
  Uuid uuid;
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

/**
 * @brief Get the object's UUID
 *
 * This overload works on non-std::variant types.
 *
 * @tparam Object Object's type
 *
 * @param[in] object Object from which to get the UUID
 * @return the specified object's UUID
 */
template <typename Object>
auto get_uuid(const Object & object) -> Uuid
{
  return object.uuid;
}

/**
 * @brief Get the object's UUID
 *
 * This overload works on std::variant types.
 *
 * @tparam Variants Paramater pack for the types that the variant supports
 *
 * @param[in] object Object from which to get the UUID
 * @return the specified object's UUID
 */
template <typename... Variants>
auto get_uuid(const std::variant<Variants...> & object) -> Uuid
{
  return std::visit([](const auto & o) { return get_uuid(o); }, object);
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
