/*
 * Copyright 2023 Leidos
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MULTIPLE_OBJECT_TRACKING_DYNAMIC_OBJECT_HPP
#define MULTIPLE_OBJECT_TRACKING_DYNAMIC_OBJECT_HPP

#include <units.h>

#include "multiple_object_tracking/uuid.hpp"
#include "multiple_object_tracking/visitor.hpp"

namespace multiple_object_tracking
{
template <typename State, typename StateCovariance, typename Tag>
struct DynamicObject
{
  using state_type = State;
  using state_covariance_type = StateCovariance;
  using tag_type = Tag;

  units::time::second_t timestamp{0.0};
  State state{};
  StateCovariance covariance{};
  Uuid uuid{""};
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

template <typename State, typename StateCovariance, typename Tag>
auto set_timestamp(
  DynamicObject<State, StateCovariance, Tag> & object, const units::time::second_t & timestamp)
{
  object.timestamp = timestamp;
}

template <typename... Alternatives>
auto set_timestamp(std::variant<Alternatives...> & object, const units::time::second_t & timestamp)
{
  std::visit(
    [](auto & o, const auto & t) { set_timestamp(o, t); }, object,
    std::variant<units::time::second_t>{timestamp});
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

template <typename State, typename StateCovariance, typename Tag>
auto set_uuid(DynamicObject<State, StateCovariance, Tag> & object, const Uuid & uuid)
{
  object.uuid = uuid;
}

template <typename... Alternatives>
auto set_uuid(std::variant<Alternatives...> & object, const Uuid & uuid)
{
  std::visit([](auto & o, const Uuid & u) { set_uuid(o, u); }, object, std::variant<Uuid>{uuid});
}

template <typename State, typename StateCovariance, typename Tag>
auto get_state(const DynamicObject<State, StateCovariance, Tag> & object)
{
  return object.state;
}

template <typename... Alternatives>
auto get_state(const std::variant<Alternatives...> & object)
{
  return std::visit([](const auto & o) { return get_state(o); }, object);
}

template <typename State, typename StateCovariance, typename Tag>
auto set_state(
  DynamicObject<State, StateCovariance, Tag> & object,
  const typename DynamicObject<State, StateCovariance, Tag>::state_type & state)
{
  object.state = state;
}

namespace detail
{
struct set_state_visitor
{
  template <typename State, typename StateCovariance, typename Tag>
  auto operator()(
    DynamicObject<State, StateCovariance, Tag> & o,
    const typename DynamicObject<State, StateCovariance, Tag>::state_type & s) const
  {
    set_state(o, s);
  }

  auto operator()(...) const { throw std::runtime_error{"state types are incompatible"}; }
};
};  // namespace detail

template <typename State, typename... Alternatives>
auto set_state(std::variant<Alternatives...> & object, const State & state)
{
  std::visit(detail::set_state_visitor{}, object, std::variant<State>{state});
}

template <typename State, typename StateCovariance, typename Tag>
auto copy_state(
  DynamicObject<State, StateCovariance, Tag> & destination,
  const DynamicObject<State, StateCovariance, Tag> & source)
{
  destination.state = destination.state;
}

template <typename... Alternatives>
auto copy_state(
  std::variant<Alternatives...> & destination, const std::variant<Alternatives...> & source)
{
  std::visit(
    Visitor{
      [](auto & dst, const auto & src) { copy_state(dst, src); },
      [](...) { throw std::runtime_error("source and destination are different types"); }},
    destination, source);
}

template <typename State, typename StateCovariance, typename Tag>
auto get_state_covariance(const DynamicObject<State, StateCovariance, Tag> & object)
{
  return object.covariance;
}

template <typename... Alternatives>
auto get_state_covariance(const std::variant<Alternatives...> & object)
{
  return std::visit([](const auto & o) { return get_state_covariance(o); }, object);
}

template <typename State, typename StateCovariance, typename Tag>
auto set_state_covariance(
  DynamicObject<State, StateCovariance, Tag> & object,
  const typename DynamicObject<State, StateCovariance, Tag>::state_covariance_type &
    state_covariance)
{
  object.covariance = state_covariance;
}

namespace detail
{
struct set_state_covariance_visitor
{
  template <typename State, typename StateCovariance, typename Tag>
  auto operator()(
    DynamicObject<State, StateCovariance, Tag> & o,
    const typename DynamicObject<State, StateCovariance, Tag>::state_covariance_type & sc) const
  {
    set_state_covariance(o, sc);
  }

  auto operator()(...) const
  {
    throw std::runtime_error{"state covariance types are incompatible"};
  }
};
};  // namespace detail

template <typename StateCovariance, typename... Alternatives>
auto set_state_covariance(
  std::variant<Alternatives...> & object, const StateCovariance & state_covariance)
{
  std::visit(
    detail::set_state_covariance_visitor{}, object,
    std::variant<StateCovariance>{state_covariance});
}

template <typename State, typename StateCovariance, typename Tag>
auto copy_state_covariance(
  DynamicObject<State, StateCovariance, Tag> & destination,
  const DynamicObject<State, StateCovariance, Tag> & source)
{
  destination.covariance = source.covariance;
}

template <typename... Alternatives>
auto copy_state_covariance(
  std::variant<Alternatives...> & destination, const std::variant<Alternatives...> & source)
{
  std::visit(
    Visitor{
      [](auto & dst, const auto & src) { copy_state_covariance(dst, src); },
      [](...) { throw std::runtime_error("source and destination are different types"); }},
    destination, source);
}

template <typename State, typename StateCovariance>
using Detection = DynamicObject<State, StateCovariance, struct DetectionTag>;

template <typename State, typename StateCovariance>
using Track = DynamicObject<State, StateCovariance, struct TrackTag>;

template <typename Track, typename Detection>
auto make_track(const Detection & detection, const Uuid & uuid) -> Track
{
  return {detection.timestamp, detection.state, detection.covariance, uuid};
}

template <typename Track, typename Detection>
auto make_track(const Detection & detection) -> Track
{
  return make_track<Track>(detection, detection.uuid);
}

template <typename Track, typename... Alternatives>
auto make_track(const std::variant<Alternatives...> & detection) -> Track
{
  return std::visit([](const auto & d) { return make_track<Track>(d); }, detection);
}

}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_DYNAMIC_OBJECT_HPP
