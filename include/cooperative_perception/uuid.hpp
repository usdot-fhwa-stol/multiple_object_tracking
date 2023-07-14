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

/*
 * Developed by the Human and Vehicle Ensembles (HIVE) Lab at Virginia Commonwealth University (VCU)
 */

#ifndef COOPERATIVE_PERCEPTION_UUID_HPP
#define COOPERATIVE_PERCEPTION_UUID_HPP

#include <algorithm>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace cooperative_perception
{
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
auto get_uuid(const Object& object) -> std::string
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
auto get_uuid(const std::variant<Variants...>& object) -> std::string
{
  return std::visit([](const auto& o) { return o.uuid; }, object);
}

/**
 * @brief Get the UUIDs of several objects
 *
 * @tparam Container Container holding the objects
 * @tparam ContainerParams Parameter pack for the container's parameter types
 *
 * @param[in] objects Objects from which to get the UUID
 * @return the specified objects' UUIDs
 */
template <template <typename...> typename Container, typename... ContainerParams>
auto get_uuids(const Container<ContainerParams...>& objects) -> Container<std::string>
{
  Container<std::string> uuids;

  if constexpr (std::is_same_v<Container<ContainerParams...>, std::vector<ContainerParams...>>)
  {
    std::ignore = std::transform(std::cbegin(objects), std::cend(objects), std::back_inserter(uuids),
                                 [](const auto& object) { return get_uuid(object); });
  }
  else
  {
    std::ignore = std::transform(std::cbegin(objects), std::cend(objects), std::inserter(uuids, std::end(uuids)),
                                 [](const auto& object) { return get_uuid(object); });
  }

  return uuids;
}

namespace detail
{
/**
 * @brief Copy UUIDs of objects matching predicate
 *
 * @tparam Container Container holding the objects
 * @tparam OutputIt Iterator type for the destination container
 * @tparam UnaryPredicate Type of predicate being used
 *
 * @param objects Objects from which to get the UUID
 * @param uuids Output iterator to the destination container holding the UUIDS
 * @param predicate Predicate to decides if each object should have its UUID gotten
 * @return void
 */
template <typename Container, typename OutputIt, typename UnaryPredicate>
auto get_uuids_if_helper(const Container& objects, OutputIt uuids, const UnaryPredicate& predicate) -> void
{
  for (const auto& object : objects)
  {
    const auto uuid{ get_uuid(object) };
    if (predicate(uuid))
    {
      *uuids = uuid;
      ++uuids;
    }
  }
}

}  // namespace detail

/**
 * @brief Get the UUIDs of several objects if the match a predicate
 *
 * @tparam Container Container holding the objects
 * @tparam UnaryPredicate Type of predicate being used
 * @tparam ContainerParams Parameter pack for the container's parameter types
 *
 * @param[in] objects Objects from which to get the UUID
 * @param[in] predicate Predicate to decide if each object should have its UUID gotten
 * @return UUIDs for object matching the predicate
 */
template <template <typename...> typename Container, typename UnaryPredicate, typename... ContainerParams>
auto get_uuids_if(const Container<ContainerParams...>& objects, const UnaryPredicate& predicate)
    -> Container<std::string>
{
  Container<std::string> uuids;

  if constexpr (std::is_same_v<Container<ContainerParams...>, std::vector<ContainerParams...>>)
  {
    detail::get_uuids_if_helper(objects, std::back_inserter(uuids), predicate);
  }
  else
  {
    detail::get_uuids_if_helper(objects, std::inserter(uuids, std::end(uuids)), predicate);
  }

  return uuids;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_UUID_HPP
