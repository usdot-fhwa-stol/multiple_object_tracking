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
 * Originally developed for Leidos by the Human and Intelligent Vehicle
 * Ensembles (HIVE) Lab at Virginia Commonwealth University (VCU).
 */

#ifndef COOPERATIVE_PERCEPTION_UUID_HPP
#define COOPERATIVE_PERCEPTION_UUID_HPP

#include <functional>
#include <ostream>
#include <string>
#include <utility>

namespace cooperative_perception
{
class Uuid
{
public:
  explicit Uuid(const std::string & uuid) : uuid_{uuid} {}
  explicit Uuid(std::string && uuid) : uuid_{std::move(uuid)} {}

  auto value() noexcept -> std::string &
  {
    return const_cast<std::string &>(std::as_const(*this).value());
  }

  auto value() const noexcept -> const std::string & { return uuid_; }

private:
  std::string uuid_;
};

inline auto operator<<(std::ostream & os, const Uuid & uuid) -> std::ostream &
{
  return os << uuid.value();
}

inline auto operator==(const Uuid & lhs, const Uuid & rhs) noexcept -> bool
{
  return lhs.value() == rhs.value();
}

inline auto operator!=(const Uuid & lhs, const Uuid & rhs) noexcept -> bool
{
  return !(lhs == rhs);
}

inline auto operator<(const Uuid & lhs, const Uuid & rhs) noexcept -> bool
{
  return lhs.value() < rhs.value();
}

inline auto operator<=(const Uuid & lhs, const Uuid & rhs) noexcept -> bool
{
  return lhs.value() <= rhs.value();
}

inline auto operator>(const Uuid & lhs, const Uuid & rhs) noexcept -> bool
{
  return lhs.value() > rhs.value();
}

inline auto operator>=(const Uuid & lhs, const Uuid & rhs) noexcept -> bool
{
  return lhs.value() >= rhs.value();
}

}  // namespace cooperative_perception

template <>
struct std::hash<cooperative_perception::Uuid>
{
  auto operator()(const cooperative_perception::Uuid & uuid) const noexcept -> std::size_t
  {
    return std::hash<std::string>{}(uuid.value());
  }
};

#endif  // COOPERATIVE_PERCEPTION_UUID_HPP
