/*
 * Copyright 2022 Leidos
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

#ifndef COOPERATIVE_PERCEPTION_UTILS_HPP
#define COOPERATIVE_PERCEPTION_UTILS_HPP

#include <cmath>

namespace cooperative_perception::utils
{
/**
 * @brief Generic visitor class
 *
 * This is a utility class to make it easier to build visitors with lambdas.
 */
template <typename... Base>
struct Visitor : Base...
{
  /**
   * @brief Bring base class' operator overloads into this scope
   */
  using Base::operator()...;
};

/**
 * @brief Type deduction hint for the Visitor constructor
 */
template <typename... T>
Visitor(T&&... t) -> Visitor<T...>;

/** Check if two floating point numbers are equal
 *
 * The two numbers are considered equal if their relative distance is within an epsilon interval.
 *
 * @param[in] first First number
 * @param[in] second Second number
 * @return If the two specified numbers are equal
 */
constexpr auto almostEqual(double first, double second) -> bool
{
  constexpr auto kEpsilon{ 1e-5 };

  return std::abs(first - second) < kEpsilon;
};

}  // namespace cooperative_perception::utils

#endif  // COOPERATIVE_PERCEPTION_UTILS_HPP
