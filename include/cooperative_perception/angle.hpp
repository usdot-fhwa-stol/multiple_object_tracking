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

#ifndef ANGLE_HPP
#define ANGLE_HPP

#include <complex>
#include <units.h>

/**
 * @brief Angle type for representing and manipulating angles
 */
namespace cooperative_perception
{
template <typename Scalar>
class Angle
{
private:
  std::complex<Scalar> value;
};

}  // namespace cooperative_perception
#endif  // ANGLE_HPP
