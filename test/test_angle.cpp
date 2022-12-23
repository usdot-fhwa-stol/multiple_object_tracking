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

#include <gtest/gtest.h>
#include <cooperative_perception/angle.hpp>
#include <units.h>

using namespace units::literals;

namespace cp = cooperative_perception;

/**
 * Test CTRV nextState function against pure rotation
 */
TEST(TestAngle, AngleAdditiveOperations)
{
  const auto pi = cp::Angle{ 3.1459_rad };
  auto pi_2 = cp::Angle{ 1.5707_rad };
  auto pi_4 = cp::Angle{ 0.7854_rad };
}
