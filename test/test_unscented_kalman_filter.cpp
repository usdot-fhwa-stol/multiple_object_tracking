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

#include <gtest/gtest.h>
#include <units.h>
#include <cooperative_perception/unscented_kalman_filter.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/utils.hpp>
#include <cooperative_perception/angle.hpp>

namespace cp = cooperative_perception;

/**
 * Test the ComputeUnscentedTransform function given a state, covariance and time step
 */
TEST(TestUnscentedKalmanFilter, CtrvPrediction)
{
  using namespace units::literals;

  // Declaring Initial state and covariance
  const cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                            { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                            { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                            { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                            { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };
  // Expected values
  const cp::CtrvState expected_state{ 7.43224_m, 2.73933_m, 2.2049_mps, cp::Angle(0.8543_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance expected_covariance{ { 0.0650073, -0.0670999, 0.00564003, -0.0463523, -0.0240175 },
                                                     { -0.0670999, 0.11094, 0.00625031, 0.0654438, 0.0333096 },
                                                     { 0.00564003, 0.00625031, 0.0054, 0.0015, 0.000800002 },
                                                     { -0.0463523, 0.0654438, 0.0015, 0.0421, 0.0223 },
                                                     { -0.0240175, 0.0333096, 0.000800002, 0.0223, 0.0123 } };

  // Declaring parameters for UT
  const auto alpha{ 1.0 };
  const auto beta{ 2.0 };
  const auto kappa{ 1.0 };

  const auto transform_res{ cp::unscentedKalmanFilterPredict(state, covariance, 1.0_s, alpha, beta, kappa) };
  cp::CtrvState result_state{ std::get<0>(transform_res) };
  cp::CtrvStateCovariance result_covariance{ std::get<1>(transform_res) };

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(result_state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(result_covariance, expected_covariance));
};
