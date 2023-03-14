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

#include <algorithm>
#include <gtest/gtest.h>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/unscented_transform.hpp>
#include <cooperative_perception/utils.hpp>
#include <cooperative_perception/augmented_state.hpp>
#include <cooperative_perception/angle.hpp>

namespace cp = cooperative_perception;

TEST(TestUnscentedTransform, GenerateSigmaPoints)
{
  using namespace units::literals;
  const cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                            { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                            { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                            { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                            { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };
  const auto alpha{ 1.0 };
  const auto kappa{ 1.0 };
  const auto lambda{ cp::generateLambda(state.kNumVars, alpha, kappa) };
  const auto sigma_points{ cp::generateSigmaPoints(state, covariance, lambda) };

  const std::unordered_set<cp::CtrvState> expected_sigma_points{
    cp::CtrvState{ 5.90472378_m, 1.33143932_m, 2.31696311_mps, cp::Angle(0.41932039_rad), 0.27809126_rad_per_s },
    cp::CtrvState{ 5.58347622_m, 1.42856068_m, 2.09283689_mps, cp::Angle(0.58367961_rad), 0.42750874_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.58938448_m, 2.26241076_mps, cp::Angle(0.68589429_rad), 0.50740598_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.17061552_m, 2.14738924_mps, cp::Angle(0.31710571_rad), 0.19819402_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.33348605_mps, cp::Angle(0.52331144_rad), 0.38608966_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.07631395_mps, cp::Angle(0.47968856_rad), 0.31951034_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.63405006_rad), 0.53858573_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.36894994_rad), 0.16701427_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.44602584_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.25957416_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s }

  };

  const auto is_expected = [&expected_sigma_points](const auto& point) {
    const auto result = std::find_if(std::cbegin(expected_sigma_points), std::cend(expected_sigma_points),
                                     [&point](const auto& expected_point) {
                                       return cp::utils::almostEqual(cp::utils::roundToDecimalPlace(point, 5),
                                                                     cp::utils::roundToDecimalPlace(expected_point, 5));
                                     });

    return result != std::cend(expected_sigma_points);
  };

  std::for_each(std::cbegin(sigma_points), std::cend(sigma_points),
                [&is_expected](const auto& point) { ASSERT_TRUE(is_expected(point)); });
}

TEST(TestUnscentedTransform, GenerateWeights)
{
  const auto n{ 5 };
  const auto alpha{ 1.0 };
  const auto beta{ 2.0 };
  const auto kappa{ 1.0 };
  const auto lambda{ cp::generateLambda(n, alpha, kappa) };

  // Call the function under test
  const auto [Wm, Wc] = cp::generateWeights(n, alpha, beta, lambda);

  // Define the expected output
  const std::vector<float> expected_Wm = { 0.16666667f, 0.08333333f, 0.08333333f, 0.08333333f, 0.08333333f, 0.08333333f,
                                           0.08333333f, 0.08333333f, 0.08333333f, 0.08333333f, 0.08333333f };
  const std::vector<float> expected_Wc = { 2.16666667, 0.08333333, 0.08333333, 0.08333333, 0.08333333, 0.08333333,
                                           0.08333333, 0.08333333, 0.08333333, 0.08333333, 0.08333333 };

  // Check that the function returns the expected output
  EXPECT_EQ(Wm, expected_Wm);
  EXPECT_EQ(Wc, expected_Wc);
}

// TEST(TestUnscentedTransform, CreateAugmentedSigmaPoints)
// {
//   using namespace units::literals;
//   using CtrvAugmentedState = cp::AugmentedState<cp::CtrvState, cp::CtrvProcessNoise>;
//   const CtrvAugmentedState state{ .state = cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad),
//                                                           0.3528_rad_per_s },
//                                   .process_noise = cp::CtrvProcessNoise{ 0_mps_sq, 0_rad_per_s_sq } };

//   // TODO: call new lambda function
//   const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
//                                             { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
//                                             { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
//                                             { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
//                                             { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };

//   const auto alpha{ 1.0 };
//   const auto kappa{ 1.0 };
//   const auto lambda{ cp::generateLambda(state.kNumVars, alpha, kappa) };
//   const auto sigma_points{ cp::generateSigmaPoints(state.state, covariance, lambda) };

//   const std::unordered_set<CtrvAugmentedState> expected_sigma_points{
//     CtrvAugmentedState{
//         .state{ 5.90472378_m, 1.33143932_m, 2.31696311_mps, cp::Angle(0.41932039_rad), 0.27809126_rad_per_s },
//         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{
//         .state{ 5.58347622_m, 1.42856068_m, 2.09283689_mps, cp::Angle(0.58367961_rad), 0.42750874_rad_per_s },
//         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{
//         .state{ 5.7441_m, 1.58938448_m, 2.26241076_mps, cp::Angle(0.68589429_rad), 0.50740598_rad_per_s },
//         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{
//         .state{ 5.7441_m, 1.17061552_m, 2.14738924_mps, cp::Angle(0.31710571_rad), 0.19819402_rad_per_s },
//         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.33348605_mps, cp::Angle(0.52331144_rad), 0.38608966_rad_per_s },
//                         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.07631395_mps, cp::Angle(0.47968856_rad), 0.31951034_rad_per_s },
//                         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.63405006_rad), 0.53858573_rad_per_s },
//                         .process_noise{ 0.34641_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.36894994_rad), 0.16701427_rad_per_s },
//                         .process_noise{ 0_mps_sq, 0.34641_rad_per_s_sq } },
//     CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.44602584_rad_per_s },
//                         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.25957416_rad_per_s },
//                         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
//     CtrvAugmentedState{ .state{ 55.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s },
//                         .process_noise{ 0_mps_sq, 0_rad_per_s_sq } }
//   };

//   const auto is_expected = [&expected_sigma_points](const auto& point) {
//     const auto result = std::find_if(std::cbegin(expected_sigma_points), std::cend(expected_sigma_points),
//                                      [&point](const auto& expected_point) {
//                                        return cp::utils::almostEqual(cp::utils::roundToDecimalPlace(point, 5),
//                                                                      cp::utils::roundToDecimalPlace(expected_point,
//                                                                      5));
//                                      });

//     return result != std::cend(expected_sigma_points);
//   };

//   std::for_each(std::cbegin(sigma_points), std::cend(sigma_points),
//                 [&is_expected](const auto& point) { ASSERT_TRUE(is_expected(point)); });
// }
// TEST(TestUnscentedTransform, ComputeUnscentedTransform)
// {
//   // TODO: make our own using python generated data
//   using namespace units::literals;
//   const cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
//   const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
//                                             { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
//                                             { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
//                                             { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
//                                             { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };

//   const cp::CtrvState expected_state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
//   const cp::CtrvStateCovariance expected_covariance{ { 0.00215007, -0.00065006, 0.00150001, -0.00110002, -0.00100002
//   },
//                                                      { -0.00065006, 0.00385017, 0.00055008, 0.00355007, 0.00300007 },
//                                                      { 0.00150001, 0.00055008, 0.00269991, 0.00035007, 0.00040005 },
//                                                      { -0.00110002, 0.00355007, 0.00035007, 0.00489998, 0.00499999 },
//                                                      { -0.00100002, 0.00300007, 0.00040005, 0.00499999, 0.00614999 }
//                                                      };

//   const auto transform_res{ cp::unscentedTransform(state, covariance, 1.0_s) };
//   cp::CtrvState result_state{ std::get<0>(transform_res) };
//   cp::CtrvStateCovariance result_covariance{ std::get<1>(transform_res) };

//   EXPECT_TRUE(cp::utils::almostEqual(result_state, expected_state));
//   std::cout << "\nExpected values: \n";
//   debugPrint(expected_state);
//   std::cout << "\nResult values: \n";
//   debugPrint(result_state);
//   // EXPECT_TRUE(cp::utils::almostEqual(result_covariance, expected_covariance));
// };
