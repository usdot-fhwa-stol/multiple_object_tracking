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

TEST(TestUnscentedTransform, CreateSigmaPoints)
{
  using namespace units::literals;
  const cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                            { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                            { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                            { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                            { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };

  const auto sigma_points{ cp::generateSigmaPoints(state, covariance) };

  const std::unordered_set<cp::CtrvState> expected_sigma_points{
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s },
    cp::CtrvState{ 5.85768_m, 1.34566_m, 2.28414_mps, cp::Angle(0.44339_rad), 0.299973_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.52806_m, 2.24557_mps, cp::Angle(0.631886_rad), 0.462123_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.29582_mps, cp::Angle(0.516923_rad), 0.376339_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.595227_rad), 0.48417_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.418721_rad_per_s },
    cp::CtrvState{ 5.63052_m, 1.41434_m, 2.12566_mps, cp::Angle(0.55961_rad), 0.405627_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.23194_m, 2.16423_mps, cp::Angle(0.371114_rad), 0.243477_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.11398_mps, cp::Angle(0.486077_rad), 0.329261_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.407773_rad), 0.22143_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.286879_rad_per_s },
  };
  // [5.90472378 1.33143932 2.31696311 0.41932039 0.27809126]
  // [5.58347622 1.42856068 2.09283689 0.58367961 0.42750874]
  // [5.7441     1.58938448 2.26241076 0.68589429 0.50740598]
  // [5.7441     1.17061552 2.14738924 0.31710571 0.19819402]
  // [5.7441     1.38       2.33348605 0.52331144 0.38608966]
  // [5.7441     1.38       2.07631395 0.47968856 0.31951034]
  // [5.7441     1.38       2.2049     0.63405006 0.53858573]
  // [5.7441     1.38       2.2049     0.36894994 0.16701427]
  // [5.7441     1.38       2.2049     0.5015     0.44602584]
  // [5.7441     1.38       2.2049     0.5015     0.25957416]
  // [5.7441     1.38       2.2049     0.5015     0.3528]
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

TEST(TestUnscentedTransform, CreateAugmentedSigmaPoints)
{
  using namespace units::literals;
  using CtrvAugmentedState = cp::AugmentedState<cp::CtrvState, cp::CtrvProcessNoise>;
  const CtrvAugmentedState state{ .state = cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad),
                                                          0.3528_rad_per_s },
                                  .process_noise = cp::CtrvProcessNoise{ 0_mps_sq, 0_rad_per_s_sq } };

  // TODO: call new lambda function
  const Eigen::Matrix<float, 7, 7> covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0, 0 },
                                               { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0, 0 },
                                               { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, 0, 0 },
                                               { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0, 0 },
                                               { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0, 0 },
                                               { 0, 0, 0, 0, 0, 0.04, 0 },
                                               { 0, 0, 0, 0, 0, 0, 0.04 } };

  const auto sigma_points{ cp::generateSigmaPoints(state, covariance) };

  const std::unordered_set<CtrvAugmentedState> expected_sigma_points{
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.85768_m, 1.34566_m, 2.28414_mps, cp::Angle(0.44339_rad), 0.299973_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.52806_m, 2.24557_mps, cp::Angle(0.631886_rad), 0.462123_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.29582_mps, cp::Angle(0.516923_rad), 0.376339_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.595227_rad), 0.48417_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.418721_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s },
                        .process_noise{ 0.34641_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s },
                        .process_noise{ 0_mps_sq, 0.34641_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.63052_m, 1.41434_m, 2.12566_mps, cp::Angle(0.55961_rad), 0.405627_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.23194_m, 2.16423_mps, cp::Angle(0.371114_rad), 0.243477_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.11398_mps, cp::Angle(0.486077_rad), 0.329261_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.407773_rad), 0.22143_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.286879_rad_per_s },
                        .process_noise{ 0_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s },
                        .process_noise{ -0.34641_mps_sq, 0_rad_per_s_sq } },
    CtrvAugmentedState{ .state{ 5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s },
                        .process_noise{ 0_mps_sq, -0.34641_rad_per_s_sq } }
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
TEST(TestUnscentedTransform, ComputeUnscentedTransform)
{
  // TODO: make our own using python generated data
  using namespace units::literals;
  const cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                            { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                            { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                            { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                            { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };

  const cp::CtrvState expected_state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance expected_covariance{ { 0.00215007, -0.00065006, 0.00150001, -0.00110002, -0.00100002 },
                                                     { -0.00065006, 0.00385017, 0.00055008, 0.00355007, 0.00300007 },
                                                     { 0.00150001, 0.00055008, 0.00269991, 0.00035007, 0.00040005 },
                                                     { -0.00110002, 0.00355007, 0.00035007, 0.00489998, 0.00499999 },
                                                     { -0.00100002, 0.00300007, 0.00040005, 0.00499999, 0.00614999 } };

  const auto transform_res{ cp::unscentedTransform(state, covariance, 1.0_s) };
  cp::CtrvState result_state{ std::get<0>(transform_res) };
  cp::CtrvStateCovariance result_covariance{ std::get<1>(transform_res) };

  EXPECT_TRUE(cp::utils::almostEqual(result_state, expected_state));
  std::cout << "\nExpected values: \n";
  debugPrint(expected_state);
  std::cout << "\nResult values: \n";
  debugPrint(result_state);
  // EXPECT_TRUE(cp::utils::almostEqual(result_covariance, expected_covariance));
};
