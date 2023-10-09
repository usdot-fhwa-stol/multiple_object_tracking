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

#include <gtest/gtest.h>

#include <algorithm>
#include <cooperative_perception/angle.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/unscented_transform.hpp>
#include <cooperative_perception/utils.hpp>

namespace cp = cooperative_perception;

/**
 * Test the generate_sigma_points function
 */
TEST(TestUnscentedTransform, GenerateSigmaPoints)
{
  using namespace units::literals;
  const cp::CtrvState state{
    5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s};
  cp::CtrvStateCovariance covariance;
  covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
    0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, -0.0020,
    0.0060, 0.0008, 0.0100, 0.0123;
  const auto alpha{1.0};
  const auto kappa{1.0};
  const auto lambda{cp::generate_lambda(state.kNumVars, alpha, kappa)};
  const auto sigma_points{cp::generate_sigma_points(state, covariance, lambda)};

  const std::vector<cp::CtrvState> expected_sigma_points{
    cp::CtrvState{
      5.90472378_m, 1.33143932_m, 2.31696311_mps, cp::Angle(0.41932039_rad), 0.27809126_rad_per_s},
    cp::CtrvState{
      5.58347622_m, 1.42856068_m, 2.09283689_mps, cp::Angle(0.58367961_rad), 0.42750874_rad_per_s},
    cp::CtrvState{
      5.7441_m, 1.58938448_m, 2.26241076_mps, cp::Angle(0.68589429_rad), 0.50740598_rad_per_s},
    cp::CtrvState{
      5.7441_m, 1.17061552_m, 2.14738924_mps, cp::Angle(0.31710571_rad), 0.19819402_rad_per_s},
    cp::CtrvState{
      5.7441_m, 1.38_m, 2.33348605_mps, cp::Angle(0.52331144_rad), 0.38608966_rad_per_s},
    cp::CtrvState{
      5.7441_m, 1.38_m, 2.07631395_mps, cp::Angle(0.47968856_rad), 0.31951034_rad_per_s},
    cp::CtrvState{5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.63405006_rad), 0.53858573_rad_per_s},
    cp::CtrvState{5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.36894994_rad), 0.16701427_rad_per_s},
    cp::CtrvState{5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.44602584_rad_per_s},
    cp::CtrvState{5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.25957416_rad_per_s},
    cp::CtrvState{5.7441_m, 1.38_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s}

  };

  const auto is_expected = [&expected_sigma_points](const auto & point) {
    const auto result = std::find_if(
      std::cbegin(expected_sigma_points), std::cend(expected_sigma_points),
      [&point](const auto & expected_point) {
        return cp::utils::almost_equal(
          cp::utils::round_to_decimal_place(point, 5),
          cp::utils::round_to_decimal_place(expected_point, 5));
      });

    return result != std::cend(expected_sigma_points);
  };

  std::for_each(
    std::cbegin(sigma_points), std::cend(sigma_points),
    [&is_expected](const auto & point) { ASSERT_TRUE(is_expected(point)); });
}

/**
 * Test the generate_weights function
 */
TEST(TestUnscentedTransform, GenerateWeights)
{
  // Declaring parameters for UT
  const auto n{5};
  const auto alpha{1.0};
  const auto beta{2.0};
  const auto kappa{1.0};
  const auto lambda{cp::generate_lambda(n, alpha, kappa)};

  // Call the function under test
  const auto [Wm, Wc] = cp::generate_weights(n, alpha, beta, lambda);

  // Define the expected output
  Eigen::VectorXf expected_Wm(11);
  expected_Wm << 0.16666667f, 0.08333333f, 0.08333333f, 0.08333333f, 0.08333333f, 0.08333333f,
    0.08333333f, 0.08333333f, 0.08333333f, 0.08333333f, 0.08333333f;
  Eigen::VectorXf expected_Wc(11);
  expected_Wc << 2.16666667, 0.08333333, 0.08333333, 0.08333333, 0.08333333, 0.08333333, 0.08333333,
    0.08333333, 0.08333333, 0.08333333, 0.08333333;

  EXPECT_TRUE(cp::utils::almost_equal(expected_Wm, Wm));
  EXPECT_TRUE(cp::utils::almost_equal(expected_Wc, Wc));
}

/**
 * Test the ComputeUnscentedTransform function using purely Eigen matrices and vectors
 */
TEST(TestUnscentedTransform, ComputeUnscentedTransformPureEigen)
{
  using namespace Eigen;

  MatrixXf sigma_points(11, 5);
  sigma_points << 5.7441, 1.38, 2.2049, 0.5015, 0.3528, 5.90472378, 1.33143932, 2.31696311,
    0.41932039, 0.27809126, 5.58347622, 1.42856068, 2.09283689, 0.58367961, 0.42750874, 5.7441,
    1.58938448, 2.26241076, 0.68589429, 0.50740598, 5.7441, 1.17061552, 2.14738924, 0.31710571,
    0.19819402, 5.7441, 1.38, 2.33348605, 0.52331144, 0.38608966, 5.7441, 1.38, 2.07631395,
    0.47968856, 0.31951034, 5.7441, 1.38, 2.2049, 0.63405006, 0.53858573, 5.7441, 1.38, 2.2049,
    0.36894994, 0.16701427, 5.7441, 1.38, 2.2049, 0.5015, 0.44602584, 5.7441, 1.38, 2.2049, 0.5015,
    0.25957416;

  VectorXf Wm(11);
  Wm << 0.16666667, 0.08333333, 0.08333333, 0.08333333, 0.08333333, 0.08333333, 0.08333333,
    0.08333333, 0.08333333, 0.08333333, 0.08333333;

  VectorXf Wc(11);
  Wc << 2.16666667, 0.08333333, 0.08333333, 0.08333333, 0.08333333, 0.08333333, 0.08333333,
    0.08333333, 0.08333333, 0.08333333, 0.08333333;

  // Expected values
  VectorXf expected_state(5);
  expected_state << 5.7441, 1.38, 2.2049, 0.5015, 0.3528;

  MatrixXf expected_covariance(5, 5);
  expected_covariance << 0.0043, -0.0013, 0.003, -0.0022, -0.002, -0.0013, 0.0077, 0.0011, 0.0071,
    0.006, 0.003, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071, 0.0007, 0.0098, 0.01, -0.002,
    0.006, 0.0008, 0.01, 0.0123;

  const auto transform_res{cp::compute_unscented_transform(sigma_points, Wm, Wc)};
  const auto result_state{std::get<0>(transform_res)};
  const auto result_covariance{std::get<1>(transform_res)};

  EXPECT_TRUE(cp::utils::almost_equal(expected_state, result_state));
  EXPECT_TRUE(cp::utils::almost_equal(expected_covariance, result_covariance));
};
