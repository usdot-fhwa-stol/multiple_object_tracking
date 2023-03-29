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

#ifndef COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP
#define COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP

#include <boost/container_hash/hash.hpp>
#include <unordered_set>
#include <tuple>
#include <vector>
#include <units.h>
#include <Eigen/Dense>
#include "cooperative_perception/ctrv_model.hpp"

namespace cooperative_perception
{
/**
 * @brief Generate the scaling factor lambda for generating sigma points
 * @param[in] n Dimension of state vector
 * @param[in] alpha Scaling parameter for sigma points
 * @param[in] kappa Secondary scaling parameter for sigma points
 * @return Scaling factor lambda
 */
auto generateLambda(int n, float alpha, float kappa) -> float
{
  return alpha * alpha * (n + kappa) - n;
}

/**
 * @brief This function generates weights for the Unscented Kalman Filter.
 * @param[in] n Dimension of state vector
 * @param[in] alpha Scaling parameter for sigma points
 * @param[in] kappa Secondary scaling parameter for sigma points
 * @param[in] lambda A tuning parameter affecting how the points are sampled
 * @return Wm: vector of weights for mean calculation
 * Wc: vector of weights for covariance calculation
 */
auto generateWeights(int n, float alpha, float beta, float lambda) -> std::tuple<std::vector<float>, std::vector<float>>
{
  float wm_0 = lambda / (n + lambda);
  float wc_0 = (lambda / (n + lambda)) + (1 - alpha * alpha + beta);
  float wm_i = 0.5 * (1 / (n + lambda));
  std::vector<float> Wm(2 * n + 1);
  std::vector<float> Wc(2 * n + 1);
  Wm[0] = wm_0;
  Wc[0] = wc_0;
  std::fill(Wm.begin() + 1, Wm.end(), wm_i);
  std::fill(Wc.begin() + 1, Wc.end(), wm_i);
  return { Wm, Wc };
}

/**
 * @brief Generate sample points from a state's distribution
 *
 * @tparam State State vector type of state distribution being sampled
 * @tparam StateCovariance Covariance matrix type of state distribution being sampled
 *
 * @param[in] state Mean state vector of state distribution
 * @param[in] covariance Covariance matrix of state distribution
 * @param[in] num_points Number of points to sample
 * @param[in] lambda A tuning parameter affecting how the points are sampled
 * @return Set of sampled points
 */
template <typename State, typename StateCovariance>
auto generateSigmaPoints(const State& state, const StateCovariance& covariance, const float& lambda)
    -> std::unordered_set<State>
{
  std::unordered_set<State> sigma_pts{};
  const StateCovariance covariance_sqrt{ covariance.llt().matrixL() };
  for (const auto& column : covariance_sqrt.colwise())
  {
    const auto result{ std::sqrt(covariance.rows() + lambda) * column };
    const auto result_state{ State::fromEigenVector(result) };

    sigma_pts.insert(state + result_state);
    sigma_pts.insert(state - result_state);
  }

  return sigma_pts;
}

auto vectorToMatrix(Eigen::VectorXf vector, int num_rows) -> Eigen::MatrixXf
{
  int num_cols = vector.size();
  Eigen::MatrixXf result(num_rows, num_cols);
  for (int i = 0; i < num_rows; i++)
  {
    result.row(i) = vector;
  }
  return result;
}

auto vectorToVectorXf(const std::vector<float>& input) -> Eigen::VectorXf
{
  Eigen::VectorXf output(input.size());
  for (int i = 0; i < input.size(); i++)
  {
    output(i) = input[i];
  }
  return output;
}

template <typename State>
auto sigmaSetToMatrixXf(const State& state, const std::unordered_set<State>& sigma_points) -> Eigen::MatrixXf
{
  Eigen::MatrixXf matrix(std::size(sigma_points) + 1, State::kNumVars);
  matrix.row(0) = State::toEigenVector(state).transpose();
  auto i = 1;
  for (const auto& sigma_point : sigma_points)
  {
    matrix.row(i) = State::toEigenVector(sigma_point).transpose();
    i++;
  }

  return matrix;
}

auto unscentedTransform(const Eigen::MatrixXf& sigmas, const Eigen::VectorXf& Wm, const Eigen::VectorXf& Wc)
    -> std::tuple<Eigen::VectorXf, Eigen::MatrixXf>
{
  Eigen::VectorXf x = Wm.transpose() * sigmas;
  Eigen::MatrixXf x1 = vectorToMatrix(x, sigmas.rows());
  Eigen::MatrixXf y = sigmas - x1;
  Eigen::MatrixXf P = y.transpose() * (Wc.asDiagonal() * y);
  return std::make_tuple(x, P);
}

template <typename State, typename StateCovariance>
auto computeUnscentedTransform(const State& state, const StateCovariance& covariance, units::time::second_t time_step)
    -> std::tuple<State, StateCovariance>
{
  // Declaring parameters for UT
  const auto alpha{ 1.0 };
  const auto beta{ 2.0 };
  const auto kappa{ 1.0 };
  const auto lambda{ generateLambda(state.kNumVars, alpha, kappa) };
  const auto sigma_points{ generateSigmaPoints(state, covariance, lambda) };

  // Generating weights
  const auto [vector_Wm, vector_Wc] = generateWeights(state.kNumVars, alpha, beta, lambda);

  // Convert weights in Eigen::VectorXf
  const auto Wm{ vectorToVectorXf(vector_Wm) };
  const auto Wc{ vectorToVectorXf(vector_Wc) };

  // TODO: Advance state and sigma_points to nextState before applying UT

  // Convert sigma points into Eigen::MatrixXf
  const auto m_sigma_points{ sigmaSetToMatrixXf(state, sigma_points) };

  // Compute UT based on the sigma points and weights
  const auto transform_res{ unscentedTransform(m_sigma_points, Wm, Wc) };
  const auto result_state_vector{ std::get<0>(transform_res) };
  const auto result_covariance_matrix{ std::get<1>(transform_res) };

  const auto result_state{ State::fromEigenVector(result_state_vector) };
  const CtrvStateCovariance result_covariance{ result_covariance_matrix };
  return { result_state, result_covariance };
}

namespace utils
{
/**
 * @brief Compares the almost-equality of two vector of floats
 *
 * @param[in] lhs Left-hand side (lhs) of the almost-equal expression
 * @param[in] rhs Right-hand side (rhs) of the almost-equal expression
 *
 * @return True if vectors are almost-equal, false otherwise
 */
inline auto almostEqual(const std::vector<float>& lhs, const std::vector<float>& rhs) -> bool
{
  if (lhs.size() != rhs.size())
  {
    return false;
  }

  for (int i = 0; i < lhs.size(); ++i)
  {
    if (!almostEqual(lhs[i], rhs[i]))
    {
      return false;
    }
  }

  return true;
}
}  // namespace utils

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP
