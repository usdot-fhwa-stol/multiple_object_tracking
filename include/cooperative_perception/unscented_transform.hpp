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

#include <tuple>
#include <math.h>
#include <vector>
#include <units.h>
#include <Eigen/Dense>
#include <unordered_set>
#include "cooperative_perception/utils.hpp"

namespace cooperative_perception
{
/**
 * @brief Generate the scaling factor lambda for generating sigma points
 * @param[in] n Dimension of state vector
 * @param[in] alpha Scaling parameter for sigma points
 * @param[in] kappa Secondary scaling parameter for sigma points
 * @return Scaling factor lambda
 */
inline auto generateLambda(int n, float alpha, float kappa) -> float
{
  return alpha * alpha * (n + kappa) - n;
}

/**
 * @brief This function generates weights for the Unscented Kalman Filter.
 * @param[in] n Dimension of state vector
 * @param[in] alpha Scaling parameter for sigma points
 * @param[in] kappa Secondary scaling parameter for sigma points
 * @param[in] lambda A tuning parameter affecting how the points are sampled
 * @return tuple with Wm: vector of weights for mean calculation and
 * Wc: vector of weights for covariance calculation
 */
inline auto generateWeights(int n, float alpha, float beta, float lambda)
    -> std::tuple<Eigen::VectorXf, Eigen::VectorXf>
{
  float wm_0 = lambda / (n + lambda);
  float wc_0 = (lambda / (n + lambda)) + (1 - alpha * alpha + beta);
  float wm_i = 0.5 * (1 / (n + lambda));
  Eigen::VectorXf Wm(2 * n + 1);
  Eigen::VectorXf Wc(2 * n + 1);
  Wm[0] = wm_0;
  Wc[0] = wc_0;
  Wm.segment(1, 2 * n).setConstant(wm_i);
  Wc.segment(1, 2 * n).setConstant(wm_i);
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
inline auto generateSigmaPoints(const State& state, const StateCovariance& covariance, const float& lambda)
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

/**
 * @brief This function takes a vector and stacks it vertically into a matrix with the specified number of rows.
 * @param[in] vector The input vector to be stacked into the matrix
 * @param[in] num_rows The number of rows in the output matrix
 * @return The stacked matrix with the input vector repeated for each row
 */
inline auto stackVectorIntoMatrix(Eigen::VectorXf vector, int num_rows) -> Eigen::MatrixXf
{
  int num_cols = vector.size();
  Eigen::MatrixXf result(num_rows, num_cols);
  for (int i = 0; i < num_rows; i++)
  {
    result.row(i) = vector;
  }
  return result;
}

/**
 * @brief This function takes a std::vector of floats and returns an Eigen::VectorXf.
 * @param[in] input The input std::vector of floats to be converted into an Eigen::VectorXf.
 * @return The resulting Eigen::VectorXf that has the same values as the input std::vector.
 */
inline auto vectorToVectorXf(const std::vector<float>& input) -> Eigen::VectorXf
{
  Eigen::VectorXf output(input.size());
  for (int i = 0; i < input.size(); i++)
  {
    output(i) = input[i];
  }
  return output;
}

/**
 * @brief This function takes a state and a set of sigma points and returns them as a matrix.
 * @tparam State A class representing the state variables.
 * @param[in] state The current state variables.
 * @param[in] sigma_points The set of sigma points to be converted into a matrix.
 * @return A matrix containing the state and sigma points as rows.
 */
template <typename State>
inline auto sigmaSetToMatrixXf(const State& state, const std::unordered_set<State>& sigma_points) -> Eigen::MatrixXf
{
  Eigen::MatrixXf matrix(std::size(sigma_points) + 1, State::kNumVars);
  matrix.row(0) = State::toEigenVector(state).transpose();
  auto i{ 1 };
  for (const auto& sigma_point : sigma_points)
  {
    matrix.row(i) = State::toEigenVector(sigma_point).transpose();
    i++;
  }
  return matrix;
}

/**
 *@brief This function performs the unscented transform on a set of sigma points. It computes the weighted mean and
 *covariance of the given sigma points.
 *@param[in] sigmas Matrix of sigma points. The first row of sigmas should correspond to the mean of the distribution.
 *@param[in] Wm Vector of weights used to compute the weighted mean of the sigma points.
 *@param[in] Wc Vector of weights used to compute the weighted covariance of the sigma points.
 *@return A tuple containing the weighted mean and weighted covariance of the sigma points.
 */
inline auto computeUnscentedTransform(const Eigen::MatrixXf& sigmas, const Eigen::VectorXf& Wm,
                                      const Eigen::VectorXf& Wc) -> std::tuple<Eigen::VectorXf, Eigen::MatrixXf>
{
  Eigen::VectorXf x = Wm.transpose() * sigmas;
  Eigen::MatrixXf y = sigmas - stackVectorIntoMatrix(x, sigmas.rows());
  Eigen::MatrixXf P = y.transpose() * (Wc.asDiagonal() * y);
  return { x, P };
}

/**
 * This function generates the sigma points and weights required for the Unscented Kalman Filter.
 * It computes the scaling factor lambda based on the provided alpha and kappa parameters.
 * The function then generates the sigma points using the given state and covariance, along with the computed lambda.
 * Additionally, it calculates the weights for mean and covariance calculations using the provided alpha, beta, and
 * lambda.
 *
 * @tparam State The type of the state vector.
 * @tparam StateCovariance The type of the covariance matrix.
 *
 * @param[in] state The initial state vector.
 * @param[in] covariance The covariance matrix associated with the state.
 * @param[in] alpha The scaling parameter for sigma points in the unscented transform.
 * @param[in] beta The secondary scaling parameter for sigma points in the unscented transform.
 * @param[in] kappa The secondary scaling parameter for sigma points in the unscented transform.
 *
 * @return A tuple containing the generated sigma points, weight vector for mean calculation, and weight vector for
 * covariance calculation.
 */
template <typename StateType, typename CovarianceType>
inline auto generateSigmaPointsAndWeights(const StateType& state, const CovarianceType& covariance, const float alpha,
                                          const float beta, const float kappa)
    -> std::tuple<std::unordered_set<StateType>, Eigen::VectorXf, Eigen::VectorXf>
{
  const auto lambda{ generateLambda(state.kNumVars, alpha, kappa) };
  const auto sigma_points{ generateSigmaPoints(state, covariance, lambda) };
  const auto [Wm, Wc] = generateWeights(state.kNumVars, alpha, beta, lambda);
  return { sigma_points, Wm, Wc };
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

/**
 * @brief Compares the almost-equality of two Eigen::VectorXf
 *
 * @param[in] lhs Left-hand side (lhs) of the almost-equal expression
 * @param[in] rhs Right-hand side (rhs) of the almost-equal expression
 *
 * @return True if vectors are almost-equal, false otherwise
 */
inline auto almostEqual(const Eigen::VectorXf& lhs, const Eigen::VectorXf& rhs) -> bool
{
  if (lhs.size() != rhs.size())
  {
    return false;  // vectors are not equal if they have different sizes
  }

  for (int i = 0; i < lhs.size(); ++i)
  {
    if (!almostEqual(lhs[i], rhs[i]))
    {
      return false;  // vectors are not equal if any of their elements differ
    }
  }

  return true;  // vectors are equal if all their elements are the same
}

/**
 * @brief Rounds a float to the nearest decimal place. Useful for comparing covariance values
 *
 * @param[in] n float being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths
 * (0.001)
 * @return Rounded float
 */
inline auto roundToDecimalPlace(float n, std::size_t decimal_place) -> float
{
  const auto multiplier{ std::pow(10.0f, decimal_place) };

  float x = round(n * multiplier) / multiplier;
  return x;
}

/**
 * @brief Rounds a Eigen::MatrixXf to the nearest decimal place. Useful for comparing covariance values
 *
 * @param[in] n Eigen::MatrixXf being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths
 * (0.001)
 * @return Rounded Eigen::MatrixXf
 */
inline auto roundToDecimalPlace(const Eigen::MatrixXf& n, std::size_t decimal_place) -> Eigen::MatrixXf
{
  Eigen::MatrixXf x(n.rows(), n.cols());
  for (int i = 0; i < n.rows(); ++i)
  {
    for (int j = 0; j < n.cols(); ++j)
    {
      x(i, j) = roundToDecimalPlace(n(i, j), decimal_place);
    }
  }
  return x;
}

/**
 * @brief Compares the almost-equality of two Eigen::MatrixXf
 *
 * @param[in] lhs Left-hand side (lhs) of the almost-equal expression
 * @param[in] rhs Right-hand side (rhs) of the almost-equal expression
 *
 * @return True if vectors are almost-equal, false otherwise
 */
inline auto almostEqual(const Eigen::MatrixXf& lhs, const Eigen::MatrixXf& rhs) -> bool
{
  if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols())
  {
    return false;  // matrices are not equal if they have different dimensions
  }

  for (int i = 0; i < lhs.rows(); ++i)
  {
    for (int j = 0; j < lhs.cols(); ++j)
    {
      if (!almostEqual(lhs(i, j), rhs(i, j)))
      {
        return false;  // matrices are not equal if any of their elements differ
      }
    }
  }

  return true;  // matrices are equal if all their elements are the same
}

}  // namespace utils

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP
