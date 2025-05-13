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

#ifndef MULTIPLE_OBJECT_TRACKING_UNSCENTED_TRANSFORM_HPP
#define MULTIPLE_OBJECT_TRACKING_UNSCENTED_TRANSFORM_HPP

#include <Eigen/Dense>
#include <tuple>
#include <vector>
#include "multiple_object_tracking/utils.hpp"

namespace multiple_object_tracking
{
/**
 * @brief Generate the scaling factor lambda for generating sigma points
 * @param[in] n Dimension of state vector
 * @param[in] alpha Scaling parameter for sigma points
 * @param[in] kappa Secondary scaling parameter for sigma points
 * @return Scaling factor lambda
 */
inline auto generate_lambda(int n, float alpha, float kappa) -> float
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
inline auto generate_weights(int n, float alpha, float beta, float lambda)
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
  return {Wm, Wc};
}

/**
 * @brief Generate sample points from a state's distribution
 *
 * @tparam StateType State vector type of state distribution being sampled
 * @tparam CovarianceType Covariance matrix type of state distribution being sampled
 *
 * @param[in] state Mean state vector of state distribution
 * @param[in] covariance Covariance matrix of state distribution
 * @param[in] num_points Number of points to sample
 * @param[in] lambda A tuning parameter affecting how the points are sampled
 * @return Vector of sampled points
 */
template <typename StateType, typename CovarianceType>
inline auto generate_sigma_points(
  const StateType & state, const CovarianceType & covariance, const float & lambda)
  -> std::vector<StateType>
{
  std::vector<StateType> sigma_points{};
  const CovarianceType covariance_sqrt{covariance.llt().matrixL()};
  for (auto column{0U}; column < covariance_sqrt.cols(); ++column) {
    const auto result{std::sqrt(covariance.rows() + lambda) * covariance_sqrt.col(column)};
    const auto result_state{StateType::from_eigen_vector(result)};

    sigma_points.push_back(state + result_state);
    sigma_points.push_back(state - result_state);
  }
  // for (const auto & column : covariance_sqrt.colwise()) {
  //   const auto result{std::sqrt(covariance.rows() + lambda) * column};
  //   const auto result_state{StateType::from_eigen_vector(result)};

  //   sigma_points.push_back(state + result_state);
  //   sigma_points.push_back(state - result_state);
  // }

  return sigma_points;
}

/**
 * @brief This function takes a vector and stacks it vertically into a matrix with the specified number of rows.
 * @param[in] vector The input vector to be stacked into the matrix
 * @param[in] num_rows The number of rows in the output matrix
 * @return The stacked matrix with the input vector repeated for each row
 */
inline auto stack_vector_into_matrix(Eigen::VectorXf vector, int num_rows) -> Eigen::MatrixXf
{
  int num_cols = vector.size();
  Eigen::MatrixXf result(num_rows, num_cols);
  for (int i = 0; i < num_rows; i++) {
    result.row(i) = vector;
  }
  return result;
}

/**
 * @brief This function takes a std::vector of floats and returns an Eigen::VectorXf.
 * @param[in] input The input std::vector of floats to be converted into an Eigen::VectorXf.
 * @return The resulting Eigen::VectorXf that has the same values as the input std::vector.
 */
inline auto vector_to_vector_xf(const std::vector<float> & input) -> Eigen::VectorXf
{
  Eigen::VectorXf output(input.size());
  for (int i = 0; i < input.size(); i++) {
    output(i) = input[i];
  }
  return output;
}

/**
 * @brief This function takes a mean and its vector of sigma points and returns them as a matrix.
 * @tparam StateType A class representing the mean type.
 * @param[in] mean The current mean.
 * @param[in] sigma_points The vector of sigma points to be converted into a matrix.
 * @return A matrix containing the state and sigma points as rows.
 */
template <typename StateType>
inline auto mean_and_sigma_points_to_matrix_xf(
  const StateType & mean, const std::vector<StateType> & sigma_points) -> Eigen::MatrixXf
{
  Eigen::MatrixXf matrix(sigma_points.size() + 1, StateType::kNumVars);
  matrix.row(0) = StateType::to_eigen_vector(mean).transpose();
  for (std::size_t i = 0; i < sigma_points.size(); ++i) {
    matrix.row(i + 1) = StateType::to_eigen_vector(sigma_points[i]).transpose();
  }
  return matrix;
}

/**
 * @brief Performs the unscented transform on sigma points with special handling for angular states.
 *
 * This function computes the weighted mean and covariance of a set of sigma points, with specific
 * considerations for angular quantities (like yaw at index 3). The process follows these steps:
 *
 * 1. Computes the weighted mean for non-angular states using standard weighted average
 * 2. Handles angular states separately using circular statistics to compute a proper circular mean
 * 3. Computes the covariance matrix with special handling for angular differences:
 *    - For non-angular states, standard differences are used
 *    - For angular states, angle_difference() is used to find the shortest arc between angles
 *
 * @param[in] sigma_points Matrix of sigma points where each row is a sigma point and each column
 *            is a state dimension
 * @param[in] Wm Vector of weights used to compute the weighted mean of the sigma points
 * @param[in] Wc Vector of weights used to compute the weighted covariance of the sigma points
 *
 * @return A tuple containing the weighted mean vector and weighted covariance matrix of the sigma
 *         points
 *
 * @note The function assumes that angular quantities (e.g., yaw) are at index 3 of the state vector
 *       Angular values require special handling due to their circular nature, and this function
 *       implements circular statistics to properly compute means and covariances for such values.
 */
inline auto compute_unscented_transform(
  const Eigen::MatrixXf & sigma_points, const Eigen::VectorXf & Wm, const Eigen::VectorXf & Wc)
  -> std::tuple<Eigen::VectorXf, Eigen::MatrixXf>
{
  // Define which indices are angular quantities (assuming index 3 is yaw)
  const std::vector<int> angle_indices = {3};

  // Get dimensions
  const auto n_sigma_points{sigma_points.rows()};
  const auto n_dim{sigma_points.cols()};

  // Preallocate mean vector
  Eigen::VectorXf mean = Eigen::VectorXf::Zero(n_dim);

  // First compute the mean for non-angular states using weighted average
  for (auto i = 0; i < n_dim; ++i) {
    // Skip angular states, handle them separately
    if (std::find(angle_indices.begin(), angle_indices.end(), i) != angle_indices.end()) {
      continue;
    }

    // Standard weighted mean for non-angular states
    for (auto j = 0; j < n_sigma_points; ++j) {
      mean(i) += Wm(j) * sigma_points(j, i);
    }
  }

  // Now handle angular states using circular statistics
  for (auto idx : angle_indices) {
    // Extract angles from all sigma points
    std::vector<float> angles;
    std::vector<float> weights;

    for (auto j = 0; j < n_sigma_points; ++j) {
      angles.push_back(sigma_points(j, idx));
      weights.push_back(Wm(j));
    }

    // Compute circular mean
  mean(idx) = utils::weighted_circular_mean(angles, weights);
  }

  // Compute covariance, handling angular differences properly
  Eigen::MatrixXf covariance = Eigen::MatrixXf::Zero(n_dim, n_dim);

  for (auto i = 0; i < n_sigma_points; ++i) {
    // Create residual vector
    Eigen::VectorXf residual = Eigen::VectorXf::Zero(n_dim);

    for (auto j = 0; j < n_dim; ++j) {
      if (std::find(angle_indices.begin(), angle_indices.end(), j) != angle_indices.end()) {
        // For angular states, use angle_difference
        residual(j) = utils::angle_difference(mean(j), sigma_points(i, j));
      } else {
        // For non-angular states, use standard difference
        residual(j) = sigma_points(i, j) - mean(j);
      }
    }

    // Update covariance
    covariance += Wc(i) * residual * residual.transpose();
  }

  return {mean, covariance};
}

/**
 * This function generates the sigma points and weights required for the Unscented Kalman Filter.
 * It computes the scaling factor lambda based on the provided alpha and kappa parameters.
 * The function then generates the sigma points using the given state and covariance, along with the computed lambda.
 * Additionally, it calculates the weights for mean and covariance calculations using the provided alpha, beta, and
 * lambda.
 *
 * @tparam StateType The type of the state vector.
 * @tparam CovarianceType The type of the covariance matrix.
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
inline auto generate_sigma_points_and_weights(
  const StateType & state, const CovarianceType & covariance, const float alpha, const float beta,
  const float kappa) -> std::tuple<std::vector<StateType>, Eigen::VectorXf, Eigen::VectorXf>
{
  const auto lambda{generate_lambda(state.kNumVars, alpha, kappa)};
  const auto sigma_points{generate_sigma_points(state, covariance, lambda)};
  const auto [Wm, Wc] = generate_weights(state.kNumVars, alpha, beta, lambda);
  return {sigma_points, Wm, Wc};
}

}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_UNSCENTED_TRANSFORM_HPP
