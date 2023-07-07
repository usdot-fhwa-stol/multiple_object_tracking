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

#ifndef COOPERATIVE_PERCEPTION_UNSCENTED_KALMAN_FILTER_HPP
#define COOPERATIVE_PERCEPTION_UNSCENTED_KALMAN_FILTER_HPP

#include "cooperative_perception/unscented_transform.hpp"

namespace cooperative_perception
{
/**
 * This library breaks apart the UKF structure into two functions:
 * 1) prediction step
 * 2) update step
 * This decomposition allows the use of the prediction step in isolation, which
 * is useful for temporal alignment.
 */

/**
 * This function performs the prediction step of the Unscented Kalman Filter by computing the unscented transform
 * for a given state and state covariance matrix. It generates sigma points and weights, and uses them to compute the
 * mean and covariance of the transformed sigma points through a non-linear model. The function advances the current
 * state of the system through the non-linear model using the specified time step. The Unscented Kalman Filter
 * prediction parameters (alpha, beta, kappa) are provided to generate the sigma points and weights. The nextState()
 * function is used to advance the state and sigma points through the non-linear model. The computed mean and sigma
 * points are converted to an Eigen::MatrixXf and used to compute the unscented transform, which returns the predicted
 * state and covariance.
 *
 * @param[in] state The initial state of the system.
 * @param[in] covariance The covariance matrix of the system.
 * @param[in] time_step The time step to advance the system forward.
 * @param[in] alpha The scaling parameter for sigma points in the unscented transform.
 * @param[in] beta The secondary scaling parameter for sigma points in the unscented transform.
 * @param[in] kappa The secondary scaling parameter for sigma points in the unscented transform.
 *
 * @return Tuple containing the resulting state and covariance matrix after the prediction step.
 */
template <typename StateType, typename CovarianceType>
inline auto unscentedKalmanFilterPredict(const StateType& state, const CovarianceType& covariance,
                                         const units::time::second_t time_step, const float alpha, const float beta,
                                         const float kappa) -> std::tuple<StateType, CovarianceType>
{
  // Generate sigma points and weights
  const auto [sigma_points, Wm, Wc] = generateSigmaPointsAndWeights(state, covariance, alpha, beta, kappa);

  // Advance mean and sigma points through the non-linear model
  const auto predicted_mean{ nextState(state, time_step) };
  std::unordered_set<StateType> predicted_sigma_points;
  for (const auto& sigma_point : sigma_points)
  {
    predicted_sigma_points.insert(nextState(sigma_point, time_step));
  }

  // Convert mean and sigma points into Eigen::MatrixXf
  const auto m_sigma_points{ sigmaSetToMatrixXf(predicted_mean, predicted_sigma_points) };

  // Compute UT based on the sigma points and weights
  const auto [result_state_vector, result_covariance_matrix] = computeUnscentedTransform(m_sigma_points, Wm, Wc);

  const auto result_state{ StateType::fromEigenVector(std::move(result_state_vector)) };
  const CovarianceType result_covariance{ std::move(result_covariance_matrix) };
  return { std::move(result_state), std::move(result_covariance) };
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_UNSCENTED_KALMAN_FILTER_HPP
