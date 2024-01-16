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

#ifndef MULTIPLE_OBJECT_TRACKING_UNSCENTED_KALMAN_FILTER_HPP
#define MULTIPLE_OBJECT_TRACKING_UNSCENTED_KALMAN_FILTER_HPP

#include <units.h>

#include <tuple>
#include <vector>

#include "multiple_object_tracking/unscented_transform.hpp"

namespace multiple_object_tracking
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
 * prediction parameters (alpha, beta, kappa) are provided to generate the sigma points and weights. The get_next_state()
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
inline auto unscented_kalman_filter_predict(
  const StateType & state, const CovarianceType & covariance, const units::time::second_t time_step,
  const float alpha, const float beta, const float kappa) -> std::tuple<StateType, CovarianceType>
{
  // Generate sigma points and weights
  const auto [sigma_points, Wm, Wc] =
    generate_sigma_points_and_weights(state, covariance, alpha, beta, kappa);

  // Advance mean and sigma points through the non-linear model
  const auto predicted_mean{get_next_state(state, time_step)};
  std::vector<StateType> predicted_sigma_points;
  for (const auto & sigma_point : sigma_points) {
    predicted_sigma_points.push_back(get_next_state(sigma_point, time_step));
  }

  std::cout << "mean weights:\n" << Wm << '\n';
  std::cout << "\ncovariance weights:\n" << Wc << '\n';

  // Convert mean and sigma points into Eigen::MatrixXf
  const auto m_sigma_points{
    mean_and_sigma_pints_to_matrix_xf(predicted_mean, predicted_sigma_points)};

  // Compute UT based on the sigma points and weights
  // const auto [result_state_vector, result_covariance_matrix] =
  //   compute_unscented_transform(m_sigma_points, Wm, Wc);

  const auto [result_state_vector, result_covariance_matrix] =
  compute_unscented_transform2(predicted_mean, predicted_sigma_points, Wm, Wc);

  // std::cout << '\n' << result_state_vector << '\n';
  // std::cout << '\n' << result_covariance_matrix << '\n';

  // std::cout << '\n' << result_state_vector2 << '\n';
  // std::cout << '\n' << result_covariance_matrix2 << '\n';

  const auto result_state{StateType::from_eigen_vector(std::move(result_state_vector))};
  const CovarianceType result_covariance{std::move(result_covariance_matrix)};
  return {std::move(result_state), std::move(result_covariance)};
}

/**
 * @brief Visitor for performing prediction using Unscented Kalman Filter (UKF).
 *
 * The `UkfPredictionVisitor` class is a callable visitor that performs prediction on an object using the Unscented
 * Kalman Filter. It applies the prediction algorithm to update the state and covariance of the object based on the
 * provided parameters.
 */
class UkfPredictionVisitor
{
public:
  /**
   * @brief Constructs a `UkfPredictionVisitor` with the specified parameters.
   *
   * @param[in] alpha The scaling parameter for sigma points.
   * @param[in] beta The secondary scaling parameter for sigma points.
   * @param[in] kappa A tuning parameter affecting how the points are sampled.
   */
  explicit UkfPredictionVisitor(float alpha, float beta, float kappa)
  : alpha_(alpha), beta_(beta), kappa_(kappa)
  {
  }

  /**
   * @brief Performs prediction on the specified object using the provided time stamp.
   *
   * This function applies the Unscented Kalman Filter prediction algorithm to update the state and covariance of the
   * given object based on the time difference between the object's timestamp and the provided time stamp. The
   * prediction results are stored in the object itself.
   *
   * @tparam ObjectType The type of the object being predicted.
   * @param[in,out] object The object to be predicted.
   * @param[in] time The time stamp for prediction.
   */
  template <typename ObjectType>
  auto operator()(ObjectType & object, units::time::second_t time) const -> void
  {
    const auto [state, covariance] = unscented_kalman_filter_predict(
      object.state, object.covariance, time - object.timestamp, alpha_, kappa_, beta_);
    object.state = state;
    object.covariance = covariance;
    object.timestamp = time;
  }

private:
  float alpha_;
  float beta_;
  float kappa_;
};

}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_UNSCENTED_KALMAN_FILTER_HPP
