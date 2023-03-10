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
#include "cooperative_perception/ctrv_model.hpp"

namespace cooperative_perception
{
// TODO: Create lambda function
auto generate_lambda(int n, double alpha, int kappa) -> float
{
  return pow(alpha, 2) * (n + kappa) - n;
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
// TODO: Add lambda as a parameter
// TODO: refactor to generate_sigma_points
template <typename State, typename StateCovariance>
auto sampleStateDistribution(const State& state, const StateCovariance& covariance) -> std::unordered_set<State>
{
  std::unordered_set<State> sigma_pts{};
  const auto lambda{ 3 - State::kNumVars };
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

template <typename State, typename StateCovariance>
auto unscentedTransform(const State& state, const StateCovariance& covariance, units::time::second_t time_step)
    -> std::tuple<State, StateCovariance>
{
  // Compute the sigma points based on the state vector and state covariance
  const auto sigma_points{ cooperative_perception::sampleStateDistribution(state, covariance) };

  // TODO: Consider better variable names since sigma_points are being propagated with nextState
  // Propagate state - this is sigma point 0
  const auto pred_state_0{ nextState(state, time_step) };

  // Propagate each sigma point through the process dynamics
  std::unordered_set<State> pred_sigma_points{};
  for (const auto& state : sigma_points)
  {
    const auto pred_sigma_point{ nextState(state, time_step) };
    pred_sigma_points.insert(pred_sigma_point);
  }

  // Compute the constants and weights for performing mean/covariance computations
  const float lambda{ 3.0 - State::kNumVars };
  const float w_0{ lambda / (lambda + State::kNumVars) };
  const float w_i{ lambda / (2.0 * (lambda + State::kNumVars)) };

  /**
   * Compute the new mean and covariance using these sigma points.
   * The state and sigma point objects are converted to Eigen vector types
   * for matrix arithmetic.
   */
  // Predicted state calculation
  auto pred_state{ w_0 * pred_state_0 };
  for (const auto& pred_sigma_point : pred_sigma_points)
  {
    pred_state += w_i * pred_sigma_point;
  }

  // Covariance calculation
  const Eigen::Vector<float, State::kNumVars> pred_state_vector_0{ State::toEigenVector(pred_state_0) };
  const Eigen::Vector<float, State::kNumVars> pred_state_vector{ State::toEigenVector(pred_state) };
  CtrvStateCovariance pred_covar{ w_0 * (pred_state_vector_0 - pred_state_vector) *
                                  (pred_state_vector_0 - pred_state_vector).transpose() };
  // for (const auto& pred_sigma_point : pred_sigma_points)
  // {
  //   auto state_diff = pred_sigma_point - pred_state;
  //   // If needed, fix yaw angle as it ranges 0 -> 2PI
  //   if (units::math::fmod(state_diff.yaw.get_angle(), units::angle::radian_t{ 2 * M_PI }) >
  //       units::angle::radian_t{ M_PI })
  //   {
  //     state_diff.yaw -= units::angle::radian_t{ 2 * M_PI };
  //   }
  //   const Eigen::Vector<float, State::kNumVars> state_diff_vector{ State::toEigenVector(state_diff) };
  //   pred_covar += w_i * state_diff_vector * state_diff_vector.transpose();
  // }

  return { pred_state, pred_covar };
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP
