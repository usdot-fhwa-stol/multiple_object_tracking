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

auto vectorToMatrix(Eigen::VectorXd vector, int num_rows) -> Eigen::MatrixXd
{
  int num_cols = vector.size();
  Eigen::MatrixXd result(num_rows, num_cols);
  for (int i = 0; i < num_rows; i++)
  {
    result.row(i) = vector;
  }
  return result;
}

auto vectorToVectorXd(const std::vector<float>& input) -> Eigen::VectorXd
{
  Eigen::VectorXd output(input.size());
  for (int i = 0; i < input.size(); i++)
  {
    output(i) = input[i];
  }
  return output;
}

// template <typename State>
// auto sigmaSetToMatrixXd(const std::unordered_set<State>& sigma_points) -> Eigen::MatrixXd
// {
//   std::vector<Eigen::VectorXd> input_vec(sigma_points.begin(), sigma_points.end());
//   int rows = input_vec.size();
//   int cols = input_vec[0].size();

//   Eigen::MatrixXd output_matrix(rows, cols);
//   for (const auto& sigma_point : sigma_points)
//   {
//     // output_matrix.row(i) = State::toEigenVector(sigma_point);
//     std::cout << State::toEigenVector(sigma_point) << "\n";
//   }

// return output_matrix;
// }

auto unscentedTransform(const Eigen::MatrixXd& sigmas, const Eigen::VectorXd& Wm, const Eigen::VectorXd& Wc)
    -> std::tuple<Eigen::VectorXd, Eigen::MatrixXd>
{
  Eigen::VectorXd x = Wm.transpose() * sigmas;
  Eigen::MatrixXd x1 = vectorToMatrix(x, sigmas.rows());
  Eigen::MatrixXd y = sigmas - x1;
  Eigen::MatrixXd P = y.transpose() * (Wc.asDiagonal() * y);
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

  // Convert weights in Eigen::VectorXd
  const auto Wm{ vectorToVectorXd(vector_Wm) };
  const auto Wc{ vectorToVectorXd(vector_Wc) };

  // Convert sigma points into Eigen::MatrixXd
  // const auto m_sigma_points{ sigmaSetToMatrixXd(sigma_points) };

  // std::cout << "sigma points: \n" << m_sigma_points << "\n";

  std::cout << "Wm: \n" << Wm << "\n";
  std::cout << "Wc: \n" << Wc << "\n";

  // // Compute the sigma points based on the state vector and state covariance

  // const auto sigma_points{ cooperative_perception::generateSigmaPoints(state, covariance) };

  // const auto pred_state_0{ nextState(state, time_step) };

  // // Propagate each sigma point through the process dynamics
  // std::unordered_set<State> pred_sigma_points{};
  // for (const auto& state : sigma_points)
  // {
  //   const auto pred_sigma_point{ nextState(state, time_step) };
  //   pred_sigma_points.insert(pred_sigma_point);
  // }

  // // Compute the constants and weights for performing mean/covariance computations
  // const float lambda{ 3.0 - State::kNumVars };
  // const float w_0{ lambda / (lambda + State::kNumVars) };
  // const float w_i{ lambda / (2.0 * (lambda + State::kNumVars)) };

  // /**
  //  * Compute the new mean and covariance using these sigma points.
  //  * The state and sigma point objects are converted to Eigen vector types
  //  * for matrix arithmetic.
  //  */
  // // Predicted state calculation
  // auto pred_state{ w_0 * pred_state_0 };
  // for (const auto& pred_sigma_point : pred_sigma_points)
  // {
  //   pred_state += w_i * pred_sigma_point;
  // }

  // // Covariance calculation
  // const Eigen::Vector<float, State::kNumVars> pred_state_vector_0{ State::toEigenVector(pred_state_0) };
  // const Eigen::Vector<float, State::kNumVars> pred_state_vector{ State::toEigenVector(pred_state) };
  // CtrvStateCovariance pred_covar{ w_0 * (pred_state_vector_0 - pred_state_vector) *
  //                                 (pred_state_vector_0 - pred_state_vector).transpose() };
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

  using namespace units::literals;
  const CtrvState pred_state{ 5.7441_m, 1.3800_m, 2.2049_mps, Angle(0.5015_rad), 0.3528_rad_per_s };
  const CtrvStateCovariance pred_covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                             { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                             { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                             { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                             { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };

  return { pred_state, pred_covariance };
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
