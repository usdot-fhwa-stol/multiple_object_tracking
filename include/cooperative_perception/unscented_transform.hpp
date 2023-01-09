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
#include "cooperative_perception/ctrv_model.hpp"

namespace cooperative_perception
{
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
auto sampleStateDistribution(const State& state, const StateCovariance covariance, std::size_t num_points, float lambda)
    -> std::unordered_set<State>
{
  std::unordered_set<State> sigma_pts{ state };
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

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP
