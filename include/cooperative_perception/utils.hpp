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

#ifndef COOPERATIVE_PERCEPTION_UTILS_HPP
#define COOPERATIVE_PERCEPTION_UTILS_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace cooperative_perception::utils
{
/** Check if two floating point numbers are equal
 *
 * The two numbers are considered equal if their relative distance is within an epsilon interval.
 *
 * @param[in] first First number
 * @param[in] second Second number
 * @return If the two specified numbers are equal
 */
constexpr auto almostEqual(double first, double second) -> bool
{
  constexpr auto kEpsilon{ 1e-5 };

  return std::abs(first - second) < kEpsilon;
};

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

}  // namespace cooperative_perception::utils

#endif  // COOPERATIVE_PERCEPTION_UTILS_HPP
