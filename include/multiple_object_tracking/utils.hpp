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

#ifndef MULTIPLE_OBJECT_TRACKING_UTILS_HPP
#define MULTIPLE_OBJECT_TRACKING_UTILS_HPP

#include <Eigen/Dense>
#include <cmath>
#include <variant>
#include <vector>

#include "multiple_object_tracking/ctra_model.hpp"
#include "multiple_object_tracking/ctrv_model.hpp"
#include "multiple_object_tracking/visitor.hpp"

namespace multiple_object_tracking::utils
{
/**
 * @brief Rounds a float to the nearest decimal place. Useful for comparing covariance values
 *
 * @param[in] n float being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths
 * (0.001)
 * @return Rounded float
 */
inline auto round_to_decimal_place(float n, std::size_t decimal_place) -> float
{
  const auto multiplier{std::pow(10.0f, decimal_place)};

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
inline auto round_to_decimal_place(const Eigen::MatrixXf & n, std::size_t decimal_place)
  -> Eigen::MatrixXf
{
  Eigen::MatrixXf x(n.rows(), n.cols());
  for (int i = 0; i < n.rows(); ++i) {
    for (int j = 0; j < n.cols(); ++j) {
      x(i, j) = round_to_decimal_place(n(i, j), decimal_place);
    }
  }
  return x;
}

/**
 * @brief Visitor for printing entity information.
 *
 * This visitor prints the timestamp, UUID, state, and covariance of the input entity to the standard output.
 *
 * @param[in] entity The entity object to be printed.
 */
constexpr Visitor print_entity_visitor{[](const auto & entity) {
  std::cout << "Timestamp: " << entity.timestamp << "\n";
  std::cout << "UUID: " << entity.uuid << "\n";
  print_state(entity.state);
  std::cout << "\nCovariance:\n" << entity.covariance;
  std::cout << "\n------------------------------------------------------------\n\n";
}};

/**
 * @brief Print the container of entities.
 *
 * This function takes a container of entities and prints each entity's information using the print_entity_visitor.
 *
 * @param[in] entities The container of entities to be printed.
 */
template <typename EntityContainer>
inline auto print_container(const EntityContainer & entities) -> void
{
  for (const auto & entity : entities) {
    std::visit(print_entity_visitor, entity);
  }
}

/**
 * @brief Normalize an angle to the range [-π, π)
 *
 * This function takes an angle in radians and normalizes it to the
 * range [-π, π) by adding or subtracting 2π as needed.
 *
 * @param[in] angle The angle to normalize (in radians)
 * @return The normalized angle in the range [-π, π)
 */
 inline float normalize_angle(float angle) {
  constexpr float TWO_PI = 2.0f * 3.14159265359f;
  // First, use fmod to get angle in range (-2π, 2π)
  float normalized = std::fmod(angle, TWO_PI);

  // Then shift to range [-π, π)
  if (normalized >= 3.14159265359f) {
      normalized -= TWO_PI;
  } else if (normalized < -3.14159265359f) {
      normalized += TWO_PI;
  }

  return normalized;
}

/**
* @brief Compute the shortest angular difference between two angles
*
* This function computes the shortest angular difference between two angles,
* accounting for the circular nature of angles.
*
* @param[in] from The starting angle (in radians)
* @param[in] to The target angle (in radians)
* @return The shortest angular difference in the range [-π, π)
*/
inline float angle_difference(float from, float to) {
  float diff = normalize_angle(to - from);
  return diff;
}

/**
* @brief Normalize angles in an Eigen vector
*
* This function normalizes angles at specified indices in an Eigen vector.
*
* @param[in,out] vector The vector containing angles to normalize
* @param[in] angle_indices Vector of indices where angles are located
* @return The normalized vector
*/
inline Eigen::VectorXf normalize_angles_in_vector(const Eigen::VectorXf& vector,
  const std::vector<int>& angle_indices) {
Eigen::VectorXf normalized_vector = vector;
for (auto idx : angle_indices) {
normalized_vector[idx] = normalize_angle(normalized_vector[idx]);
}
return normalized_vector;
}

/**
* @brief Normalize angles in a matrix of vectors (e.g., sigma points)
*
* This function normalizes angles at specified indices in each row of a matrix.
*
* @param[in,out] matrix The matrix containing vectors with angles to normalize
* @param[in] angle_indices Vector of indices where angles are located in each row
*/
inline void normalize_angles_in_matrix(Eigen::MatrixXf& matrix,
                                    const std::vector<int>& angle_indices) {
  for (int row = 0; row < matrix.rows(); ++row) {
      for (auto idx : angle_indices) {
          matrix(row, idx) = normalize_angle(matrix(row, idx));
      }
  }
}

}  // namespace multiple_object_tracking::utils

#endif  // MULTIPLE_OBJECT_TRACKING_UTILS_HPP
