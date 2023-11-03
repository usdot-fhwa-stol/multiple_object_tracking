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

}  // namespace multiple_object_tracking::utils

#endif  // MULTIPLE_OBJECT_TRACKING_UTILS_HPP
