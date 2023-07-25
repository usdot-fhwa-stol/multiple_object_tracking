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
#include <variant>
#include <Eigen/Dense>
#include "cooperative_perception/visitor.hpp"
#include "cooperative_perception/ctra_model.hpp"
#include "cooperative_perception/ctrv_model.hpp"

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

/**
 * @brief Visitor for checking almost equality of two entities.
 *
 * This visitor checks if two entities are almost equal by comparing their timestamps, uuids, states,
 * and covariances with a given precision (decimal_place).
 *
 * @param[in] lhs The left-hand side entity for comparison.
 * @param[in] rhs The right-hand side entity for comparison.
 * @param[in] decimal_place The precision (number of decimal places) used for comparisons.
 * @return True if entities are almost equal; false otherwise.
 */
constexpr Visitor almost_equal_visitor{ [](const auto& lhs, const auto& rhs, const auto& decimal_place) -> bool {
  if constexpr (std::is_same_v<decltype(lhs), decltype(rhs)>)
  {
    if (lhs.state.kNumVars != rhs.state.kNumVars)
    {
      return false;  // Not equal if the state vector have different sizes
    }
    if (lhs.timestamp != rhs.timestamp || lhs.uuid != rhs.uuid)
    {
      return false;  // Not equal if objects have different timestamp or uuid
    }
    if (!almostEqual(roundToDecimalPlace(lhs.state, decimal_place), roundToDecimalPlace(rhs.state, decimal_place)))
    {
      return false;  // Not equal if states are not equal after rounding up
    }

    if (!almostEqual(roundToDecimalPlace(lhs.covariance, decimal_place),
                     roundToDecimalPlace(rhs.covariance, decimal_place)))
    {
      return false;  // Not equal if covariances are not equal after rounding up
    }
    return true;
  }
  else
  {
    return false;  // Not equal if both objects are not the same type
  }
} };

/**
 * @brief Compare two vectors of entities for almost equality.
 *
 * This function compares two vectors of entities for almost equality by visiting each entity and using the
 * almost_equal_visitor to check their similarity with the given precision (decimal_place).
 *
 * @param[in] lhs The left-hand side vector of entities for comparison.
 * @param[in] rhs The right-hand side vector of entities for comparison.
 * @param[in] decimal_place The precision (number of decimal places) used for comparisons.
 * @return True if the vectors are almost equal; false otherwise.
 */
template <typename EntityContainer>
inline auto almostEqual(const EntityContainer& lhs, const EntityContainer& rhs, std::size_t decimal_place = 5) -> bool
{
  if (lhs.size() != rhs.size())
  {
    return false;  // Different number of elements, container are not equal
  }

  for (std::size_t i = 0; i < lhs.size(); ++i)
  {
    if (!std::visit(almost_equal_visitor, lhs[i], rhs[i], std::variant<std::size_t>(decimal_place)))
    {
      return false;  // Elements differ, vectors are not equal
    }
  }
  return true;  // All elements are equal, vectors are equal
}

/**
 * @brief Visitor for printing entity information.
 *
 * This visitor prints the timestamp, UUID, state, and covariance of the input entity to the standard output.
 *
 * @param[in] entity The entity object to be printed.
 */
constexpr Visitor print_entity_visitor{ [](const auto& entity) {
  std::cout << "Timestamp: " << entity.timestamp << "\n";
  std::cout << "UUID: " << entity.uuid << "\n";
  printState(entity.state);
  std::cout << "\nCovariance:\n" << entity.covariance;
  std::cout << "\n------------------------------------------------------------\n\n";
} };

/**
 * @brief Print the container of entities.
 *
 * This function takes a container of entities and prints each entity's information using the print_entity_visitor.
 *
 * @param[in] entities The container of entities to be printed.
 */
template <typename EntityContainer>
inline auto printContainer(const EntityContainer& entities) -> void
{
  for (const auto& entity : entities)
  {
    std::visit(print_entity_visitor, entity);
  }
}

}  // namespace cooperative_perception::utils

#endif  // COOPERATIVE_PERCEPTION_UTILS_HPP
