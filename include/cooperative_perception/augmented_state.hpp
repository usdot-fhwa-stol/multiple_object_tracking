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

#ifndef COOPERATIVE_PERCEPTION_AUGMENTED_STATE_HPP
#define COOPERATIVE_PERCEPTION_AUGMENTED_STATE_HPP

#include <cstddef>
#include <functional>

namespace cooperative_perception
{
/**
 * @brief Augmented state vector containing a motion model's state vector and its process noise vector
 *
 * @tparam State Motion model's state vector type
 * @tparam ProcessNoise Motion model's process noise vector type
 */
template <typename State, typename ProcessNoise>
struct AugmentedState
{
  State state;
  ProcessNoise process_noise;

  /**
   * @brief Number of elements in the augmented state vector
   */
  static constexpr auto kNumVars{ State::kNumVars + ProcessNoise::kNumVars };

  /**
   * @brief Convert an Eigen::Vector into an AugmentedState
   *
   * @param[in] vec Vector being converted
   * @return AugmentState instance
   */
  static auto fromEigenVector(const Eigen::Vector<float, kNumVars>& vec) noexcept -> AugmentedState
  {
    return AugmentedState<State, ProcessNoise>{
      .state{ State::fromEigenVector(Eigen::Vector<float, State::kNumVars>{ vec(Eigen::seqN(0, State::kNumVars)) }) },
      .process_noise{ ProcessNoise::fromEigenVector(
          Eigen::Vector<float, ProcessNoise::kNumVars>{ vec(Eigen::seqN(State::kNumVars, ProcessNoise::kNumVars)) }) }
    };
  }
};

/**
 * @brief Compare true equality between two AugmentedStates
 *
 * This function was added to support unordered containers. It should not be used in general computations for the same
 * reasons that floating point values cannot be equated exactly.
 *
 * @tparam State State vector type of AugmentedState being compared
 * @tparam ProcessNoise Process noise vector type of AugmentedState being compared
 *
 * @param[in] lhs Left-hand side (lhs) of the equality expression
 * @param[in] rhs Right-hand side (rhs) of the equality expression
 * @return True if AugmentedState are exactly equal, false otherwise
 */
template <typename State, typename ProcessNoise>
inline auto operator==(const AugmentedState<State, ProcessNoise>& lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> bool
{
  return lhs.state == rhs.state && lhs.process_noise == rhs.process_noise;
}

/**
 * @brief Add-assignment operator overload
 *
 * Adds two AugmentedState variables together and stores the result in the left-hand side operand.
 *
 * @tparam State State vector type of AugmentedState being added
 * @tparam ProcessNoise Process noise vector type of AugmentedState being added
 *
 * @param[in] lhs Left-hand side (lhs) of the add-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the add-assignment expression
 * @return Modified left-hand side operand
 */
template <typename State, typename ProcessNoise>
inline auto operator+=(AugmentedState<State, ProcessNoise>& lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> AugmentedState<State, ProcessNoise>&
{
  lhs.state += rhs.state;
  lhs.process_noise += rhs.process_noise;
  return lhs;
}

/**
 * @brief Addition operator overload
 *
 * Adds two AugmentedState variables together and returns the result in a new AugmentedState.
 *
 * @tparam State State vector type of AugmentedState being added
 * @tparam ProcessNoise Process noise vector type of AugmentedState being added
 *
 * @param[in] lhs Left-hand side (lhs) of the addition expression
 * @param[in] rhs Right-hand side (rhs) of the addition expression
 * @return Operation result
 */
template <typename State, typename ProcessNoise>
inline auto operator+(AugmentedState<State, ProcessNoise> lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> AugmentedState<State, ProcessNoise>
{
  lhs += rhs;
  return lhs;
}

/**
 * @brief Subtract-assignment operator overload
 *
 * Subtracts two AugmentedState variables together and stores the result in the left-hand side operand.
 *
 * @tparam State State vector type of AugmentedState being subtracted
 * @tparam ProcessNoise Process noise vector type of AugmentedState being subtracted
 *
 * @param[in] lhs Left-hand side (lhs) of the subtract-assignment expression
 * @param[in] rhs Right-hand side (rhs) of the subtract-assignment expression
 * @return Modified left-hand side operand
 */
template <typename State, typename ProcessNoise>
inline auto operator-=(AugmentedState<State, ProcessNoise>& lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> AugmentedState<State, ProcessNoise>&
{
  lhs.state -= rhs.state;
  lhs.process_noise -= rhs.process_noise;
  return lhs;
}

/**
 * @brief Subtraction operator overload
 *
 * Subtracts two AugmentedState variables together and returns the result in a new AugmentedState.
 *
 * @tparam State State vector type of AugmentedState being subtracted
 * @tparam ProcessNoise Process noise vector type of AugmentedState being subtracted
 *
 * @param[in] lhs Left-hand side (lhs) of the subtraction expression
 * @param[in] rhs Right-hand side (rhs) of the subtraction expression
 * @return Operation result
 */
template <typename State, typename ProcessNoise>
inline auto operator-(AugmentedState<State, ProcessNoise> lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> AugmentedState<State, ProcessNoise>
{
  lhs -= rhs;
  return lhs;
}

namespace utils
{
/**
 * @brief Compares the almost-equality of two AugmentedStates
 *
 * @tparam State State vector type of AugmentedState being almost-equated
 * @tparam ProcessNoise Process noise vector type of AugmentedState being almost-equated
 *
 * @param[in] lhs Left-hand side (lhs) of the almost-equal expression
 * @param[in] rhs Right-hand side (rhs) of the almost-equal expression
 * @return True if AugmentedState are almost-equal, false otherwise
 */
template <typename State, typename ProcessNoise>
inline auto almostEqual(const AugmentedState<State, ProcessNoise>& lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> bool
{
  return almostEqual(lhs.state, rhs.state) && almostEqual(lhs.process_noise, rhs.process_noise);
}

/**
 * @brief Rounds AugmentedState vector elements to the nearest decimal place
 *
 * @tparam State State vector type of AugmentedState being rounded
 * @tparam ProcessNoise Process noise vector type of AugmentedState being rounded
 *
 * @param[in] state AugmentedState being rounded
 * @param[in] decimal_place Number of decimal placed to round. For example, 3 means round to nearest thousandths (0.001)
 * @return Rounded AugmentedState
 */
template <typename State, typename ProcessNoise>
inline auto roundToDecimalPlace(const AugmentedState<State, ProcessNoise>& state, std::size_t decimal_place)
    -> AugmentedState<State, ProcessNoise>
{
  return AugmentedState<State, ProcessNoise>{ roundToDecimalPlace(state.state, decimal_place),
                                              roundToDecimalPlace(state.process_noise, decimal_place) };
}
}  // namespace utils

}  // namespace cooperative_perception

namespace std
{
/**
 * @brief std::hash specialization for AugmentedState
 *
 * This specialization is necessary to use AugmentedState in unordered containers (e.g., std::unordered_set)
 *
 * @tparam State State vector type of AugmentedState being hashed
 * @tparam ProcessNoise Process noise vector type of AugmentedState being hashed
 */
template <typename State, typename ProcessNoise>
struct hash<cooperative_perception::AugmentedState<State, ProcessNoise>>
{
  /**
   * @brief Call operator overload
   *
   * Computes the has of AugmentedState when called
   *
   * @param[in] state AugmentedState being hashed
   * @return hash corresponding to the AugmentedState
   */
  std::size_t operator()(const cooperative_perception::AugmentedState<State, ProcessNoise>& state) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, std::hash<State>{}(state.state));
    boost::hash_combine(seed, std::hash<ProcessNoise>{}(state.process_noise));

    return seed;
  }
};
}  // namespace std

#endif  // COOPERATIVE_PERCEPTION_AUGMENTED_STATE_HPP
