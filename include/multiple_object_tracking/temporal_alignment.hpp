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
 * Originally developed for Leidos by the Human and Intelligent Vehicle
 * Ensembles (HIVE) Lab at Virginia Commonwealth University (VCU).
 */

#ifndef MULTIPLE_OBJECT_TRACKING_TEMPORAL_ALIGNMENT_HPP
#define MULTIPLE_OBJECT_TRACKING_TEMPORAL_ALIGNMENT_HPP

#include <units.h>

#include <variant>

#include "multiple_object_tracking/unscented_kalman_filter.hpp"

namespace multiple_object_tracking
{
/**
 * @brief Function object to predict a system's state and covariance using an unscented transform
 */
struct UnscentedTransform
{
  float alpha; /** A tuning parameter */
  float beta;  /** A tuning parameter */
  float kappa; /** A tuning parameter */

  /**
   * @brief Transform the state and state covariance using an unscented transform
   *
   * @tparam State Type of the state being transformed
   * @tparam Covariance Type of the state covariance being transformed
   *
   * @param[in,out] state State being transformed
   * @param[in,out] covariance State covariance being transformed
   * @param[in] duration Transformation duration
   *
   * @return void
   */
  template <typename State, typename Covariance>
  auto operator()(State & state, Covariance & covariance, units::time::second_t duration) const
    -> void
  {
    const auto [s, c] =
      unscented_kalman_filter_predict(state, covariance, duration, alpha, kappa, beta);
    state = s;
    covariance = c;
  }
};

inline constexpr UnscentedTransform default_unscented_transform{1.0, 2.0, 1.0};

/**
 * @brief Propagate a Detection or Track to a specific time
 *
 * This overload operates on non-variant object types.
 *
 * @tparam Object The type of object being propagated
 * @tparam Propagator The type of the function object being used to propagate
 *
 * @param[in,out] object The object being propagated
 * @param[in] time The time that the object will be propagated to
 * @param[in] propagator The propagation function object that will do the propagation
 *
 * @return void
 */
template <typename Object, typename Propagator>
auto propagate_to_time(Object & object, units::time::second_t time, const Propagator & propagator)
  -> void
{
  propagator(object.state, object.covariance, time - object.timestamp);
  object.timestamp = time;
}

/**
 * @brief Propagate a Detection or Track to a specific time
 *
 * This overload operators on variant types
 *
 * @tparam Propagator The type of the function object being used to propagate
 * @tparam Alternatives Parameter pack containing the variant's alternative types
 *
 * @param[in,out] object The object being propagated
 * @param[in] time The time that the object will be propagated to
 * @param[in] propagator The propagation function object that will do the propagation
 *
 * @return void
 */
template <typename Propagator, typename... Alternatives>
auto propagate_to_time(
  std::variant<Alternatives...> & object, units::time::second_t time, const Propagator & propagator)
{
  std::visit(
    [&propagator](auto & o, units::time::second_t t) {
      multiple_object_tracking::propagate_to_time(o, t, propagator);
    },
    object, std::variant<units::time::second_t>{time});
}

/**
 * @brief Predict a Detection or Track to a specific time
 *
 * @tparam Object The type of object being predicted
 * @tparam Propagator The type of the function object being used to propagate the predicted object
 *
 * @param[in] object The object being predicted
 * @param[in] time The time that the object will be predicted to
 * @param[in] propagator The propagation function object that will do the propagation
 *
 * @return The predicted object
 */
template <typename Object, typename Propagator>
auto predict_to_time(Object object, units::time::second_t time, const Propagator & propagator)
{
  propagate_to_time(object, time, propagator);

  return object;
}

}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_TEMPORAL_ALIGNMENT_HPP
