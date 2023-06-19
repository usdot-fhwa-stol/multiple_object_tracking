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

#ifndef COOPERATIVE_PERCEPTION_COVARIANCE_CALIBRATION_HPP
#define COOPERATIVE_PERCEPTION_COVARIANCE_CALIBRATION_HPP

#include "cooperative_perception/detection.hpp"

namespace cooperative_perception
{
/**
 * @brief Covariance Calibration
 *
 * @tparam DetectionType for whose covariance needs to be calibrated
 */
template <typename DetectionType>
auto calibrateCovariance(DetectionType object) -> void
{
  // TODO: Implement covariance calibration algorithm here
  // Covariance calibration magic here
}
}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_UNSCENTED_TRANSFORM_HPP
