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

#ifndef MULTIPLE_OBJECT_TRACKING_COVARIANCE_CALIBRATION_HPP
#define MULTIPLE_OBJECT_TRACKING_COVARIANCE_CALIBRATION_HPP

namespace multiple_object_tracking
{
/**
 * @brief Covariance Calibration
 *
 * @tparam DetectionType for whose covariance needs to be calibrated
 */
template <typename DetectionType>
auto calibrate_covariance(DetectionType & detection) -> void
{
  // TODO: Implement covariance calibration algorithm here
  // Covariance calibration magic here
}
}  // namespace multiple_object_tracking

#endif  // MULTIPLE_OBJECT_TRACKING_UNSCENTED_TRANSFORM_HPP
