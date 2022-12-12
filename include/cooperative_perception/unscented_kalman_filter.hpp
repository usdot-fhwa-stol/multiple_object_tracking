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

#ifndef UNSCENTED_KALMAN_FILTER_HPP
#define UNSCENTED_KALMAN_FILTER_HPP

// Acquire sigma points generation method
#include "cooperative_perception/unscented_transform.hpp"

/**
 * This library breaks apart the UKF structure into two functions:
 * 1) prediction step
 * 2) update step
 * This decomposition allows the use of the prediction step in isolation, which
 * is useful for temporal alignment.
 */

#endif  // UNSCENTED_KALMAN_FILTER_HPP
