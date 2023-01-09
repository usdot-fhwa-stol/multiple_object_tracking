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

#ifndef COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
#define COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP

#include <variant>
#include <units.h>
#include "cooperative_perception/detected_object.hpp"
#include "cooperative_perception/utils.hpp"

namespace cooperative_perception
{
/**
 * @brief State propagation visitor
 *
 * When called, this visitor will propagate the visited object's state vector and update its timestamp.
 */
constexpr utils::Visitor kStatePropagator{ [](auto& object, units::time::second_t time) {
  object.state = nextState(object.state, time - object.timestamp);
  object.timestamp = time;
} };

/**
 * @brief Get predicted DetectedObject for specified time
 *
 * @param object DetectedObjectType being predicted
 * @param time Prediction time
 * @return DetectedObjectType whose state corresponds to the specified time
 */
auto objectAtTime(const DetectedObjectType& object, units::time::second_t time) -> DetectedObjectType
{
  DetectedObjectType new_object{ object };

  std::visit(kStatePropagator, new_object, std::variant<units::time::second_t>(time));

  return new_object;
};

/**
 * @brief Get predicted DetectedObjects for specified time
 *
 * @param objects List of DetectedObjectTypes being predicted
 * @param time Prediction time
 * @return List of DetectedObjectTypes, each of whose state corresponds to the specified time
 */
auto objectsAtTime(const DetectedObjectList& objects, units::time::second_t time) -> DetectedObjectList
{
  DetectedObjectList new_objects{ objects };

  std::transform(std::cbegin(new_objects), std::cend(new_objects), std::begin(new_objects),
                 [time](const DetectedObjectType& object) { return objectAtTime(object, time); });

  return new_objects;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TEMPORAL_ALIGNMENT_HPP
