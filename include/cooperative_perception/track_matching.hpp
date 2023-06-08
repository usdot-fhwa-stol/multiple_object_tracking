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

#ifndef COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP
#define COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP

#include "cooperative_perception/track.hpp"
#include <cooperative_perception/scoring.hpp>
#include "cooperative_perception/detected_object.hpp"
#include <dlib/optimization/max_cost_assignment.h>

namespace cooperative_perception
{
void assign_objects_to_tracks(const std::vector<DetectedObject> objects, std::vector<Track> tracks)
{
  using namespace dlib;
  // Steps for assigning tracks to object

  // Step 1: Score each object versus each track using Mahalanobis distance
  std::vector<float> scores;

  for (int i = 0; i < tracks.size(); i++)
  {
    for (int j = 0; i < objects.size(); i++)
    {
      scores.push_back(mahalanobis_distance(object[j], tracks[i]));
    }
  }

  // Step 2: Using the scores, build up a cost matrix that the dlib library can interpret
  matrix<int> cost(tracks.size(), objects.size());

  for (int i = 0; i < scores.size(); i++)
  {
    cost(i) = scores[i];
  }

  // Step 3: Call the max_cost_assignment function and get the assignment row vector
  std::vector<long> assignment = max_cost_assignment(cost);

  // Step 4: Once each object is matched with each track, we add the object uuid to the track

  // added detected object uuid to matched tracked
}
}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP
