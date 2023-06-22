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
#include "cooperative_perception/scoring.hpp"
#include "cooperative_perception/detection.hpp"
#include <map>
#include <vector>
#include <string>
#include <dlib/optimization/max_cost_assignment.h>
#include <dlib/matrix.h>

namespace cooperative_perception
{
using AssociationMap = std::map<std::string, std::vector<std::string>>;

auto scoreMatrixFromScoreMap(const ScoreMap& scores, std::set<std::string>& trackSet,
                             std::set<std::string>& detectionSet) -> dlib::matrix<float>
{
  std::vector<float> values;

  // Extract values and track/detection uuids from the ScoreMap
  for (const auto& pair : scores)
  {
    values.push_back(pair.second);
    trackSet.insert(pair.first.first);
    detectionSet.insert(pair.first.second);
  }

  // Determine the number of tracks and detections
  const long numTracks = trackSet.size();
  const long numDetections = detectionSet.size();

  // Create the dlib matrix with the appropriate size and initialize all values to zero
  dlib::matrix<float> output(numTracks, numDetections);
  output = 0.0;

  // Assign the values to the matrix
  for (long r = 0; r < output.nr(); ++r)
  {
    for (long c = 0; c < output.nc(); ++c)
    {
      const auto& track_uuid = *std::next(trackSet.begin(), r);
      const auto& detection_uuid = *std::next(detectionSet.begin(), c);
      const auto& it = scores.find({ track_uuid, detection_uuid });
      if (it != scores.end())
      {
        const auto& score = it->second;
        output(r, c) = score;
      }
    }
  }
  return output;
}

auto costMatrixFromScoreMatrix(const dlib::matrix<float>& scoreMatrix) -> dlib::matrix<int>
{
  // Determine the dimensions of the score matrix
  const long numTracks = scoreMatrix.nr();
  const long numDetections = scoreMatrix.nc();

  // Find the maximum value in the score matrix
  const float maxScore = dlib::max(scoreMatrix);

  // Create the cost matrix with the same dimensions
  dlib::matrix<int> costMatrix(numTracks, numDetections);

  // Normalize the score matrix, subtract from 1, and convert to integer
  for (long r = 0; r < numTracks; ++r)
  {
    for (long c = 0; c < numDetections; ++c)
    {
      const float normalizedScore = scoreMatrix(r, c) / maxScore;
      const float costValue = 1.0 - normalizedScore;
      // Scale the value by multiplying by 100 and convert to int
      const int scaledCost = static_cast<int>(costValue * 100);
      costMatrix(r, c) = scaledCost;
    }
  }

  return costMatrix;
}

template <typename T>
const T& getElementAt(const std::set<T>& s, size_t index)
{
  if (index >= s.size())
  {
    throw std::out_of_range("Index out of range");
  }

  auto it = s.begin();
  std::advance(it, index);
  return *it;
}

auto associationMapFromScoreMap(const ScoreMap& scores, const std::vector<long>& assignments,
                                const std::set<std::string>& trackSet, const std::set<std::string>& detectionSet)
    -> AssociationMap
{
  // Create the AssociationMap
  AssociationMap associations;

  for (int i = 0; i < assignments.size(); i++)
  {
    const auto& track_uuid = getElementAt(trackSet, i);
    const auto& detection_uuid = getElementAt(detectionSet, assignments[i]);
    associations[track_uuid].push_back(detection_uuid);
  }

  return associations;
}

auto GnnAssociator(const ScoreMap& scores) -> AssociationMap
{
  std::set<std::string> trackSet;
  std::set<std::string> detectionSet;

  // Generate the score matrix
  const auto scoreMatrix = scoreMatrixFromScoreMap(scores, trackSet, detectionSet);

  // Generate the cost matrix
  const auto costMatrix = costMatrixFromScoreMatrix(scoreMatrix);

  // Perform max_cost_assignment
  const std::vector<long> assignments = max_cost_assignment(costMatrix);

  // Generate the association map
  const auto associations = associationMapFromScoreMap(scores, assignments, trackSet, detectionSet);

  return associations;
}

auto printAssociationMap(const AssociationMap& associations) -> void
{
  for (const auto& pair : associations)
  {
    const std::string& trackUUID = pair.first;
    const std::vector<std::string>& detectionUUIDs = pair.second;

    std::cout << "Track UUID: " << trackUUID << std::endl;
    std::cout << "Assigned Detection UUIDs: ";
    for (const std::string& detectionUUID : detectionUUIDs)
    {
      std::cout << detectionUUID << " ";
    }
    std::cout << std::endl << std::endl;
  }
}

// association_visitor

template <typename AssociationVisitor>
auto associate_objects_to_tracks(ScoreMap gated_scores, const AssociationVisitor& association_visitor) -> AssociationMap
{
  //   using namespace dlib;
  AssociationMap associations;

  for (const auto& [uuid_pair, score] : gated_scores)
  {
    associations[uuid_pair.first] = std::visit(association_visitor, uuid_pair.first, gated_scores);
  }

  return associations;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP
