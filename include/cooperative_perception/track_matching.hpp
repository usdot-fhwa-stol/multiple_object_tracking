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

auto scoreMatrixFromScoreMap(const ScoreMap& scores, std::set<std::string>& track_set,
                             std::set<std::string>& detection_set) -> dlib::matrix<float>
{
  std::vector<float> values;

  // Extract values and track/detection uuids from the ScoreMap
  for (const auto& pair : scores)
  {
    values.push_back(pair.second);
    track_set.insert(pair.first.first);
    detection_set.insert(pair.first.second);
  }

  // Determine the number of tracks and detections
  const long num_tracks = track_set.size();
  const long num_detections = detection_set.size();

  // Create the dlib matrix with the appropriate size and initialize all values to zero
  dlib::matrix<float> score_matrix(num_tracks, num_detections);
  score_matrix = std::numeric_limits<float>::max();

  // Assign the values to the matrix
  for (long r = 0; r < score_matrix.nr(); ++r)
  {
    for (long c = 0; c < score_matrix.nc(); ++c)
    {
      const auto& track_uuid = *std::next(track_set.begin(), r);
      const auto& detection_uuid = *std::next(detection_set.begin(), c);
      const auto& it = scores.find({ track_uuid, detection_uuid });
      if (it != scores.end())
      {
        const auto& score = it->second;
        score_matrix(r, c) = score;
      }
    }
  }
  return score_matrix;
}

auto costMatrixFromScoreMatrix(const dlib::matrix<float>& score_matrix) -> dlib::matrix<int>
{
  // Determine the dimensions of the score matrix
  const long num_tracks = score_matrix.nr();
  const long num_detections = score_matrix.nc();

  // Find the maximum value in the score matrix
  const float max_score = dlib::max(score_matrix);

  // Create the cost matrix with the same dimensions
  dlib::matrix<int> cost_matrix(num_tracks, num_detections);

  // Normalize the score matrix, subtract from 1, and convert to integer
  for (long r = 0; r < num_tracks; ++r)
  {
    for (long c = 0; c < num_detections; ++c)
    {
      const float normalized_score = score_matrix(r, c) / max_score;
      const float cost_value = 1.0 - normalized_score;
      // Scale the value by multiplying by 100 and convert to int
      const int scaled_cost = static_cast<int>(cost_value * 100);
      cost_matrix(r, c) = scaled_cost;
    }
  }

  return cost_matrix;
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
                                const std::set<std::string>& track_set, const std::set<std::string>& detection_set)
    -> AssociationMap
{
  // Create the AssociationMap
  AssociationMap associations;

  for (int i = 0; i < assignments.size(); i++)
  {
    const auto& track_uuid = getElementAt(track_set, i);
    const auto& detection_uuid = getElementAt(detection_set, assignments[i]);
    associations[track_uuid].push_back(detection_uuid);
  }

  return associations;
}

auto gnnAssociator(const ScoreMap& scores) -> AssociationMap
{
  std::set<std::string> track_set;
  std::set<std::string> detection_set;

  // Generate the score matrix
  const auto score_matrix = scoreMatrixFromScoreMap(scores, track_set, detection_set);

  // Generate the cost matrix
  const auto cost_matrix = costMatrixFromScoreMatrix(score_matrix);

  // Perform max_cost_assignment
  const std::vector<long> assignments = max_cost_assignment(cost_matrix);

  // Generate the association map
  const auto associations = associationMapFromScoreMap(scores, assignments, track_set, detection_set);

  return associations;
}

auto printAssociationMap(const AssociationMap& associations) -> void
{
  for (const auto& pair : associations)
  {
    const std::string& track_uuid = pair.first;
    const std::vector<std::string>& detection_uuids = pair.second;

    std::cout << "Track UUID: " << track_uuid << std::endl;
    std::cout << "Assigned Detection UUIDs: ";
    for (const std::string& detection_uuid : detection_uuids)
    {
      std::cout << detection_uuid << " ";
    }
    std::cout << std::endl << std::endl;
  }
}

// association_visitor
constexpr Visitor gnn_association_visitor{ [](const auto& scores) -> AssociationMap { return gnnAssociator(scores); } };

template <typename AssociationVisitor>
auto associateDetectionsToTracks(const ScoreMap gated_scores, const AssociationVisitor& association_visitor)
    -> AssociationMap
{
  AssociationMap associations = std::visit(association_visitor, std::variant<ScoreMap>(gated_scores));

  return associations;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP
