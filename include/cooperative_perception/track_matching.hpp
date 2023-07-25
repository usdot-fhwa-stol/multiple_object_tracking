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
#include <set>
#include <vector>
#include <string>
#include <dlib/optimization/max_cost_assignment.h>
#include <dlib/matrix.h>

namespace cooperative_perception
{
/**
 * @brief Definition of AssociationMap, which is a mapping of track UUIDs to vectors of detection UUIDs
 */
using AssociationMap = std::map<std::string, std::vector<std::string>>;

/**
 * @brief Convert a ScoreMap to a score matrix.
 *
 * @param scores ScoreMap containing track-detection scores.
 * @param track_set Set to store track UUIDs.
 * @param detection_set Set to store detection UUIDs.
 * @return Score matrix representing the track-detection scores.
 */
inline auto scoreMatrixFromScoreMap(const ScoreMap& scores, std::set<std::string>& track_set,
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

  // Create the dlib matrix with the appropriate size and initialize all values to maximum float value
  dlib::matrix<float> score_matrix(num_tracks, num_detections);
  score_matrix = std::numeric_limits<float>::max();

  // Assign the values to the matrix
  for (long r = 0; r < score_matrix.nr(); ++r)
  {
    for (long c = 0; c < score_matrix.nc(); ++c)
    {
      // Get the track and detection UUIDs at the current matrix indices
      const auto& track_uuid = *std::next(track_set.begin(), r);
      const auto& detection_uuid = *std::next(detection_set.begin(), c);

      // Find the corresponding score in the ScoreMap
      const auto& it = scores.find({ track_uuid, detection_uuid });
      if (it != scores.end())
      {
        const auto& score = it->second;
        score_matrix(r, c) = score;
      }
    }
  }

  // Return the score matrix
  return score_matrix;
}

/**
 * @brief Convert a score matrix to a cost matrix.
 *
 * @param score_matrix Score matrix containing track-detection scores.
 * @return Cost matrix representing the costs between tracks and detections.
 */
inline auto costMatrixFromScoreMatrix(const dlib::matrix<float>& score_matrix) -> dlib::matrix<int>
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
      // Normalize the score by dividing it by the maximum score
      const float normalized_score = score_matrix(r, c) / max_score;

      // Calculate the cost value by subtracting the normalized score from 1
      const float cost_value = 1.0 - normalized_score;

      // Scale the cost value by multiplying it by 100 and convert it to an integer
      const int scaled_cost = static_cast<int>(cost_value * 100);

      // Assign the scaled cost to the cost matrix
      cost_matrix(r, c) = scaled_cost;
    }
  }

  // Return the cost matrix
  return cost_matrix;
}

/**
 * @brief Get the element at the specified index from a set.
 *
 * @param s The set from which to retrieve the element.
 * @param index The index of the element to retrieve.
 * @return The element at the specified index.
 * @throw std::out_of_range If the index is out of range.
 */
template <typename T>
inline auto getElementAt(const std::set<T>& s, size_t index) -> const T&
{
  // Check if the index is valid
  if (index >= s.size())
  {
    throw std::out_of_range("Index out of range");
  }

  // Create an iterator pointing to the element at the specified index
  auto it = s.begin();
  std::advance(it, index);

  // Return a reference to the element
  return *it;
}

/**
 * @brief Create an AssociationMap based on the scores and assignments.
 *
 * @param scores The ScoreMap containing the scores for each track-detection pair.
 * @param assignments The vector of assignments representing the indices of the detections for each track.
 * @param track_set The set of track UUIDs.
 * @param detection_set The set of detection UUIDs.
 * @return The created AssociationMap.
 */
inline auto associationMapFromScoreMap(const ScoreMap& scores, const std::vector<long>& assignments,
                                       const std::set<std::string>& track_set,
                                       const std::set<std::string>& detection_set) -> AssociationMap
{
  // Create the AssociationMap
  AssociationMap associations;

  // Iterate over the assignments and populate the AssociationMap
  for (int i = 0; i < assignments.size(); i++)
  {
    // Get the track UUID at the current index
    const auto& track_uuid = getElementAt(track_set, i);

    // Get the detection UUID corresponding to the assigned index
    const auto& detection_uuid = getElementAt(detection_set, assignments[i]);

    // Add the detection UUID to the track's vector in the AssociationMap
    associations[track_uuid].push_back(detection_uuid);
  }

  return associations;
}

/**
 * @brief Perform track association using the GNN (Global Nearest Neighbor) algorithm.
 *
 * @param scores The ScoreMap containing the scores for each track-detection pair.
 * @return The resulting AssociationMap after track association.
 */
inline auto gnnAssociator(const ScoreMap& scores) -> AssociationMap
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

/**
 * @brief Print the contents of an AssociationMap to the console.
 *
 * @param associations The AssociationMap to be printed.
 */
inline auto printAssociationMap(const AssociationMap& associations) -> void
{
  for (const auto& pair : associations)
  {
    const std::string& track_uuid = pair.first;
    const std::vector<std::string>& detection_uuids = pair.second;

    // Print track UUID
    std::cout << "Track UUID: " << track_uuid << std::endl;

    // Print assigned detection UUIDs
    std::cout << "Assigned Detection UUIDs: ";
    for (const std::string& detection_uuid : detection_uuids)
    {
      std::cout << detection_uuid << " ";
    }
    std::cout << std::endl << std::endl;
  }
}

/**
 * @brief Constant expression visitor for GNN association.
 *
 * This visitor applies the GNN association algorithm to the given scores and returns the resulting
 * AssociationMap.
 *
 * @param scores Score map containing the scores for track-detection pairs.
 * @return Association map indicating the assigned detections for each track.
 */
constexpr Visitor gnn_association_visitor{ [](const auto& scores) -> AssociationMap { return gnnAssociator(scores); } };

/**
 * @brief Associates detections to tracks using the specified association visitor.
 *
 * @param gated_scores Score map containing the scores for track-detection pairs.
 * @param association_visitor Visitor function or functor for computing associations.
 * @return Association map indicating the assigned detections for each track.
 */
template <typename AssociationVisitor>
inline auto associateDetectionsToTracks(const ScoreMap gated_scores, const AssociationVisitor& association_visitor)
    -> AssociationMap
{
  AssociationMap associations = std::visit(association_visitor, std::variant<ScoreMap>(gated_scores));

  return associations;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TRACK_MATCHING_HPP
