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

#include <gtest/gtest.h>

#include <multiple_object_tracking/gating.hpp>
#include <multiple_object_tracking/scoring.hpp>

namespace mot = multiple_object_tracking;

TEST(TestGating, TestPruning)
{
  mot::ScoreMap scores{
    {{mot::Uuid{"track1"}, mot::Uuid{"detection1"}}, 10.0},
    {{mot::Uuid{"track2"}, mot::Uuid{"detection2"}}, 20.0},
    {{mot::Uuid{"track3"}, mot::Uuid{"detection3"}}, 30.0}};

  const mot::ScoreMap expected_pruned_scores{
    {{mot::Uuid{"track1"}, mot::Uuid{"detection1"}}, 10.0},
    {{mot::Uuid{"track2"}, mot::Uuid{"detection2"}}, 20.0}};

  mot::prune_track_and_detection_scores_if(scores, [](const auto & score) { return score > 25.0; });

  EXPECT_EQ(std::size(scores), std::size(expected_pruned_scores));

  for (const auto & [pair, score] : scores) {
    EXPECT_FLOAT_EQ(expected_pruned_scores.at(pair), score);
  }
}
