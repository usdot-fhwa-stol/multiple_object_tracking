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

#include <gtest/gtest.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <cooperative_perception/track.hpp>
#include <cooperative_perception/scoring.hpp>

namespace cp = cooperative_perception;

TEST(TestTrackMatching, VerifyLibraryInstallation)
{
  using namespace dlib;
  // Example from library
  // source: http://dlib.net/max_cost_assignment_ex.cpp.html

  matrix<int> cost(3, 3);
  cost = 1, 2, 6, 5, 3, 6, 4, 5, 0;

  // To find out the best assignment of people to jobs we just need to call this function.
  std::vector<long> result_assignment = max_cost_assignment(cost);

  // Assignments should be:  [2, 0, 1] which indicates that we should assign
  // the person from the first row of the cost matrix to job 2, the middle row person to
  // job 0, and the bottom row person to job 1.
  std::vector<long> expected_assignment{ 2, 0, 1 };

  // The optimal cost should be:  16.0
  // which is correct since our optimal assignment is 6+5+5
  auto expected_optimal_cost{ 16.0 };
  // Compute optimal cost
  auto result_optimal_cost{ assignment_cost(cost, result_assignment) };

  for (unsigned int i = 0; i < result_assignment.size(); i++)
  {
    EXPECT_EQ(expected_assignment[i], result_assignment[i]);
  }

  EXPECT_DOUBLE_EQ(expected_optimal_cost, result_optimal_cost);
}

TEST(TestTrackMatching, Example)
{
  using namespace dlib;

  int n = 3;
  matrix<int> test(n, n);
  for (int i = 0; i < n * n; i++)
  {
    test(i) = i;
  }
  std::cout << test << std::endl;

  matrix<int> cost(3, 2);
  cost = 1, 2, 6, 5, 3, 6;
  std::vector<long> result_assignment = max_cost_assignment(cost);
  for (int i = 0; i < result_assignment.size(); i++)
  {
    std::cout << result_assignment[i] << std::endl;
  }
}
