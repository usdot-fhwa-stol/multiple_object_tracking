# Copyright 2023 Leidos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Developed by the Human and Vehicle Ensembles (HIVE) Lab at Virginia Commonwealth University (VCU)

add_executable(test_cooperative_perceptionExecutable
  test_angle.cpp
  test_ctrv_model.cpp
  test_ctra_model.cpp
  test_detection.cpp
  test_unscented_transform.cpp
  test_host_object_alignment.cpp
  test_scoring.cpp
  test_track_matching.cpp
  test_gating.cpp
  test_unscented_kalman_filter.cpp
)

target_link_libraries(test_cooperative_perceptionExecutable
  PRIVATE
  cooperative_perception::cooperative_perception
  GTest::GTest
  GTest::Main
)

include(GoogleTest)
gtest_discover_tests(test_cooperative_perceptionExecutable)
