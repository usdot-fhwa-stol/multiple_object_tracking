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

add_executable(test_multiple_object_trackingExecutable
  test_angle.cpp
  test_ctrv_model.cpp
  test_ctra_model.cpp
  test_detection.cpp
  test_unscented_transform.cpp
  test_temporal_alignment.cpp
  test_unscented_kalman_filter.cpp
  test_scoring.cpp
  test_track_matching.cpp
  test_gating.cpp
  test_fusing.cpp
  test_json_parsing.cpp
  test_clustering.cpp
  test_track_management.cpp
)

target_include_directories(test_multiple_object_trackingExecutable
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test>
)

target_link_libraries(test_multiple_object_trackingExecutable
  PRIVATE
    multiple_object_tracking::multiple_object_tracking
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
    GTest::gmock_main
)

include(GoogleTest)
gtest_discover_tests(test_multiple_object_trackingExecutable
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)
add_custom_target(
  run_unit_test ALL
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS test_multiple_object_trackingExecutable
  )
if(GENERATE_COVERAGE)

  find_program( GCOVR_PATH gcovr )
  if(NOT GCOVR_PATH)
      message(FATAL_ERROR "gcovr not found! Aborting...")
  endif() # NOT GCOVR_PATH

  set(COVERAGE_COMPILER_FLAGS "--coverage" CACHE INTERNAL "")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COVERAGE_COMPILER_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_COMPILER_FLAGS}")
  set(COVERAGE_OUTPUT_FILE coverage_sonar.xml)

  add_custom_target(
      generate_coverage ALL
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      # use --exclude-throw-branches to get branch coverage more sensible per https://gcovr.com/en/stable/faq.html
      COMMAND ${GCOVR_PATH} --exclude-throw-branches --sonarqube build/${COVERAGE_OUTPUT_FILE} -s --filter src 2> gcov_run_cmake.out
      DEPENDS run_unit_test
      )

endif() # GENERATE_COVERAGE
