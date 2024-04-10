# Copyright 2022 Leidos
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

cmake_minimum_required(VERSION 3.16)
project(multiple_object_tracking)
enable_testing()

# In CMake 3.21, PROJECT_IS_TOP_LEVEL is provided, so if block can be removed when the
# project upgrades to that CMake version
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(PROJECT_IS_TOP_LEVEL TRUE CACHE INTERNAL "")
else()
  set(PROJECT_IS_TOP_LEVEL FALSE CACHE INTERNAL "")
endif()

option(multiple_object_tracking_ENABLE_TESTING "Enable unit tests for multiple_object_tracking" ${PROJECT_IS_TOP_LEVEL})
option(multiple_object_tracking_EXPORT_COMPILE_COMMANDS "Export compile commands" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ${multiple_object_tracking_EXPORT_COMPILE_COMMANDS})

# Language standard is required for all project targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_C_STANDARD 11) # C17 available starting in CMake 3.21
set(CMAKE_C_STANDARD_REQUIRED TRUE)

include(dependencies.cmake)

add_subdirectory(src)

if(multiple_object_tracking_ENABLE_TESTING OR PROJECT_IS_TOP_LEVEL)
  add_subdirectory(test)
endif()
