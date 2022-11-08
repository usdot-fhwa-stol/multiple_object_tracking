cmake_minimum_required(VERSION 3.16)
project(cooperative_perception_core)
enable_testing()

# In CMake 3.21, PROJECT_IS_TOP_LEVEL is provided, so if block can be removed when the
# project upgrades to that CMake version
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(PROJECT_IS_TOP_LEVEL TRUE CACHE INTERNAL "")
else()
  set(PROJECT_IS_TOP_LEVEL FALSE CACHE INTERNAL "")
endif()

option(cooperative_perception_ENABLE_TESTING "Enable unit tests for cooperative_perception" ${PROJECT_IS_TOP_LEVEL})
option(cooperative_perception_EXPORT_COMPILE_COMMANDS "Export compile commands" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ${cooperative_perception_EXPORT_COMPILE_COMMANDS})

# Language standard is required for all project targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11) # C17 available starting in CMake 3.21

include(dependencies.cmake)

add_subdirectory(src)

if(cooperative_perception_ENABLE_TESTING OR PROJECT_IS_TOP_LEVEL)
  add_subdirectory(test)
endif()
