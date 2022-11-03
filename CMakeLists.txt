cmake_minimum_required(VERSION 3.16)
project(cooperative_perception_core)
enable_testing()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Language standard is required for all project targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)

include(dependencies.cmake)

add_subdirectory(src)
