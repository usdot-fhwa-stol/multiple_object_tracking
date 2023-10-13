# Copyright 2023 Leidos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(CPM_DOWNLOAD_VERSION  0.38.2)
include(cmake/get_cpm.cmake)

CPMAddPackage(NAME units
  GITHUB_REPOSITORY nholthaus/units
  GIT_TAG v2.3.3
  OPTIONS
    "BUILD_TESTS FALSE"
    "BUILD_DOCS FALSE"
)

CPMAddPackage(NAME Eigen3
  GITLAB_REPOSITORY "libeigen/eigen"
  GIT_TAG 3.3.7
  DOWNLOAD_ONLY TRUE
  OPTIONS
    "EIGEN_BUILD_DOC FALSE"
    "EIGEN_BUILD_TESTING FALSE"
    "BUILD_TESTING FALSE"
    "EIGEN_BUILD_PKGCONFIG FALSE"
)

if(Eigen3_ADDED)
  add_library(Eigen3 INTERFACE IMPORTED GLOBAL)
  add_library(Eigen3::Eigen ALIAS Eigen3)

  target_include_directories(Eigen3 INTERFACE ${Eigen3_SOURCE_DIR})
endif()

CPMAddPackage(NAME dlib
  GITHUB_REPOSITORY davisking/dlib
  GIT_TAG v19.24.2
  OPTIONS
    "DLIB_NO_GUI_SUPPORT TRUE"
    "DLIB_JPEG_SUPPORT FALSE"
    "DLIB_WEBP_SUPPORT FALSE"
    "DLIB_PNG_SUPPORT FALSE"
    "DLIB_GIF_SUPPORT FALSE"
    "DLIB_USE_MKL_FFT FALSE"
    "DLIB_USE_FFMPEG FALSE"
)

CPMAddPackage(NAME nlohmann_json
  GITHUB_REPOSITORY nlohmann/json
  GIT_TAG v3.11.2
  OPTIONS
    "JSON_BuildTests FALSE"
    "JSON_Install TRUE"
)

find_package(Boost REQUIRED COMPONENTS container)

if(cooperative_perception_ENABLE_TESTING OR PROJECT_IS_TOP_LEVEL)
  CPMAddPackage("gh:google/googletest#v1.14.0")
endif()
