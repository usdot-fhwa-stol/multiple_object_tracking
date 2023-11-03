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

add_library(multiple_object_trackingLibrary
  ctrv_model.cpp
  ctra_model.cpp
)

add_library(multiple_object_tracking::multiple_object_tracking ALIAS multiple_object_trackingLibrary)

target_include_directories(multiple_object_trackingLibrary
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(multiple_object_trackingLibrary
  PUBLIC
    Boost::container
    Eigen3::Eigen
    units::units
    dlib::dlib
    nlohmann_json::nlohmann_json
)

set_target_properties(multiple_object_trackingLibrary PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
)

include(GNUInstallDirs)

install(TARGETS multiple_object_trackingLibrary
  EXPORT multiple_object_trackingTargets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/multiple_object_tracking
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN *.hpp
)

install(FILES ${PROJECT_SOURCE_DIR}/cmake/multiple_object_trackingConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/multiple_object_tracking
)

install(EXPORT multiple_object_trackingTargets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/multiple_object_tracking/multiple_object_tracking
  FILE multiple_object_trackingTargets.cmake
  NAMESPACE multiple_object_tracking::
)
