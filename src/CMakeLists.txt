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

add_library(cooperative_perception_coreLibrary
  ctrv_model.cpp
  ctra_model.cpp
)

add_library(cooperative_perception::cooperative_perception ALIAS cooperative_perception_coreLibrary)

target_include_directories(cooperative_perception_coreLibrary
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(cooperative_perception_coreLibrary
  PUBLIC
    Boost::container
    Eigen3::Eigen
    units::units
    dlib::dlib
    nlohmann_json::nlohmann_json
)

set_target_properties(cooperative_perception_coreLibrary PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
)

include(GNUInstallDirs)

install(TARGETS cooperative_perception_coreLibrary
  EXPORT cooperative_perception_coreTargets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/cooperative_perception
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN *.hpp
)

install(FILES ${PROJECT_SOURCE_DIR}/cmake/cooperative_perception_coreConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cooperative_perception_core
)

install(EXPORT cooperative_perception_coreTargets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cooperative_perception_core/cooperative_perception_core
  FILE cooperative_perception_coreTargets.cmake
  NAMESPACE cooperative_perception_core::
)
