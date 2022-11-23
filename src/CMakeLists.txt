add_library(cooperative_perception_coreLibrary
  ctrv_model.cpp
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
)
