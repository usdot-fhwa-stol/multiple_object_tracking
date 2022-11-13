find_package(Eigen3 REQUIRED)

if(cooperative_perception_ENABLE_TESTING OR PROJECT_IS_TOP_LEVEL)
  find_package(GTest REQUIRED)
endif()
