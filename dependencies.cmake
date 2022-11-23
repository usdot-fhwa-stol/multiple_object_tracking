find_package(Boost REQUIRED COMPONENTS container)
find_package(Eigen3 REQUIRED)
find_package(units REQUIRED)

if(cooperative_perception_ENABLE_TESTING OR PROJECT_IS_TOP_LEVEL)
  find_package(GTest REQUIRED)
endif()
