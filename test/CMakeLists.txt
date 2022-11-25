add_executable(test_cooperative_perceptionExecutable
  test_ctrv_model.cpp
  test_detected_object.cpp
  test_unscented_transform.cpp
)

target_link_libraries(test_cooperative_perceptionExecutable
  PRIVATE
    cooperative_perception::cooperative_perception
    GTest::GTest
    GTest::Main
)

include(GoogleTest)
gtest_discover_tests(test_cooperative_perceptionExecutable)
