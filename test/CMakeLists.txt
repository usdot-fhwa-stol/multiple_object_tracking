add_executable(test_cooperative_perceptionExecutable
  test_ctrv_model.cpp
)

target_link_libraries(test_cooperative_perceptionExecutable
  PRIVATE
    cooperative_perception::cooperative_perception
    GTest::GTest
    GTest::Main
)

include(GoogleTest)
gtest_discover_tests(test_cooperative_perceptionExecutable)