#include <gtest/gtest.h>
#include <cooperative_perception/ctrv_model.hpp>

/**
 * Test CTRV nextState function against pure rotation
 */
TEST(TestCtrvModel, NextStatePureRotation)
{
  cooperative_perception::CtrvState state;
  state << 0, 0, 0, 0, 1;

  const auto time_step{ 0.5F };

  const auto next_state{ cooperative_perception::nextState(state, time_step) };

  EXPECT_FLOAT_EQ(next_state[0], 0);
  EXPECT_FLOAT_EQ(next_state[1], 0);
  EXPECT_FLOAT_EQ(next_state[2], 0);
  EXPECT_FLOAT_EQ(next_state[3], 0.5);
  EXPECT_FLOAT_EQ(next_state[4], 1);
}

/**
 * Test CTRV nextState function against pure translation
 */
TEST(TestCtrvModel, NextStatePureTranslation)
{
  cooperative_perception::CtrvState state;
  state << 0, 0, 1, 0, 0;

  const auto time_step{ 0.5F };

  const auto next_state{ cooperative_perception::nextState(state, time_step) };

  EXPECT_FLOAT_EQ(next_state[0], 0.5);
  EXPECT_FLOAT_EQ(next_state[1], 0);
  EXPECT_FLOAT_EQ(next_state[2], 1);
  EXPECT_FLOAT_EQ(next_state[3], 0);
  EXPECT_FLOAT_EQ(next_state[4], 0);
}

/**
 * Test CTRV nextState function against mixed rotation and translation
 */
TEST(TestCtrvModel, NextStateRotationAndTranslation)
{
  cooperative_perception::CtrvState state;
  state << 0, 0, 1, 0, 1;

  const auto time_step{ 0.5F };

  const auto next_state{ cooperative_perception::nextState(state, time_step) };

  EXPECT_FLOAT_EQ(next_state[0], 0.479425539);
  EXPECT_FLOAT_EQ(next_state[1], 0.122417438);
  EXPECT_FLOAT_EQ(next_state[2], 1);
  EXPECT_FLOAT_EQ(next_state[3], 0.5);
  EXPECT_FLOAT_EQ(next_state[4], 1);
}
