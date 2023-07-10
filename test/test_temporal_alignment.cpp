/*
 * Copyright 2023 Leidos
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Developed by the Human and Vehicle Ensembles (HIVE) Lab at Virginia Commonwealth University (VCU)
 */

#include <gtest/gtest.h>
#include <units.h>
#include <cooperative_perception/utils.hpp>
#include <cooperative_perception/angle.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/detection.hpp>
#include <cooperative_perception/track.hpp>
#include <cooperative_perception/temporal_alignment.hpp>

namespace cp = cooperative_perception;

/**
 * Test the temporal alignment for a CTRV Detection at one second into the future
 */
TEST(TestTemporalAlignment, CtrvDetection)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>;

  // Declaring initial state and covariance for the detection that will be temporally aligned
  TestDetection detection{ .timestamp{ units::time::second_t{ 0 } },
                           .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad),
                                                  0.3528_rad_per_s } },
                           .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                                                 { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                                                 { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                                                 { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                                                 { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } } };
  // Expected values
  const cp::CtrvState expected_state{ 7.43226_m, 2.73934_m, 2.2049_mps, cp::Angle(0.8543_rad), 0.3528_rad_per_s };

  const cp::CtrvStateCovariance expected_covariance{ { 0.064633, -0.0671223, 0.00564156, -0.0462884, -0.0239845 },
                                                     { -0.0671223, 0.110445, 0.00623941, 0.0653499, 0.0332627 },
                                                     { 0.00564156, 0.00623941, 0.0054, 0.0015, 0.000800001 },
                                                     { -0.0462884, 0.0653499, 0.0015, 0.0421, 0.0223 },
                                                     { -0.0239845, 0.0332627, 0.000800001, 0.0223, 0.0123 } };

  // Amount of time the detection will be propagated into the future
  auto time_step{ 1.0_s };

  // Call the functions under test
  cp::propagateToTime(detection, time_step, cp::ukf_prediction_visitor);
  auto result_detection = cp::predictToTime(detection, time_step, cp::ukf_prediction_visitor);

  // Check that function returns expected value
  // Check detection that was modified in place
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.covariance, 4),
                                     cp::utils::roundToDecimalPlace(expected_covariance, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(detection.timestamp.value(), time_step.value()));

  // Check new predicted detection
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(result_detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(result_detection.covariance, 4),
                                     cp::utils::roundToDecimalPlace(expected_covariance, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(result_detection.timestamp.value(), time_step.value()));
};

/**
 * Test the temporal alignment for a CTRV Detection at five second into the future
 */
TEST(TestTemporalAlignment, CtrvDetectionFiveSeconds)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>;

  // Declaring initial state and covariance for the detection that will be temporally aligned
  TestDetection detection{ .timestamp{ units::time::second_t{ 0 } },
                           .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad),
                                                  0.3528_rad_per_s } },
                           .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                                                 { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                                                 { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                                                 { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                                                 { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } } };
  // Expected values
  const cp::CtrvState expected_state{ 7.43226_m, 2.73934_m, 2.20490_mps, cp::Angle(0.85430_rad), 0.35280_rad_per_s };
  const cp::CtrvStateCovariance expected_covariance{ { 0.06463, -0.06712, 0.00564, -0.04629, -0.02398 },
                                                     { -0.06712, 0.11044, 0.00624, 0.06535, 0.03326 },
                                                     { 0.00564, 0.00624, 0.00540, 0.00150, 0.00080 },
                                                     { -0.04629, 0.06535, 0.00150, 0.04210, 0.02230 },
                                                     { -0.02398, 0.03326, 0.00080, 0.02230, 0.01230 } };

  // Amount of time the detection will be propagated into the future
  auto time_step{ 1.0_s };

  // Call the function under test
  cp::propagateToTime(detection, time_step, cp::ukf_prediction_visitor);

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.covariance, 5),
                                     cp::utils::roundToDecimalPlace(expected_covariance, 5)));
  EXPECT_TRUE(cp::utils::almostEqual(detection.timestamp.value(), time_step.value()));
}

/**
 * Test the temporal alignment for a CTRA Detection at one second into the future
 */
TEST(TestTemporalAlignment, CtraDetection)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtraState, cp::CtraStateCovariance>;

  // Declaring initial state and covariance for the detection that will be temporally aligned
  TestDetection detection{ .timestamp{ units::time::second_t{ 0 } },
                           .state{ cp::CtraState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad),
                                                  0.3528_rad_per_s, 1_mps_sq } },
                           .covariance{ cp::CtraStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5 },
                                                                 { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123 },
                                                                 { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34 },
                                                                 { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009 },
                                                                 { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021 },
                                                                 { 0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701 } } } };
  // Expected values
  const cp::CtraState expected_state{ 7.79608_m,        3.06936_m, 3.2049_mps, cp::Angle(0.8543_rad),
                                      0.3528_rad_per_s, 1_mps_sq };
  const cp::CtraStateCovariance expected_covariance{ { 39.64152, 34.73881, 106.16204, -0.06633, -0.03572, 106.28491 },
                                                     { 34.73881, 32.49688, 95.98216, 0.06945, 0.03433, 96.09180 },
                                                     { 106.16204, 95.98216, 288.55994, 0.01260, 0.00290, 288.89456 },
                                                     { -0.06633, 0.06945, 0.01260, 0.04210, 0.02230, 0.01110 },
                                                     { -0.03572, 0.03433, 0.00290, 0.02230, 0.01230, 0.00210 },
                                                     { 106.28493, 96.09180, 288.89453, 0.01110, 0.00210, 289.23453 } };

  // Amount of time the detection will be propagated into the future
  auto time_step{ 1.0_s };

  // Call the function under test
  cp::propagateToTime(detection, time_step, cp::ukf_prediction_visitor);

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.covariance, 5),
                                     cp::utils::roundToDecimalPlace(expected_covariance, 5)));
  EXPECT_TRUE(cp::utils::almostEqual(detection.timestamp.value(), time_step.value()));
}

/**
 * Test the temporal alignment for a CTRA Detection at five second into the future
 */
TEST(TestTemporalAlignment, CtraDetectionFiveSeconds)
{
  using namespace units::literals;

  using TestDetection = cp::Detection<cp::CtraState, cp::CtraStateCovariance>;

  // Declaring initial state and covariance for the detection that will be temporally aligned
  TestDetection detection{ .timestamp{ units::time::second_t{ 0 } },
                           .state{ cp::CtraState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad),
                                                  0.3528_rad_per_s, 1_mps_sq } },
                           .covariance{ cp::CtraStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5 },
                                                                 { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123 },
                                                                 { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34 },
                                                                 { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009 },
                                                                 { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021 },
                                                                 { 0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701 } } } };
  // Expected values
  const cp::CtraState expected_state{ 7.79608_m,         3.06936_m,     3.20490_mps, cp::Angle(0.85430_rad),
                                      0.35280_rad_per_s, 1.00000_mps_sq };
  const cp::CtraStateCovariance expected_covariance{ { 39.64152, 34.73881, 106.16204, -0.06633, -0.03572, 106.28491 },
                                                     { 34.73881, 32.49688, 95.98216, 0.06945, 0.03433, 96.09180 },
                                                     { 106.16204, 95.98216, 288.55994, 0.01260, 0.00290, 288.89456 },
                                                     { -0.06633, 0.06945, 0.01260, 0.04210, 0.02230, 0.01110 },
                                                     { -0.03572, 0.03433, 0.00290, 0.02230, 0.01230, 0.00210 },
                                                     { 106.28493, 96.09180, 288.89453, 0.01110, 0.00210, 289.23453 } };

  // Amount of time the detection will be propagated into the future
  auto time_step{ 1.0_s };

  // Call the function under test
  cp::propagateToTime(detection, time_step, cp::ukf_prediction_visitor);

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.covariance, 5),
                                     cp::utils::roundToDecimalPlace(expected_covariance, 5)));
  EXPECT_TRUE(cp::utils::almostEqual(detection.timestamp.value(), time_step.value()));
}

/**
 * Test the temporal alignment for a CTRV Track at one second into the future
 */
TEST(TestTemporalAlignment, CtrvTrack)
{
  using namespace units::literals;

  using TestTrack = cp::Track<cp::CtrvState, cp::CtrvStateCovariance>;

  // Declaring initial state and covariance for the track that will be temporally aligned
  TestTrack track{ .timestamp{ units::time::second_t{ 0 } },
                   .state{ cp::CtrvState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s } },
                   .covariance{ cp::CtrvStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                                         { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                                         { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                                         { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                                         { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } } } };
  // Expected values
  const cp::CtrvState expected_state{ 7.43226_m, 2.73934_m, 2.2049_mps, cp::Angle(0.8543_rad), 0.3528_rad_per_s };

  const cp::CtrvStateCovariance expected_covariance{ { 0.064633, -0.0671223, 0.00564156, -0.0462884, -0.0239845 },
                                                     { -0.0671223, 0.110445, 0.00623941, 0.0653499, 0.0332627 },
                                                     { 0.00564156, 0.00623941, 0.0054, 0.0015, 0.000800001 },
                                                     { -0.0462884, 0.0653499, 0.0015, 0.0421, 0.0223 },
                                                     { -0.0239845, 0.0332627, 0.000800001, 0.0223, 0.0123 } };

  // Amount of time the track will be propagated into the future
  auto time_step{ 1.0_s };

  // Call the functions under test
  cp::propagateToTime(track, time_step, cp::ukf_prediction_visitor);
  auto result_track = cp::predictToTime(track, time_step, cp::ukf_prediction_visitor);

  // Check that function returns expected value
  // Check track that was modified in place
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(track.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(track.covariance, 4),
                                     cp::utils::roundToDecimalPlace(expected_covariance, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(track.timestamp.value(), time_step.value()));

  // Check new predicted track
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(result_track.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(result_track.covariance, 4),
                                     cp::utils::roundToDecimalPlace(expected_covariance, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(result_track.timestamp.value(), time_step.value()));
}

/**
 * Test the temporal alignment for a CTRA Track at one second into the future
 */
TEST(TestTemporalAlignment, CtraTrack)
{
  using namespace units::literals;

  using TestTrack = cp::Track<cp::CtraState, cp::CtraStateCovariance>;

  // Declaring initial state and covariance for the track that will be temporally aligned
  TestTrack track{ .timestamp{ units::time::second_t{ 0 } },
                   .state{ cp::CtraState{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s,
                                          1_mps_sq } },
                   .covariance{ cp::CtraStateCovariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5 },
                                                         { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123 },
                                                         { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34 },
                                                         { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009 },
                                                         { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021 },
                                                         { 0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701 } } } };
  // Expected values
  const cp::CtraState expected_state{ 7.79608_m,        3.06936_m, 3.2049_mps, cp::Angle(0.8543_rad),
                                      0.3528_rad_per_s, 1_mps_sq };
  const cp::CtraStateCovariance expected_covariance{ { 39.64152, 34.73881, 106.16204, -0.06633, -0.03572, 106.28491 },
                                                     { 34.73881, 32.49688, 95.98216, 0.06945, 0.03433, 96.09180 },
                                                     { 106.16204, 95.98216, 288.55994, 0.01260, 0.00290, 288.89456 },
                                                     { -0.06633, 0.06945, 0.01260, 0.04210, 0.02230, 0.01110 },
                                                     { -0.03572, 0.03433, 0.00290, 0.02230, 0.01230, 0.00210 },
                                                     { 106.28493, 96.09180, 288.89453, 0.01110, 0.00210, 289.23453 } };

  // Amount of time the track will be propagated into the future
  auto time_step{ 1.0_s };

  // Call the function under test
  cp::propagateToTime(track, time_step, cp::ukf_prediction_visitor);

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(track.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(track.covariance, 5),
                                     cp::utils::roundToDecimalPlace(expected_covariance, 5)));
  EXPECT_TRUE(cp::utils::almostEqual(track.timestamp.value(), time_step.value()));
};
