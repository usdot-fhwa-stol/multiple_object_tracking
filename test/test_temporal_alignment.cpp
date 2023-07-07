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
#include <cooperative_perception/temporal_alignment.hpp>

namespace cp = cooperative_perception;

/**
 * Test the temporal alignment for a CTRV Detection at one second into the future
 */
TEST(TestTemporalAlignment, CtrvDetection)
{
  using namespace units::literals;
  // Declaring Initial state and covariance
  auto time_step{ 1.0_s };
  const cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                            { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                            { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                            { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                            { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };
  // Expected values
  const cp::CtrvState expected_state{ 7.43226_m, 2.73934_m, 2.2049_mps, cp::Angle(0.8543_rad), 0.3528_rad_per_s };

  const cp::CtrvStateCovariance expected_covariance{ { 0.064633, -0.0671223, 0.00564156, -0.0462884, -0.0239845 },
                                                     { -0.0671223, 0.110445, 0.00623941, 0.0653499, 0.0332627 },
                                                     { 0.00564156, 0.00623941, 0.0054, 0.0015, 0.000800001 },
                                                     { -0.0462884, 0.0653499, 0.0015, 0.0421, 0.0223 },
                                                     { -0.0239845, 0.0332627, 0.000800001, 0.0223, 0.0123 } };

  // Created Detection that will be temporally align
  auto detection =
      cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>{ units::time::second_t{ 0 }, state, covariance };

  // Call the function under test
  cp::alignToTime(detection, time_step);

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(detection.covariance, expected_covariance));
  EXPECT_TRUE(cp::utils::almostEqual(detection.timestamp.value(), time_step.value()));
};

/**
 * Test the temporal alignment for a CTRV Detection at five second into the future
 */
TEST(TestTemporalAlignment, CtrvDetectionFiveSeconds)
{
  using namespace units::literals;
  // Declaring Initial state and covariance
  auto time_step{ 5.0_s };
  const cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                            { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                            { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                            { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                            { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };
  // Expected values
  const cp::CtrvState expected_state{ 7.6724_m, 10.0889_m, 2.2049_mps, cp::Angle(2.2655_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance expected_covariance{ { 11.3487, -0.258615, -0.015211, -2.1669, -0.370411 },
                                                     { -0.258615, 1.57984, 0.0232558, 0.0112134, 0.000969029 },
                                                     { -0.015211, 0.0232558, 0.0054, 0.00470001, 0.000800001 },
                                                     { -2.1669, 0.0112134, 0.00470001, 0.4173, 0.0715 },
                                                     { -0.370411, 0.000969029, 0.000800001, 0.0715, 0.0123 } };
  // Created Detection that will be temporally align
  auto detection =
      cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>{ units::time::second_t{ 0 }, state, covariance };

  // Call the function under test
  cp::alignToTime(detection, time_step);

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(detection.covariance, expected_covariance));
  EXPECT_TRUE(cp::utils::almostEqual(detection.timestamp.value(), time_step.value()));
}

/**
 * Test the temporal alignment for a CTRA Detection at one second into the future
 */
TEST(TestTemporalAlignment, CtraDetection)
{
  using namespace units::literals;

  // Declaring Initial state and covariance
  auto time_step{ 1.0_s };
  const cp::CtraState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s, 1_mps_sq };
  const cp::CtraStateCovariance covariance{
    { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5 },  { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123 },
    { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34 },   { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009 },
    { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021 }, { 0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701 }
  };

  // Expected values
  const cp::CtraState expected_state{
    5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s, 1_mps_sq
  };
  const cp::CtraStateCovariance expected_covariance{
    { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5 },  { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.123 },
    { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34 },   { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, 0.009 },
    { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021 }, { 0.5, 0.123, -0.34, 0.009, 0.0021, -0.8701 }
  };

  // Created Detection that will be temporally align
  auto detection =
      cp::Detection<cp::CtraState, cp::CtraStateCovariance>{ units::time::second_t{ 0 }, state, covariance };

  // Call the function under test
  cp::alignToTime(detection, time_step);

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(detection.covariance, expected_covariance));
  EXPECT_TRUE(cp::utils::almostEqual(detection.timestamp.value(), time_step.value()));
};

/**
 * Test the temporal alignment for a CTRA Detection at five second into the future
 */
TEST(TestTemporalAlignment, CtraDetectionFiveSeconds)
{
  using namespace units::literals;
  // Declaring Initial state and covariance
  auto time_step{ 5.0_s };
  const cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, cp::Angle(0.5015_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                            { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                            { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                            { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                            { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };
  // Expected values
  const cp::CtrvState expected_state{ 7.6724_m, 10.0889_m, 2.2049_mps, cp::Angle(2.2655_rad), 0.3528_rad_per_s };
  const cp::CtrvStateCovariance expected_covariance{ { 11.3487, -0.258615, -0.015211, -2.1669, -0.370411 },
                                                     { -0.258615, 1.57984, 0.0232558, 0.0112134, 0.000969029 },
                                                     { -0.015211, 0.0232558, 0.0054, 0.00470001, 0.000800001 },
                                                     { -2.1669, 0.0112134, 0.00470001, 0.4173, 0.0715 },
                                                     { -0.370411, 0.000969029, 0.000800001, 0.0715, 0.0123 } };
  // Created Detection that will be temporally align
  auto detection =
      cp::Detection<cp::CtrvState, cp::CtrvStateCovariance>{ units::time::second_t{ 0 }, state, covariance };

  // Call the function under test
  cp::alignToTime(detection, time_step);

  EXPECT_TRUE(cp::utils::almostEqual(cp::utils::roundToDecimalPlace(detection.state, 4),
                                     cp::utils::roundToDecimalPlace(expected_state, 4)));
  EXPECT_TRUE(cp::utils::almostEqual(detection.covariance, expected_covariance));
  EXPECT_TRUE(cp::utils::almostEqual(detection.timestamp.value(), time_step.value()));
};
