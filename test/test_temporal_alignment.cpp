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

#include <multiple_object_tracking/angle.hpp>
#include <multiple_object_tracking/ctra_model.hpp>
#include <multiple_object_tracking/ctrv_model.hpp>
#include <multiple_object_tracking/dynamic_object.hpp>
#include <multiple_object_tracking/temporal_alignment.hpp>
#include <multiple_object_tracking/unscented_kalman_filter.hpp>
#include <multiple_object_tracking/utils.hpp>
#include <variant>

#include "multiple_object_tracking/test/gmock_matchers.hpp"

namespace mot = multiple_object_tracking;

/**
 * Test the temporal alignment for a CTRV Detection at one second into the future
 */
TEST(TestTemporalAlignment, CtrvDetection)
{
  using namespace units::literals;

  using TestDetection = mot::Detection<mot::CtrvState, mot::CtrvStateCovariance>;

  // Declaring initial state and covariance for the detection that will be temporally aligned
  mot::CtrvStateCovariance covariance;
  covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
    0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, -0.0020,
    0.0060, 0.0008, 0.0100, 0.0123;

  TestDetection detection{
    .timestamp{units::time::second_t{0}},
    .state{mot::CtrvState{5.7441_m, 1.3800_m, 2.2049_mps, mot::Angle(0.5015_rad), 0.3528_rad_per_s}},
    .covariance{std::move(covariance)},
    .uuid{mot::Uuid{""}}};
  // Expected values
  const mot::CtrvState expected_state{
    7.43226_m, 2.73934_m, 2.2049_mps, mot::Angle(0.8543_rad), 0.3528_rad_per_s};

  mot::CtrvStateCovariance expected_covariance;
  expected_covariance << 0.064633, -0.0671223, 0.00564156, -0.0462884, -0.0239845, -0.0671223,
    0.110445, 0.00623941, 0.0653499, 0.0332627, 0.00564156, 0.00623941, 0.0054, 0.0015, 0.000800001,
    -0.0462884, 0.0653499, 0.0015, 0.0421, 0.0223, -0.0239845, 0.0332627, 0.000800001, 0.0223,
    0.0123;

  // Amount of time the detection will be propagated into the future
  auto time_step{1.0_s};

  // Create detection variant
  std::variant<TestDetection> detection_variant{detection};
  std::variant<TestDetection> detection_variant_immutable{detection};

  // Call the functions under test
  mot::propagate_to_time(detection_variant, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
  TestDetection result_detection = std::get<TestDetection>(detection_variant);

  auto result_detection_variant = mot::predict_to_time(
    detection_variant_immutable, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
  TestDetection result_detection_immutable = std::get<TestDetection>(result_detection_variant);

  // Check that function returns expected value
  // Check detection that was modified in place
  EXPECT_THAT(result_detection.timestamp, DimensionedNear(time_step, 1e-4));
  EXPECT_THAT(result_detection.state, CtrvStateNear(expected_state, 1e-4));
  EXPECT_THAT(result_detection.covariance, EigenMatrixNear(expected_covariance, 1e-4));

  // Check new predicted detection
  EXPECT_THAT(result_detection_immutable.timestamp, DimensionedNear(time_step, 1e-4));
  EXPECT_THAT(result_detection_immutable.state, CtrvStateNear(expected_state, 1e-4));
  EXPECT_THAT(result_detection_immutable.covariance, EigenMatrixNear(expected_covariance, 1e-4));
}

/**
 * Test the temporal alignment for a CTRV Detection at five second into the future
 */
// TEST(TestTemporalAlignment, CtrvDetectionFiveSeconds)
// {
//   using namespace units::literals;

//   using TestDetection = mot::Detection<mot::CtrvState, mot::CtrvStateCovariance>;

//   // Declaring initial state and covariance for the detection that will be temporally aligned
//   mot::CtrvStateCovariance covariance;
//   covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
//     0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, -0.0020,
//     0.0060, 0.0008, 0.0100, 0.0123;

//   TestDetection detection{
//     .timestamp{units::time::second_t{0}},
//     .state{mot::CtrvState{5.7441_m, 1.3800_m, 2.2049_mps, mot::Angle(0.5015_rad), 0.3528_rad_per_s}},
//     .covariance{std::move(covariance)},
//     .uuid{mot::Uuid{""}}};
//   // Expected values
//   const mot::CtrvState expected_state{
//     7.67240_m, 10.08887_m, 2.20490_mps, mot::Angle(2.26550_rad), 0.35280_rad_per_s};
//   mot::CtrvStateCovariance expected_covariance;
//   expected_covariance << 11.34872, -0.25861, -0.01521, -2.16690, -0.37041, -0.25861, 1.57984,
//     0.02326, 0.01121, 0.00097, -0.01521, 0.02326, 0.00540, 0.00470, 0.00080, -2.16690, 0.01121,
//     0.00470, 0.41730, 0.07150, -0.37041, 0.00097, 0.00080, 0.07150, 0.01230;

//   // Amount of time the detection will be propagated into the future
//   auto time_step{5.0_s};

//   // Create detection variant
//   std::variant<TestDetection> detection_variant{detection};
//   std::variant<TestDetection> detection_variant_immutable{detection};

//   // Call the functions under test
//   mot::propagate_to_time(detection_variant, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
//   TestDetection result_detection = std::get<TestDetection>(detection_variant);

//   auto result_detection_variant = mot::predict_to_time(
//     detection_variant_immutable, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
//   TestDetection result_detection_immutable = std::get<TestDetection>(result_detection_variant);

//   // Check that function returns expected value
//   // Check detection that was modified in place
//   EXPECT_THAT(result_detection.timestamp, DimensionedNear(time_step, 1e-4));
//   EXPECT_THAT(result_detection.state, CtrvStateNear(expected_state, 1e-4));
//   EXPECT_THAT(result_detection.covariance, EigenMatrixNear(expected_covariance, 1e-4));

//   // Check new predicted detection
//   EXPECT_THAT(result_detection_immutable.timestamp, DimensionedNear(time_step, 1e-4));
//   EXPECT_THAT(result_detection_immutable.state, CtrvStateNear(expected_state, 1e-4));
//   EXPECT_THAT(result_detection_immutable.covariance, EigenMatrixNear(expected_covariance, 1e-4));
// }

/**
 * Test the temporal alignment for a CTRA Detection at one second into the future
 */
TEST(TestTemporalAlignment, CtraDetection)
{
  using namespace units::literals;

  using TestDetection = mot::Detection<mot::CtraState, mot::CtraStateCovariance>;

  // Declaring initial state and covariance for the detection that will be temporally aligned
  mot::CtraStateCovariance covariance;
  covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5, -0.0013, 0.0077, 0.0011, 0.0071,
    0.0060, 0.123, 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34, -0.0022, 0.0071, 0.0007, 0.0098,
    0.0100, 0.009, -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021, 0.5, 0.123, -0.34, 0.009,
    0.0021, -0.8701;

  TestDetection detection{
    .timestamp{units::time::second_t{0}},
    .state{mot::CtraState{
      5.7441_m, 1.3800_m, 2.2049_mps, mot::Angle(0.5015_rad), 0.3528_rad_per_s, 1_mps_sq}},
    .covariance{std::move(covariance)},
    .uuid{mot::Uuid{""}}};
  // Expected values
  const mot::CtraState expected_state{
    7.79608_m, 3.06936_m, 3.20490_mps, mot::Angle(0.85430_rad), 0.35280_rad_per_s, 1.00000_mps_sq};
  mot::CtraStateCovariance expected_covariance;
  expected_covariance << 39.64152, 34.73882, 106.16206, -0.06633, -0.03572, 106.28492, 34.73882,
    32.49688, 95.98216, 0.06945, 0.03433, 96.09180, 106.16206, 95.98216, 288.55997, 0.01260,
    0.00290, 288.89456, -0.06633, 0.06945, 0.01260, 0.04210, 0.02230, 0.01110, -0.03572, 0.03433,
    0.00290, 0.02230, 0.01230, 0.00210, 106.28492, 96.09181, 288.89453, 0.01110, 0.00210, 289.23456;

  // Amount of time the detection will be propagated into the future
  auto time_step{1.0_s};

  // Create detection variant
  std::variant<TestDetection> detection_variant{detection};
  std::variant<TestDetection> detection_variant_immutable{detection};

  // Call the functions under test
  mot::propagate_to_time(detection_variant, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
  TestDetection result_detection = std::get<TestDetection>(detection_variant);

  auto result_detection_variant = mot::predict_to_time(
    detection_variant_immutable, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
  TestDetection result_detection_immutable = std::get<TestDetection>(result_detection_variant);

  // Check that function returns expected value
  // Check detection that was modified in place
  EXPECT_THAT(result_detection.timestamp, DimensionedNear(time_step, 1e-4));
  EXPECT_THAT(result_detection.state, CtraStateNear(expected_state, 1e-4));
  EXPECT_THAT(result_detection.covariance, EigenMatrixNear(expected_covariance, 1e-4));

  // Check new predicted detection
  EXPECT_THAT(result_detection_immutable.timestamp, DimensionedNear(time_step, 1e-4));
  EXPECT_THAT(result_detection_immutable.state, CtraStateNear(expected_state, 1e-4));
  EXPECT_THAT(result_detection_immutable.covariance, EigenMatrixNear(expected_covariance, 1e-4));
}

/**
 * Test the temporal alignment for a CTRA Detection at five second into the future
 */
// TEST(TestTemporalAlignment, CtraDetectionFiveSeconds)
// {
//   using namespace units::literals;

//   using TestDetection = mot::Detection<mot::CtraState, mot::CtraStateCovariance>;

//   // Declaring initial state and covariance for the detection that will be temporally aligned
//   mot::CtraStateCovariance covariance;
//   covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5, -0.0013, 0.0077, 0.0011, 0.0071,
//     0.0060, 0.123, 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34, -0.0022, 0.0071, 0.0007, 0.0098,
//     0.0100, 0.009, -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021, 0.5, 0.123, -0.34, 0.009,
//     0.0021, -0.8701;

//   TestDetection detection{
//     .timestamp{units::time::second_t{0}},
//     .state{mot::CtraState{
//       5.7441_m, 1.3800_m, 2.2049_mps, mot::Angle(0.5015_rad), 0.3528_rad_per_s, 1_mps_sq}},
//     .covariance{std::move(covariance)},
//     .uuid{mot::Uuid{""}}};
//   // Expected values
//   const mot::CtraState expected_state{
//     8.92670_m, 21.12001_m, 7.20490_mps, mot::Angle(2.26550_rad), 0.35280_rad_per_s, 1.00000_mps_sq};
//   mot::CtraStateCovariance expected_covariance;
//   expected_covariance << 3272.17554, -2753.25513, -1620.65405, -2.53488, -0.43595, -324.23853,
//     -2753.25513, 34524.34766, 15688.62207, -8.90359, -1.52473, 3138.58984, -1620.65405, 15688.62207,
//     7227.46826, 0.10220, 0.01130, 1445.83264, -2.53488, -8.90359, 0.10220, 0.41730, 0.07150,
//     0.01950, -0.43595, -1.52473, 0.01130, 0.07150, 0.01230, 0.00210, -324.23856, 3138.58936,
//     1445.83252, 0.01950, 0.00210, 289.23456;

//   // Amount of time the detection will be propagated into the future
//   auto time_step{5.0_s};

//   // Create detection variant
//   std::variant<TestDetection> detection_variant{detection};
//   std::variant<TestDetection> detection_variant_immutable{detection};

//   // Call the functions under test
//   mot::propagate_to_time(detection_variant, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
//   TestDetection result_detection = std::get<TestDetection>(detection_variant);

//   auto result_detection_variant = mot::predict_to_time(
//     detection_variant_immutable, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
//   TestDetection result_detection_immutable = std::get<TestDetection>(result_detection_variant);

//   // Check that function returns expected value
//   // Check detection that was modified in place
//   EXPECT_THAT(result_detection.timestamp, DimensionedNear(time_step, 1e-3));
//   EXPECT_THAT(result_detection.state, CtraStateNear(expected_state, 1e-3));
//   EXPECT_THAT(result_detection.covariance, EigenMatrixNear(expected_covariance, 1e-3));

//   // Check new predicted detection
//   EXPECT_THAT(result_detection_immutable.timestamp, DimensionedNear(time_step, 1e-3));
//   EXPECT_THAT(result_detection_immutable.state, CtraStateNear(expected_state, 1e-3));
//   EXPECT_THAT(result_detection_immutable.covariance, EigenMatrixNear(expected_covariance, 1e-3));
// }

/**
 * Test the temporal alignment for a CTRV Track at one second into the future
 */
TEST(TestTemporalAlignment, CtrvTrack)
{
  using namespace units::literals;

  using TestTrack = mot::Track<mot::CtrvState, mot::CtrvStateCovariance>;

  // Declaring initial state and covariance for the track that will be temporally aligned
  mot::CtrvStateCovariance covariance;
  covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
    0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, -0.0020,
    0.0060, 0.0008, 0.0100, 0.0123;

  TestTrack track{
    .timestamp{units::time::second_t{0}},
    .state{mot::CtrvState{5.7441_m, 1.3800_m, 2.2049_mps, mot::Angle(0.5015_rad), 0.3528_rad_per_s}},
    .covariance{std::move(covariance)},
    .uuid{mot::Uuid{""}}};
  // Expected values
  const mot::CtrvState expected_state{
    7.43226_m, 2.73934_m, 2.2049_mps, mot::Angle(0.8543_rad), 0.3528_rad_per_s};

  mot::CtrvStateCovariance expected_covariance;
  expected_covariance << 0.064633, -0.0671223, 0.00564156, -0.0462884, -0.0239845, -0.0671223,
    0.110445, 0.00623941, 0.0653499, 0.0332627, 0.00564156, 0.00623941, 0.0054, 0.0015, 0.000800001,
    -0.0462884, 0.0653499, 0.0015, 0.0421, 0.0223, -0.0239845, 0.0332627, 0.000800001, 0.0223,
    0.0123;

  // Amount of time the track will be propagated into the future
  auto time_step{1.0_s};

  // Create track variant
  std::variant<TestTrack> track_variant{track};
  std::variant<TestTrack> track_variant_immutable{track};

  // Call the functions under test
  mot::propagate_to_time(track_variant, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
  TestTrack result_track = std::get<TestTrack>(track_variant);

  auto result_track_variant =
    mot::predict_to_time(track_variant_immutable, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
  TestTrack result_track_immutable = std::get<TestTrack>(result_track_variant);

  // Check that function returns expected value
  // Check track that was modified in place
  EXPECT_THAT(result_track.timestamp, DimensionedNear(time_step, 1e-4));
  EXPECT_THAT(result_track.state, CtrvStateNear(expected_state, 1e-4));
  EXPECT_THAT(result_track.covariance, EigenMatrixNear(expected_covariance, 1e-4));

  // Check new predicted detection
  EXPECT_THAT(result_track_immutable.timestamp, DimensionedNear(time_step, 1e-4));
  EXPECT_THAT(result_track_immutable.state, CtrvStateNear(expected_state, 1e-4));
  EXPECT_THAT(result_track_immutable.covariance, EigenMatrixNear(expected_covariance, 1e-4));
}

/**
 * Test the temporal alignment for a CTRA Track at one second into the future
 */
TEST(TestTemporalAlignment, CtraTrack)
{
  using namespace units::literals;

  using TestTrack = mot::Track<mot::CtraState, mot::CtraStateCovariance>;

  // Declaring initial state and covariance for the track that will be temporally aligned
  mot::CtraStateCovariance covariance;
  covariance << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, 0.5, -0.0013, 0.0077, 0.0011, 0.0071,
    0.0060, 0.123, 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.34, -0.0022, 0.0071, 0.0007, 0.0098,
    0.0100, 0.009, -0.0020, 0.0060, 0.0008, 0.0100, 0.0123, 0.0021, 0.5, 0.123, -0.34, 0.009,
    0.0021, -0.8701;

  TestTrack track{
    .timestamp{units::time::second_t{0}},
    .state{mot::CtraState{
      5.7441_m, 1.3800_m, 2.2049_mps, mot::Angle(0.5015_rad), 0.3528_rad_per_s, 1_mps_sq}},
    .covariance{std::move(covariance)},
    .uuid{mot::Uuid{""}}};
  // Expected values
  const mot::CtraState expected_state{
    7.79608_m, 3.06936_m, 3.20490_mps, mot::Angle(0.85430_rad), 0.35280_rad_per_s, 1.00000_mps_sq};
  mot::CtraStateCovariance expected_covariance;
  expected_covariance << 39.64152, 34.73882, 106.16206, -0.06633, -0.03572, 106.28492, 34.73882,
    32.49688, 95.98216, 0.06945, 0.03433, 96.09180, 106.16206, 95.98216, 288.55997, 0.01260,
    0.00290, 288.89456, -0.06633, 0.06945, 0.01260, 0.04210, 0.02230, 0.01110, -0.03572, 0.03433,
    0.00290, 0.02230, 0.01230, 0.00210, 106.28492, 96.09181, 288.89453, 0.01110, 0.00210, 289.23456;

  // Amount of time the track will be propagated into the future
  auto time_step{1.0_s};

  // Create track variant
  std::variant<TestTrack> track_variant{track};
  std::variant<TestTrack> track_variant_immutable{track};

  // Call the functions under test
  mot::propagate_to_time(track_variant, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
  TestTrack result_track = std::get<TestTrack>(track_variant);

  auto result_track_variant =
    mot::predict_to_time(track_variant_immutable, time_step, mot::UnscentedTransform{1.0, 2.0, 1.0});
  TestTrack result_track_immutable = std::get<TestTrack>(result_track_variant);

  // Check that function returns expected value
  // Check track that was modified in place
  EXPECT_THAT(result_track.timestamp, DimensionedNear(time_step, 1e-4));
  EXPECT_THAT(result_track.state, CtraStateNear(expected_state, 1e-4));
  EXPECT_THAT(result_track.covariance, EigenMatrixNear(expected_covariance, 1e-4));

  // Check new predicted detection
  EXPECT_THAT(result_track_immutable.timestamp, DimensionedNear(time_step, 1e-4));
  EXPECT_THAT(result_track_immutable.state, CtraStateNear(expected_state, 1e-4));
  EXPECT_THAT(result_track_immutable.covariance, EigenMatrixNear(expected_covariance, 1e-4));
};
