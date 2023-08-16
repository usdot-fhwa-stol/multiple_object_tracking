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

#include <Eigen/Dense>
#include <cooperative_perception/angle.hpp>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/detection.hpp>
#include <cooperative_perception/fusing.hpp>
#include <cooperative_perception/track.hpp>
#include <cooperative_perception/track_matching.hpp>
#include <cooperative_perception/utils.hpp>

namespace cp = cooperative_perception;

/**
 * Test the generate_weight function
 */
TEST(TestFusing, GenerateWeight)
{
  // Declaring initial covariances
  Eigen::Matrix3f covariance1;
  covariance1 << 4, 0, 0, 0, 5, 0, 0, 0, 6;

  Eigen::Matrix3f covariance2;
  covariance2 << 7, 0, 0, 0, 8, 0, 0, 0, 9;

  // Expected values
  const auto expected_weight{0.5895104895104895};

  // Call the function under test
  const auto result_weight{cp::generate_weight(covariance1.inverse(), covariance2.inverse())};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(expected_weight, result_weight));
}

/**
 * Test the compute_covariance_intersection function using purely Eigen matrices and vectors
 */
TEST(TestFusing, ComputeCovarianceIntersectionPureEigen)
{
  // Declaring initial means and covariances
  Eigen::Vector3f mean1(1, 2, 3);
  Eigen::Matrix3f covariance1;
  covariance1 << 4, 0, 0, 0, 5, 0, 0, 0, 6;

  Eigen::Vector3f mean2(4, 5, 6);
  Eigen::Matrix3f covariance2;
  covariance2 << 7, 0, 0, 0, 8, 0, 0, 0, 9;

  // Expected values
  Eigen::Vector3f expected_mean(1.85392169, 2.90970142, 3.95112071);
  Eigen::Matrix3f expected_covariance;
  expected_covariance << 4.85392169, 0, 0, 0, 5.90970142, 0, 0, 0, 6.95112071;

  // Compute inverse of the covariances
  const auto inverse_covariance1{covariance1.inverse()};
  const auto inverse_covariance2{covariance2.inverse()};

  // Generate weight for CI function
  const auto weight{cp::generate_weight(inverse_covariance1, inverse_covariance2)};

  // Call the function under test
  const auto [result_mean, result_covariance]{cp::compute_covariance_intersection(
    mean1, inverse_covariance1, mean2, inverse_covariance2, weight)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(expected_mean, result_mean));
  EXPECT_TRUE(cp::utils::almost_equal(expected_covariance, result_covariance));
}

/**
 * Test fusing CTRV tracks and detections
 */
TEST(TestFusing, CtrvTracksAndDetections)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection3"}}, {"track2", {"detection2"}}, {"track3", {"detection1"}}};

  std::vector<cp::TrackVariant> tracks{
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{1_m, 2_m, 3_mps, cp::Angle(4_rad), 5_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {-0.71124, 0.25895, 0.07334, 0.90743, -0.90324},
        {-0.48288, -0.45900, -0.80263, 0.66087, -0.50648},
        {-0.71805, -0.05412, -0.70058, 0.70838, -0.43610},
        {-0.64773, 0.71184, 0.73016, -0.34521, 0.86354},
        {-0.28654, -0.55006, 0.21601, -0.75024, -0.42627}}},
      .uuid{"track1"}},
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{6_m, 7_m, 8_mps, cp::Angle(2.71681_rad), 10_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {-0.85640, 0.00882, 0.04030, 0.13055, -0.16262},
        {-0.01623, -0.51681, 0.15121, 0.88166, 0.77460},
        {-0.19319, 0.72714, 0.86701, -0.45098, -0.08517},
        {0.75262, 0.22813, 0.55607, -0.84915, 0.47627},
        {-0.94872, -0.01025, 0.30417, 0.98642, 0.78256}}},
      .uuid{"track2"}},
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{11_m, 12_m, 13_mps, cp::Angle(1.43363_rad), 15_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.30846, 0.87769, 0.73339, 0.32553, 0.63853},
        {0.96839, 0.02521, -0.14295, -0.58973, -0.30124},
        {0.51890, 0.33076, -0.90532, -0.26269, 0.16610},
        {-0.08644, 0.42692, 0.33657, -0.64111, 0.61577},
        {-0.46255, -0.91817, 0.11903, 0.26086, 0.36714}}},
      .uuid{"track3"}}};

  std::vector<cp::DetectionVariant> detections{
    cp::CtrvDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        10.72978_m, 12.57061_m, 13.00000_mps, cp::Angle(2.65044_rad), 15.00000_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.88963, 1.35620, 0.95801, -0.73548, -0.62768},
        {1.35620, 3.31145, 1.69035, -1.12470, -1.19955},
        {0.95801, 1.69035, 1.80191, -0.90369, -1.18957},
        {-0.73548, -1.12470, -0.90369, 0.77966, 0.48729},
        {-0.62768, -1.19955, -1.18957, 0.48729, 1.75366}}},
      .uuid{"detection1"}},
    cp::CtrvDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        6.45785_m, 6.65389_m, 8.00000_mps, cp::Angle(1.88242_rad), 10.00000_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.62623, 0.06581, 0.50370, -0.34803, 0.25890},
        {0.06581, 0.98165, -0.23461, 0.44184, 0.03177},
        {0.50370, -0.23461, 1.31776, -0.30712, 0.43955},
        {-0.34803, 0.44184, -0.30712, 2.04658, -0.45696},
        {0.25890, 0.03177, 0.43955, -0.45696, 2.57812}}},
      .uuid{"detection2"}},
    cp::CtrvDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        1.16422_m, 1.65312_m, 3.00000_mps, cp::Angle(2.46081_rad), 5.00000_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {1.83854, 0.34738, 0.41240, -2.36064, 0.46715},
        {0.34738, 1.42875, 0.49256, 1.63664, 0.57979},
        {0.41240, 0.49256, 1.00934, 0.00066, 0.08419},
        {-2.36064, 1.63664, 0.00066, 9.05961, -0.54188},
        {0.46715, 0.57979, 0.08419, -0.54188, 1.17590}}},
      .uuid{"detection3"}}};

  // Expected values
  std::vector<cp::TrackVariant> expected_tracks{
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        1.05749_m, 2.04737_m, 3.03512_mps, cp::Angle(3.86391_rad), 4.90450_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {-0.25576, 0.12667, -0.18088, 0.75883, -0.44233},
        {-0.19123, -0.18001, -0.32491, 0.32673, -0.20177},
        {-0.24708, -0.04796, -0.39811, 0.39406, -0.16223},
        {-0.28802, 0.09621, 0.18434, -0.41292, 0.47857},
        {-0.12247, -0.30455, 0.07302, -0.42005, -0.19994}}},
      .uuid{"track1"}},
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        4.50541_m, 5.23887_m, 1.77684_mps, cp::Angle(0.48295_rad), 4.59700_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.01323, 0.01681, 0.04270, 0.00542, 0.02767},
        {0.01095, 0.02140, 0.04287, 0.00099, 0.01798},
        {0.03273, 0.05499, 0.14541, 0.02476, 0.09505},
        {0.00668, 0.01778, 0.04269, 0.01321, 0.02387},
        {0.03738, 0.05686, 0.14170, 0.01147, 0.07430}}},
      .uuid{"track2"}},
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        8.27323_m, 9.02424_m, 11.10270_mps, cp::Angle(3.92098_rad), 16.39624_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.08726, 0.06046, 0.07614, -0.10960, 1.43314},
        {0.23282, 0.09242, 0.14369, -0.22492, 1.77051},
        {0.13275, 0.11320, -0.01331, -0.11357, 1.11781},
        {-0.06036, -0.03309, -0.03318, 0.03047, -0.88861},
        {-0.17177, -0.28715, -0.11082, 0.13803, -0.58390}}},
      .uuid{"track3"}}};

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
}

/**
 * Test fusing CTRA tracks and detections
 */
TEST(TestFusing, CtraTracksAndDetections)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection3"}}, {"track2", {"detection2"}}, {"track3", {"detection1"}}};

  std::vector<cp::TrackVariant> tracks{
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{1_m, 2_m, 3_mps, cp::Angle(4_rad), 5_rad_per_s, 6_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {0.66016, 0.67280, 0.87234, -0.04035, 0.81610, 0.40492},
        {0.12663, 0.28159, -0.50021, 0.44237, -0.75851, 0.96020},
        {-0.44479, 0.23876, -0.59174, -0.53774, 0.67688, 0.92271},
        {-0.96353, -0.97986, 0.67010, 0.64093, 0.21063, -0.17695},
        {-0.69443, 0.34330, 0.70980, -0.98922, 0.32312, 0.44862},
        {0.31894, -0.46326, 0.41500, -0.07116, -0.66293, -0.81833}}},
      .uuid{"track1"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{7_m, 8_m, 9_mps, cp::Angle(3.71681_rad), 11_rad_per_s, 12_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {-0.49361, -0.65374, 0.92303, 0.15896, -0.47469, 0.18736},
        {-0.27856, 0.44731, 0.29031, 0.04789, -0.15549, 0.75583},
        {0.29388, -0.66435, 0.35708, -0.88374, -0.12222, 0.80023},
        {-0.35242, 0.56761, 0.25037, 0.65496, 0.17750, -0.26927},
        {0.23392, -0.86756, -0.35558, -0.40433, 0.15967, 0.17871},
        {-0.62065, 0.51657, -0.13082, 0.28166, -0.82731, -0.46933}}},
      .uuid{"track2"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{13_m, 14_m, 15_mps, cp::Angle(3.43363_rad), 17_rad_per_s, 18_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {-0.28565, -0.96979, -0.46169, 0.43703, -0.88999, -0.05841},
        {-0.56770, 0.65786, 0.87880, 0.00014, 0.04837, 0.96849},
        {0.18140, 0.92977, 0.36397, 0.08688, -0.65655, -0.44626},
        {0.34891, -0.84795, 0.87197, -0.45626, -0.66138, -0.88974},
        {-0.12757, 0.95772, -0.14295, 0.81724, 0.91834, -0.21633},
        {-0.13922, -0.23413, 0.65277, -0.21056, 0.56191, 0.09530}}},
      .uuid{"track3"}}};

  std::vector<cp::DetectionVariant> detections{
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        12.68893_m, 12.94951_m, 24.0_mps, cp::Angle(4.07965_rad), 17.0_rad_per_s, 18.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692},
        {-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168},
        {-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371},
        {-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153},
        {-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597},
        {0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944}}},
      .uuid{"detection1"}},
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        7.46094_m, 8.81804_m, 15.0_mps, cp::Angle(2.93363_rad), 11.0_rad_per_s, 12.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {1.00910, -0.37564, 0.08011, -1.56304, -0.51999, 0.71663},
        {-0.37564, 3.37509, 0.48220, -1.38163, 0.71814, -0.28335},
        {0.08011, 0.48220, 1.24187, 0.11823, 0.83737, 0.42938},
        {-1.56304, -1.38163, 0.11823, 6.68945, 0.22266, -0.61728},
        {-0.51999, 0.71814, 0.83737, 0.22266, 2.46517, 0.04416},
        {0.71663, -0.28335, 0.42938, -0.61728, 0.04416, 0.86947}}},
      .uuid{"detection2"}},
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        1.29581_m, 1.59029_m, 6.0_mps, cp::Angle(2.18031_rad), 5.0_rad_per_s, 6.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692},
        {-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168},
        {-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371},
        {-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153},
        {-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597},
        {0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944}}},
      .uuid{"detection3"}}};

  // Expected values
  std::vector<cp::TrackVariant> expected_tracks{
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        0.23685_m, 3.65177_m, 6.13595_mps, cp::Angle(3.98704_rad), 5.43771_rad_per_s,
        3.84887_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {1.73333, 0.78889, -0.52390, 0.13104, 1.24063, -0.28648},
        {-0.08517, 0.76190, 0.36308, 0.96765, -1.96332, 1.36932},
        {-0.30737, -0.34690, 0.71649, -0.37899, -0.35356, -0.35271},
        {-1.67884, -1.11482, 0.65232, 3.04036, 0.56680, 1.93504},
        {-1.13169, 0.21219, 0.14250, 1.43570, 4.29881, 0.94258},
        {0.18998, -0.45901, -0.45852, -0.50814, 0.41576, 0.65043}}},
      .uuid{"track1"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        2.70064_m, 21.79559_m, 20.15401_mps, cp::Angle(2.00117_rad), 19.25853_rad_per_s,
        6.63957_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {0.88278, 0.34604, -1.16504, -2.15773, -0.93873, 0.12978},
        {1.48956, -2.51636, 8.77414, 5.05329, 8.81598, 6.18295},
        {1.83499, -2.82735, 6.40386, 1.98804, 6.19200, 4.89924},
        {-1.54311, 3.14147, -4.75623, -1.60261, -4.63417, -3.90667},
        {1.50061, -3.79495, 5.76920, 2.66764, 6.53137, 4.60193},
        {1.03998, 0.19419, -0.92281, -2.31439, -0.58422, 0.64047}}},
      .uuid{"track2"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        8.94162_m, 18.04843_m, 18.98816_mps, cp::Angle(1.32344_rad), 15.33205_rad_per_s,
        23.22585_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {-0.86394, -0.09628, -0.28560, 1.39213, -1.39202, 0.13592},
        {-0.85041, -0.40898, 0.72831, -0.03046, -0.36946, 1.39140},
        {0.41032, 0.89854, 0.28690, -0.05336, -0.15133, -0.70927},
        {-0.10843, -2.61593, 1.56048, -1.06914, 0.50692, 0.03947},
        {0.16566, 1.54753, -0.42328, 1.06169, 1.29183, -0.76139},
        {-0.43168, -1.63921, 0.89318, -0.64818, 0.93386, 0.67674}}},
      .uuid{"track3"}}};

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
}

/**
 * Test fusing a mixed vector of CTRV and CTRA tracks and detections
 */
TEST(TestFusing, MixedTracksAndDetections)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection3"}}, {"track2", {"detection2"}}, {"track3", {"detection1"}}};

  std::vector<cp::TrackVariant> tracks{
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{1_m, 2_m, 3_mps, cp::Angle(4_rad), 5_rad_per_s, 6_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {0.66016, 0.67280, 0.87234, -0.04035, 0.81610, 0.40492},
        {0.12663, 0.28159, -0.50021, 0.44237, -0.75851, 0.96020},
        {-0.44479, 0.23876, -0.59174, -0.53774, 0.67688, 0.92271},
        {-0.96353, -0.97986, 0.67010, 0.64093, 0.21063, -0.17695},
        {-0.69443, 0.34330, 0.70980, -0.98922, 0.32312, 0.44862},
        {0.31894, -0.46326, 0.41500, -0.07116, -0.66293, -0.81833}}},
      .uuid{"track1"}},
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{6_m, 7_m, 8_mps, cp::Angle(2.71681_rad), 10_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {-0.85640, 0.00882, 0.04030, 0.13055, -0.16262},
        {-0.01623, -0.51681, 0.15121, 0.88166, 0.77460},
        {-0.19319, 0.72714, 0.86701, -0.45098, -0.08517},
        {0.75262, 0.22813, 0.55607, -0.84915, 0.47627},
        {-0.94872, -0.01025, 0.30417, 0.98642, 0.78256}}},
      .uuid{"track2"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{13_m, 14_m, 15_mps, cp::Angle(3.43363_rad), 17_rad_per_s, 18_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {-0.28565, -0.96979, -0.46169, 0.43703, -0.88999, -0.05841},
        {-0.56770, 0.65786, 0.87880, 0.00014, 0.04837, 0.96849},
        {0.18140, 0.92977, 0.36397, 0.08688, -0.65655, -0.44626},
        {0.34891, -0.84795, 0.87197, -0.45626, -0.66138, -0.88974},
        {-0.12757, 0.95772, -0.14295, 0.81724, 0.91834, -0.21633},
        {-0.13922, -0.23413, 0.65277, -0.21056, 0.56191, 0.09530}}},
      .uuid{"track3"}}};

  std::vector<cp::DetectionVariant> detections{
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        12.68893_m, 12.94951_m, 24.0_mps, cp::Angle(4.07965_rad), 17.0_rad_per_s, 18.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692},
        {-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168},
        {-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371},
        {-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153},
        {-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597},
        {0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944}}},
      .uuid{"detection1"}},
    cp::CtrvDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        6.45785_m, 6.65389_m, 8.00000_mps, cp::Angle(1.88242_rad), 10.00000_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.62623, 0.06581, 0.50370, -0.34803, 0.25890},
        {0.06581, 0.98165, -0.23461, 0.44184, 0.03177},
        {0.50370, -0.23461, 1.31776, -0.30712, 0.43955},
        {-0.34803, 0.44184, -0.30712, 2.04658, -0.45696},
        {0.25890, 0.03177, 0.43955, -0.45696, 2.57812}}},
      .uuid{"detection2"}},
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        1.29581_m, 1.59029_m, 6.0_mps, cp::Angle(2.18031_rad), 5.0_rad_per_s, 6.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692},
        {-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168},
        {-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371},
        {-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153},
        {-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597},
        {0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944}}},
      .uuid{"detection3"}}};

  // Expected values
  std::vector<cp::TrackVariant> expected_tracks{
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        0.23685_m, 3.65177_m, 6.13595_mps, cp::Angle(3.98704_rad), 5.43771_rad_per_s,
        3.84887_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {1.73333, 0.78889, -0.52390, 0.13104, 1.24063, -0.28648},
        {-0.08517, 0.76190, 0.36308, 0.96765, -1.96332, 1.36932},
        {-0.30737, -0.34690, 0.71649, -0.37899, -0.35356, -0.35271},
        {-1.67884, -1.11482, 0.65232, 3.04036, 0.56680, 1.93504},
        {-1.13169, 0.21219, 0.14250, 1.43570, 4.29881, 0.94258},
        {0.18998, -0.45901, -0.45852, -0.50814, 0.41576, 0.65043}}},
      .uuid{"track1"}},
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        4.50541_m, 5.23887_m, 1.77684_mps, cp::Angle(0.48295_rad), 4.59700_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.01323, 0.01681, 0.04270, 0.00542, 0.02767},
        {0.01095, 0.02140, 0.04287, 0.00099, 0.01798},
        {0.03273, 0.05499, 0.14541, 0.02476, 0.09505},
        {0.00668, 0.01778, 0.04269, 0.01321, 0.02387},
        {0.03738, 0.05686, 0.14170, 0.01147, 0.07430}}},
      .uuid{"track2"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        8.94162_m, 18.04843_m, 18.98816_mps, cp::Angle(1.32344_rad), 15.33205_rad_per_s,
        23.22585_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {-0.86394, -0.09628, -0.28560, 1.39213, -1.39202, 0.13592},
        {-0.85041, -0.40898, 0.72831, -0.03046, -0.36946, 1.39140},
        {0.41032, 0.89854, 0.28690, -0.05336, -0.15133, -0.70927},
        {-0.10843, -2.61593, 1.56048, -1.06914, 0.50692, 0.03947},
        {0.16566, 1.54753, -0.42328, 1.06169, 1.29183, -0.76139},
        {-0.43168, -1.63921, 0.89318, -0.64818, 0.93386, 0.67674}}},
      .uuid{"track3"}}};

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
}

/**
 * Test fusing when no matching uuids are found
 */
TEST(TestFusing, UnmatchedAssociations)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection4"}}, {"track2", {"detection5"}}, {"track3", {"detection6"}}};

  std::vector<cp::TrackVariant> tracks{
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{1_m, 2_m, 3_mps, cp::Angle(4_rad), 5_rad_per_s, 6_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {0.66016, 0.67280, 0.87234, -0.04035, 0.81610, 0.40492},
        {0.12663, 0.28159, -0.50021, 0.44237, -0.75851, 0.96020},
        {-0.44479, 0.23876, -0.59174, -0.53774, 0.67688, 0.92271},
        {-0.96353, -0.97986, 0.67010, 0.64093, 0.21063, -0.17695},
        {-0.69443, 0.34330, 0.70980, -0.98922, 0.32312, 0.44862},
        {0.31894, -0.46326, 0.41500, -0.07116, -0.66293, -0.81833}}},
      .uuid{"track1"}},
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{6_m, 7_m, 8_mps, cp::Angle(2.71681_rad), 10_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {-0.85640, 0.00882, 0.04030, 0.13055, -0.16262},
        {-0.01623, -0.51681, 0.15121, 0.88166, 0.77460},
        {-0.19319, 0.72714, 0.86701, -0.45098, -0.08517},
        {0.75262, 0.22813, 0.55607, -0.84915, 0.47627},
        {-0.94872, -0.01025, 0.30417, 0.98642, 0.78256}}},
      .uuid{"track2"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{13_m, 14_m, 15_mps, cp::Angle(3.43363_rad), 17_rad_per_s, 18_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {-0.28565, -0.96979, -0.46169, 0.43703, -0.88999, -0.05841},
        {-0.56770, 0.65786, 0.87880, 0.00014, 0.04837, 0.96849},
        {0.18140, 0.92977, 0.36397, 0.08688, -0.65655, -0.44626},
        {0.34891, -0.84795, 0.87197, -0.45626, -0.66138, -0.88974},
        {-0.12757, 0.95772, -0.14295, 0.81724, 0.91834, -0.21633},
        {-0.13922, -0.23413, 0.65277, -0.21056, 0.56191, 0.09530}}},
      .uuid{"track3"}}};

  std::vector<cp::DetectionVariant> detections{
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        12.68893_m, 12.94951_m, 24.0_mps, cp::Angle(4.07965_rad), 17.0_rad_per_s, 18.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692},
        {-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168},
        {-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371},
        {-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153},
        {-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597},
        {0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944}}},
      .uuid{"detection1"}},
    cp::CtrvDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        6.45785_m, 6.65389_m, 8.00000_mps, cp::Angle(1.88242_rad), 10.00000_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.62623, 0.06581, 0.50370, -0.34803, 0.25890},
        {0.06581, 0.98165, -0.23461, 0.44184, 0.03177},
        {0.50370, -0.23461, 1.31776, -0.30712, 0.43955},
        {-0.34803, 0.44184, -0.30712, 2.04658, -0.45696},
        {0.25890, 0.03177, 0.43955, -0.45696, 2.57812}}},
      .uuid{"detection2"}},
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        1.29581_m, 1.59029_m, 6.0_mps, cp::Angle(2.18031_rad), 5.0_rad_per_s, 6.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692},
        {-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168},
        {-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371},
        {-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153},
        {-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597},
        {0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944}}},
      .uuid{"detection3"}}};

  // Expected values
  std::vector<cp::TrackVariant> expected_tracks;

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
}

/**
 * Test fusing when only a few associations are matched
 */
TEST(TestFusing, PartialMatchedAssociations)
{
  using namespace units::literals;

  // Declaring initial values
  cp::AssociationMap associations{
    {"track1", {"detection4"}}, {"track2", {"detection2"}}, {"track3", {"detection1"}}};

  std::vector<cp::TrackVariant> tracks{
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{1_m, 2_m, 3_mps, cp::Angle(4_rad), 5_rad_per_s, 6_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {0.66016, 0.67280, 0.87234, -0.04035, 0.81610, 0.40492},
        {0.12663, 0.28159, -0.50021, 0.44237, -0.75851, 0.96020},
        {-0.44479, 0.23876, -0.59174, -0.53774, 0.67688, 0.92271},
        {-0.96353, -0.97986, 0.67010, 0.64093, 0.21063, -0.17695},
        {-0.69443, 0.34330, 0.70980, -0.98922, 0.32312, 0.44862},
        {0.31894, -0.46326, 0.41500, -0.07116, -0.66293, -0.81833}}},
      .uuid{"track1"}},
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{6_m, 7_m, 8_mps, cp::Angle(2.71681_rad), 10_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {-0.85640, 0.00882, 0.04030, 0.13055, -0.16262},
        {-0.01623, -0.51681, 0.15121, 0.88166, 0.77460},
        {-0.19319, 0.72714, 0.86701, -0.45098, -0.08517},
        {0.75262, 0.22813, 0.55607, -0.84915, 0.47627},
        {-0.94872, -0.01025, 0.30417, 0.98642, 0.78256}}},
      .uuid{"track2"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{13_m, 14_m, 15_mps, cp::Angle(3.43363_rad), 17_rad_per_s, 18_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {-0.28565, -0.96979, -0.46169, 0.43703, -0.88999, -0.05841},
        {-0.56770, 0.65786, 0.87880, 0.00014, 0.04837, 0.96849},
        {0.18140, 0.92977, 0.36397, 0.08688, -0.65655, -0.44626},
        {0.34891, -0.84795, 0.87197, -0.45626, -0.66138, -0.88974},
        {-0.12757, 0.95772, -0.14295, 0.81724, 0.91834, -0.21633},
        {-0.13922, -0.23413, 0.65277, -0.21056, 0.56191, 0.09530}}},
      .uuid{"track3"}}};

  std::vector<cp::DetectionVariant> detections{
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        12.68893_m, 12.94951_m, 24.0_mps, cp::Angle(4.07965_rad), 17.0_rad_per_s, 18.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692},
        {-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168},
        {-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371},
        {-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153},
        {-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597},
        {0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944}}},
      .uuid{"detection1"}},
    cp::CtrvDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        6.45785_m, 6.65389_m, 8.00000_mps, cp::Angle(1.88242_rad), 10.00000_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.62623, 0.06581, 0.50370, -0.34803, 0.25890},
        {0.06581, 0.98165, -0.23461, 0.44184, 0.03177},
        {0.50370, -0.23461, 1.31776, -0.30712, 0.43955},
        {-0.34803, 0.44184, -0.30712, 2.04658, -0.45696},
        {0.25890, 0.03177, 0.43955, -0.45696, 2.57812}}},
      .uuid{"detection2"}},
    cp::CtraDetection{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        1.29581_m, 1.59029_m, 6.0_mps, cp::Angle(2.18031_rad), 5.0_rad_per_s, 6.0_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692},
        {-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168},
        {-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371},
        {-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153},
        {-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597},
        {0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944}}},
      .uuid{"detection3"}}};

  // Expected values
  std::vector<cp::TrackVariant> expected_tracks{
    cp::CtrvTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtrvState{
        4.50541_m, 5.23887_m, 1.77684_mps, cp::Angle(0.48295_rad), 4.59700_rad_per_s}},
      .covariance{cp::CtrvStateCovariance{
        {0.01323, 0.01681, 0.04270, 0.00542, 0.02767},
        {0.01095, 0.02140, 0.04287, 0.00099, 0.01798},
        {0.03273, 0.05499, 0.14541, 0.02476, 0.09505},
        {0.00668, 0.01778, 0.04269, 0.01321, 0.02387},
        {0.03738, 0.05686, 0.14170, 0.01147, 0.07430}}},
      .uuid{"track2"}},
    cp::CtraTrack{
      .timestamp{units::time::second_t{0}},
      .state{cp::CtraState{
        8.94162_m, 18.04843_m, 18.98816_mps, cp::Angle(1.32344_rad), 15.33205_rad_per_s,
        23.22585_mps_sq}},
      .covariance{cp::CtraStateCovariance{
        {-0.86394, -0.09628, -0.28560, 1.39213, -1.39202, 0.13592},
        {-0.85041, -0.40898, 0.72831, -0.03046, -0.36946, 1.39140},
        {0.41032, 0.89854, 0.28690, -0.05336, -0.15133, -0.70927},
        {-0.10843, -2.61593, 1.56048, -1.06914, 0.50692, 0.03947},
        {0.16566, 1.54753, -0.42328, 1.06169, 1.29183, -0.76139},
        {-0.43168, -1.63921, 0.89318, -0.64818, 0.93386, 0.67674}}},
      .uuid{"track3"}}};

  // Call the functions under test
  const auto result_tracks{
    cp::fuse_associations(associations, tracks, detections, cp::covariance_intersection_visitor)};

  // Check that function returns expected value
  EXPECT_TRUE(cp::utils::almost_equal(result_tracks, expected_tracks));
};
