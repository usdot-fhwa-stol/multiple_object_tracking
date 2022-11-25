#include <algorithm>
#include <gtest/gtest.h>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/unscented_transform.hpp>
#include <cooperative_perception/utils.hpp>

namespace cp = cooperative_perception;

TEST(TestUnscentedTransform, CreateSigmaPoints)
{
  using namespace units::literals;
  constexpr cp::CtrvState state{ 5.7441_m, 1.3800_m, 2.2049_mps, 0.5015_rad, 0.3528_rad_per_s };
  const cp::CtrvStateCovariance covariance{ { 0.0043, -0.0013, 0.0030, -0.0022, -0.0020 },
                                            { -0.0013, 0.0077, 0.0011, 0.0071, 0.0060 },
                                            { 0.0030, 0.0011, 0.0054, 0.0007, 0.0008 },
                                            { -0.0022, 0.0071, 0.0007, 0.0098, 0.0100 },
                                            { -0.0020, 0.0060, 0.0008, 0.0100, 0.0123 } };

  const auto sigma_points{ cp::sampleStateDistribution(state, covariance, 11, 3 - 5) };

  const std::unordered_set<cp::CtrvState> expected_sigma_points{
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, 0.5015_rad, 0.3528_rad_per_s },
    cp::CtrvState{ 5.85768_m, 1.34566_m, 2.28414_mps, 0.44339_rad, 0.299973_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.52806_m, 2.24557_mps, 0.631886_rad, 0.462123_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.29582_mps, 0.516923_rad, 0.376339_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, 0.595227_rad, 0.48417_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, 0.5015_rad, 0.418721_rad_per_s },
    cp::CtrvState{ 5.63052_m, 1.41434_m, 2.12566_mps, 0.55961_rad, 0.405627_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.23194_m, 2.16423_mps, 0.371114_rad, 0.243477_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.11398_mps, 0.486077_rad, 0.329261_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, 0.407773_rad, 0.22143_rad_per_s },
    cp::CtrvState{ 5.7441_m, 1.38_m, 2.2049_mps, 0.5015_rad, 0.286879_rad_per_s },
  };

  const auto is_expected = [&expected_sigma_points](const auto& point) {
    const auto result = std::find_if(std::cbegin(expected_sigma_points), std::cend(expected_sigma_points),
                                     [&point](const auto& expected_point) {
                                       return cp::utils::almostEqual(cp::utils::roundToDecimalPlace(point, 5),
                                                                     cp::utils::roundToDecimalPlace(expected_point, 5));
                                     });

    return result != std::cend(expected_sigma_points);
  };

  std::for_each(std::cbegin(sigma_points), std::cend(sigma_points),
                [&is_expected](const auto& point) { ASSERT_TRUE(is_expected(point)); });
}
