#include <algorithm>
#include <unordered_set>
#include "cooperative_perception/unscented_transform.hpp"
#include <boost/container_hash/hash.hpp>

namespace cooperative_perception
{
auto sampleStateDistribution(const CtrvState& state, const CtrvStateCovariance covariance, std::size_t num_points,
                             float lambda) -> std::unordered_set<CtrvState>
{
  std::unordered_set<CtrvState> sigma_pts{ state };

  const CtrvStateCovariance covariance_sqrt{ covariance.llt().matrixL() };
  for (const auto& column : covariance_sqrt.colwise())
  {
    using namespace units::literals;
    const auto result{ std::sqrt(covariance.rows() + lambda) * column };
    const CtrvState result_state{ .position_x = units::length::meter_t{ result(0) },
                                  .position_y = units::length::meter_t{ result(1) },
                                  .velocity = units::velocity::meters_per_second_t{ result(2) },
                                  .yaw = units::angle::radian_t{ result(3) },
                                  .yaw_rate = units::angular_velocity::radians_per_second_t{ result(4) } };
    sigma_pts.insert(state + result_state);
    sigma_pts.insert(state - result_state);
  }

  return sigma_pts;
}

}  // namespace cooperative_perception
