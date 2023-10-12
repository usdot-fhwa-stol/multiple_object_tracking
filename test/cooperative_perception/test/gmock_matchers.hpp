#ifndef COOPERATIVE_PERCEPTION_TEST_MATCHERS_HPP
#define COOPERATIVE_PERCEPTION_TEST_MATCHERS_HPP

#include <gmock/gmock.h>

#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/units.hpp>
#include <cooperative_perception/visitor.hpp>
#include <iostream>
#include <type_traits>

MATCHER(AlwaysFalse, "") { return false; }

namespace detail
{
template <typename Unit>
class DimensionedNearMatcher
{
public:
  using is_gtest_matcher = void;

  explicit DimensionedNearMatcher(Unit expected, double max_abs_error)
  : expected_{expected}, max_abs_error_{max_abs_error}
  {
  }

  auto MatchAndExplain(Unit value, std::ostream * os) const -> bool
  {
    const auto absolute_difference{units::math::abs(value - expected_)};

    if (os != nullptr) {
      *os << "absolute difference: " << absolute_difference;
    }

    return cooperative_perception::remove_units(absolute_difference) <= max_abs_error_;
  }

  auto DescribeTo(std::ostream * os) const -> void
  {
    if (os != nullptr) {
      *os << "is within " << max_abs_error_ << " of expected value";
    }
  }

  auto DescribeNegationTo(std::ostream * os) const -> void
  {
    if (os != nullptr) {
      *os << "isn't within " << max_abs_error_ << " of expected value";
    }
  }

private:
  Unit expected_{};
  double max_abs_error_{0.0};
};

}  // namespace detail

template <typename Unit>
auto DimensionedNear(units::unit_t<Unit> expected, double max_abs_error)
  -> ::testing::Matcher<units::unit_t<Unit>>
{
  return detail::DimensionedNearMatcher<units::unit_t<Unit>>{expected, max_abs_error};
}

MATCHER_P2(AngleNear, angle, max_abs_error, "")
{
  return cooperative_perception::remove_units(
           units::math::abs((arg.get_angle() - angle.get_angle()))) <= max_abs_error;
}

MATCHER_P2(EigenMatrixNear, matrix, max_abs_error, "")
{
  using ::testing::AllOf;
  using ::testing::Pointwise;
  using ::testing::Value;

  if (std::size(arg) != std::size(matrix)) {
    return false;
  }

  for (auto row{0U}; row < matrix.rows(); ++row) {
    for (auto col{0U}; col < matrix.cols(); ++col) {
      if (std::abs(arg(row, col) - matrix(row, col)) > max_abs_error) {
        return false;
      }
    }
  }

  return true;
}

inline constexpr auto CtrvStateNear =
  [](const cooperative_perception::CtrvState & state, double max_abs_error) {
    using ::testing::AllOf;
    using ::testing::Field;

    namespace cp = cooperative_perception;

    return AllOf(
      Field(&cp::CtrvState::position_x, DimensionedNear(state.position_x, max_abs_error)),
      Field(&cp::CtrvState::position_y, DimensionedNear(state.position_y, max_abs_error)),
      Field(&cp::CtrvState::velocity, DimensionedNear(state.velocity, max_abs_error)),
      Field(&cp::CtrvState::yaw, AngleNear(state.yaw, max_abs_error)),
      Field(&cp::CtrvState::yaw_rate, DimensionedNear(state.yaw_rate, max_abs_error)));
  };

inline constexpr auto CtraStateNear =
  [](const cooperative_perception::CtraState & state, double max_abs_error) {
    using ::testing::AllOf;
    using ::testing::Field;

    namespace cp = cooperative_perception;

    return AllOf(
      Field(&cp::CtraState::position_x, DimensionedNear(state.position_x, max_abs_error)),
      Field(&cp::CtraState::position_y, DimensionedNear(state.position_y, max_abs_error)),
      Field(&cp::CtraState::velocity, DimensionedNear(state.velocity, max_abs_error)),
      Field(&cp::CtraState::yaw, AngleNear(state.yaw, max_abs_error)),
      Field(&cp::CtraState::yaw_rate, DimensionedNear(state.yaw_rate, max_abs_error)),
      Field(&cp::CtraState::acceleration, DimensionedNear(state.acceleration, max_abs_error)));
  };

inline constexpr auto CtrvTrackNear =
  [](const cooperative_perception::CtrvTrack & track, double max_abs_error) {
    using ::testing::AllOf;
    using ::testing::Eq;
    using ::testing::Field;

    namespace cp = cooperative_perception;

    return AllOf(
      Field(&cp::CtrvTrack::timestamp, DimensionedNear(track.timestamp, max_abs_error)),
      Field(&cp::CtrvTrack::state, CtrvStateNear(track.state, max_abs_error)),
      Field(&cp::CtrvTrack::covariance, EigenMatrixNear(track.covariance, max_abs_error)),
      Field(&cp::CtrvTrack::uuid, Eq(track.uuid)));
  };

inline constexpr auto CtraTrackNear =
  [](const cooperative_perception::CtraTrack & track, double max_abs_error) {
    using ::testing::AllOf;
    using ::testing::Eq;
    using ::testing::Field;

    namespace cp = cooperative_perception;

    return AllOf(
      Field(&cp::CtraTrack::timestamp, DimensionedNear(track.timestamp, max_abs_error)),
      Field(&cp::CtraTrack::state, CtraStateNear(track.state, max_abs_error)),
      Field(&cp::CtraTrack::covariance, EigenMatrixNear(track.covariance, max_abs_error)),
      Field(&cp::CtraTrack::uuid, Eq(track.uuid)));
  };

namespace detail
{
class PointwiseTrackNearMatcher
{
public:
  using is_gtest_matcher = void;

  explicit PointwiseTrackNearMatcher(double max_abs_error) : max_abs_error_{max_abs_error} {}

  auto MatchAndExplain(
    const std::tuple<
      const cooperative_perception::CtrvTrack &, const cooperative_perception::CtrvTrack &> & arg,
    std::ostream *) const -> bool
  {
    using ::testing::Matches;

    const auto actual{std::get<0>(arg)};
    const auto expected{std::get<1>(arg)};

    return Matches(CtrvTrackNear(expected, max_abs_error_))(actual);
  }

  auto MatchAndExplain(
    const std::tuple<
      const cooperative_perception::CtraTrack &, const cooperative_perception::CtraTrack &> & arg,
    std::ostream *) const -> bool
  {
    using ::testing::Matches;

    const auto actual{std::get<0>(arg)};
    const auto expected{std::get<1>(arg)};

    return Matches(CtraTrackNear(expected, max_abs_error_))(actual);
  }

  template <typename... Alternatives>
  auto MatchAndExplain(
    const std::tuple<const std::variant<Alternatives...> &, const std::variant<Alternatives...> &> &
      arg,
    std::ostream *) const -> bool
  {
    using cooperative_perception::CtraTrack;
    using cooperative_perception::CtrvTrack;

    const cooperative_perception::Visitor visitor{
      [this](const CtrvTrack & actual, const CtrvTrack & expected) {
        return Matches(CtrvTrackNear(expected, max_abs_error_))(actual);
      },
      [this](const CtraTrack & actual, const CtraTrack & expected) {
        return Matches(CtraTrackNear(expected, max_abs_error_))(actual);
      },
      [](const auto &, const auto &) { return false; }};

    return std::visit(visitor, std::get<0>(arg), std::get<1>(arg));
  }

  auto DescribeTo(std::ostream * os) const -> void
  {
    if (os != nullptr) {
      *os << "is within " << max_abs_error_ << " of expected value";
    }
  }

  auto DescribeNegationTo(std::ostream * os) const -> void
  {
    if (os != nullptr) {
      *os << "isn't within " << max_abs_error_ << " of expected value";
    }
  }

private:
  double max_abs_error_{0.0};
};

}  // namespace detail

inline auto PointwiseTrackNear(double max_abs_error) noexcept -> detail::PointwiseTrackNearMatcher
{
  return detail::PointwiseTrackNearMatcher{max_abs_error};
}

namespace cooperative_perception
{
inline auto PrintTo(const CtrvTrack & track, std::ostream * os) noexcept -> void
{
  *os << "\nCtrvTrack:\n";
  *os << "    timestamp: " << track.timestamp << '\n';
  *os << "    state: \n";
  *os << "      position_x: " << track.state.position_x << '\n';
  *os << "      position_y: " << track.state.position_y << '\n';
  *os << "      velocity: " << track.state.velocity << '\n';
  *os << "      yaw: " << track.state.yaw.get_angle() << '\n';
  *os << "      yaw_rate: " << track.state.yaw_rate << '\n';
  *os << "    covariance: " << track.covariance << '\n';
  *os << "    uuid: " << track.uuid.value() << '\n';
}

inline auto PrintTo(const CtraTrack & track, std::ostream * os) noexcept -> void
{
  *os << "\nCtraTrack:\n";
  *os << "    timestamp: " << track.timestamp << '\n';
  *os << "    state: \n";
  *os << "      position_x: " << track.state.position_x << '\n';
  *os << "      position_y: " << track.state.position_y << '\n';
  *os << "      velocity: " << track.state.velocity << '\n';
  *os << "      yaw: " << track.state.yaw.get_angle() << '\n';
  *os << "      yaw_rate: " << track.state.yaw_rate << '\n';
  *os << "      acceleration: " << track.state.acceleration << '\n';
  *os << "    covariance: " << track.covariance << '\n';
  *os << "    uuid: " << track.uuid.value() << '\n';
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_TEST_MATCHERS_HPP
