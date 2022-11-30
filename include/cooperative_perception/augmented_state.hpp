#ifndef COOPERATIVE_PERCEPTION_AUGMENTED_STATE_HPP
#define COOPERATIVE_PERCEPTION_AUGMENTED_STATE_HPP

#include <cstddef>
#include <functional>

namespace cooperative_perception
{
template <typename State, typename ProcessNoise>
struct AugmentedState
{
  State state;
  ProcessNoise process_noise;

  static constexpr auto kNumVars{ State::kNumVars + ProcessNoise::kNumVars };

  static auto fromEigenVector(const Eigen::Vector<float, kNumVars>& vec) noexcept -> AugmentedState
  {
    return AugmentedState<State, ProcessNoise>{
      .state{ State::fromEigenVector(Eigen::Vector<float, State::kNumVars>{ vec(Eigen::seqN(0, State::kNumVars)) }) },
      .process_noise{ ProcessNoise::fromEigenVector(
          Eigen::Vector<float, ProcessNoise::kNumVars>{ vec(Eigen::seqN(State::kNumVars, ProcessNoise::kNumVars)) }) }
    };
  }
};

template <typename State, typename ProcessNoise>
inline auto operator==(const AugmentedState<State, ProcessNoise>& lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> bool
{
  return lhs.state == rhs.state && lhs.process_noise == rhs.process_noise;
}

template <typename State, typename ProcessNoise>
inline auto operator+=(AugmentedState<State, ProcessNoise>& lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> AugmentedState<State, ProcessNoise>&
{
  lhs.state += rhs.state;
  lhs.process_noise += rhs.process_noise;
  return lhs;
}

template <typename State, typename ProcessNoise>
inline auto operator+(AugmentedState<State, ProcessNoise> lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> AugmentedState<State, ProcessNoise>
{
  lhs += rhs;
  return lhs;
}

template <typename State, typename ProcessNoise>
inline auto operator-=(AugmentedState<State, ProcessNoise>& lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> AugmentedState<State, ProcessNoise>&
{
  lhs.state -= rhs.state;
  lhs.process_noise -= rhs.process_noise;
  return lhs;
}

template <typename State, typename ProcessNoise>
inline auto operator-(AugmentedState<State, ProcessNoise> lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> AugmentedState<State, ProcessNoise>
{
  lhs -= rhs;
  return lhs;
}

namespace utils
{
template <typename State, typename ProcessNoise>
inline auto almostEqual(const AugmentedState<State, ProcessNoise>& lhs, const AugmentedState<State, ProcessNoise>& rhs)
    -> bool
{
  return almostEqual(lhs.state, rhs.state) && almostEqual(lhs.process_noise, rhs.process_noise);
}

template <typename State, typename ProcessNoise>
inline auto roundToDecimalPlace(const AugmentedState<State, ProcessNoise>& state, std::size_t decimal_place)
    -> AugmentedState<State, ProcessNoise>
{
  return AugmentedState<State, ProcessNoise>{ roundToDecimalPlace(state.state, decimal_place),
                                              roundToDecimalPlace(state.process_noise, decimal_place) };
}
}  // namespace utils

}  // namespace cooperative_perception

namespace std
{
template <typename State, typename ProcessNoise>
struct hash<cooperative_perception::AugmentedState<State, ProcessNoise>>
{
  std::size_t operator()(const cooperative_perception::AugmentedState<State, ProcessNoise>& state) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, std::hash<State>{}(state.state));
    boost::hash_combine(seed, std::hash<ProcessNoise>{}(state.process_noise));

    return seed;
  }
};
}  // namespace std

#endif  // COOPERATIVE_PERCEPTION_AUGMENTED_STATE_HPP
