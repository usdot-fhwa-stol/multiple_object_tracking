#ifndef COOPERATIVE_PERCEPTION_CLUSTERING_HPP
#define COOPERATIVE_PERCEPTION_CLUSTERING_HPP

#include <units.h>

#include <random>
#include <variant>
#include <vector>

#include "cooperative_perception/angle.hpp"
#include "cooperative_perception/ctra_model.hpp"
#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/uuid.hpp"
#include "cooperative_perception/visitor.hpp"

namespace cooperative_perception
{
struct Point
{
  units::length::meter_t position_x{0};
  units::length::meter_t position_y{0};
  units::velocity::meters_per_second_t velocity{0};
  Angle yaw{units::angle::radian_t{0}};
  units::angular_velocity::radians_per_second_t yaw_rate{0};
};

inline auto operator/(Point point, double scalar) -> Point
{
  point.position_x /= scalar;
  point.position_y /= scalar;
  point.velocity /= scalar;
  point.yaw /= scalar;
  point.yaw_rate /= scalar;

  return point;
}

namespace detail
{
struct element_wise_add_fn
{
  template <typename Detection>
  auto operator()(Point point, const Detection & detection) const -> Point
  {
    point.position_x += detection.state.position_x;
    point.position_y += detection.state.position_y;
    point.velocity += detection.state.velocity;
    point.yaw += detection.state.yaw;
    point.yaw_rate += detection.state.yaw_rate;

    return point;
  }

  template <typename... Alternatives>
  auto operator()(Point point, const std::variant<Alternatives...> & detection) const -> Point
  {
    const Visitor visitor{
      [this](const Point & p, const CtrvDetection & d) { return this->operator()(p, d); },
      [this](const Point & p, const CtraDetection & d) { return this->operator()(p, d); },
      [this](const auto &, const auto &) { throw std::runtime_error("cannot add vector types"); }};

    return std::visit(visitor, std::variant<Point>{point}, detection);
  }
};

inline constexpr element_wise_add_fn element_wise_add{};

struct element_wise_subtract_fn
{
  template <typename Detection>
  auto operator()(Point point, const Detection & detection) const -> Point
  {
    point.position_x -= detection.state.position_x;
    point.position_y -= detection.state.position_y;
    point.velocity -= detection.state.velocity;
    point.yaw -= detection.state.yaw;
    point.yaw_rate -= detection.state.yaw_rate;

    return point;
  }

  template <typename... Alternatives>
  auto operator()(Point point, const std::variant<Alternatives...> & detection) const -> Point
  {
    const Visitor visitor{
      [this](const Point & p, const CtrvDetection & d) { return this->operator()(p, d); },
      [this](const Point & p, const CtraDetection & d) { return this->operator()(p, d); },
      [this](const auto &, const auto &) {
        throw std::runtime_error("cannot subtract vector types");
      }};

    return std::visit(visitor, std::variant<Point>{point}, detection);
  }
};

inline constexpr element_wise_subtract_fn element_wise_subtract{};

}  // namespace detail

template <typename Detection>
class Cluster
{
public:
  [[nodiscard]] auto is_empty() const noexcept -> bool { return std::size(detections_) == 0; }
  auto clear() noexcept -> void { detections_.clear(); }

  auto add_detection(const Detection & detection)
  {
    state_sum_ = detail::element_wise_add(state_sum_, detection);
    detections_.emplace(get_uuid(detection), detection);
  }

  auto remove_detection(const Uuid & uuid)
  {
    state_sum_ = detail::element_wise_subtract(state_sum_, detections_.at(uuid));
    detections_.erase(uuid);
  }

  [[nodiscard]] auto get_detections() const noexcept -> const std::unordered_map<Uuid, Detection> &
  {
    return detections_;
  }

  [[nodiscard]] auto get_centroid() const -> Point
  {
    if (detections_.empty()) {
      throw std::runtime_error("cluster is empty");
    }

    return state_sum_ / std::size(detections_);
  }

  Point state_sum_;
  std::unordered_map<Uuid, Detection> detections_;
};

namespace detail
{
struct squared_euclidean_distance_fn
{
  template <typename Detection>
  auto operator()(const Point & point, const Detection & detection) const -> double
  {
    double sum{0};

    sum += std::pow(remove_units(point.position_x - detection.state.position_x), 2);
    sum += std::pow(remove_units(point.position_y - detection.state.position_y), 2);
    sum += std::pow(remove_units(point.velocity - detection.state.velocity), 2);
    sum += std::pow(remove_units(point.yaw.get_angle() - detection.state.yaw.get_angle()), 2);
    sum += std::pow(remove_units(point.yaw_rate - detection.state.yaw_rate), 2);

    return sum;
  }

  template <typename... Alternatives>
  auto operator()(Point point, const std::variant<Alternatives...> & detection) const -> double
  {
    const Visitor visitor{
      [this](const Point & p, const CtrvDetection & d) { return this->operator()(p, d); },
      [this](const Point & p, const CtraDetection & d) { return this->operator()(p, d); },
      [this](const auto &, const auto &) { std::numeric_limits<double>::max(); }};

    return std::visit(visitor, std::variant<Point>{point}, detection);
  }
};

inline constexpr squared_euclidean_distance_fn squared_euclidean_distance{};

template <typename Detection>
auto make_point(const Detection & detection) -> Point
{
  return Point{
    detection.state.position_x, detection.state.position_y, detection.state.velocity,
    detection.state.yaw, detection.state.yaw_rate};
}

template <typename... Alternatives>
auto make_point(const std::variant<Alternatives...> & detection) -> Point
{
  const Visitor visitor{
    [](const CtrvDetection & detection) {
      return Point{
        detection.state.position_x, detection.state.position_y, detection.state.velocity,
        detection.state.yaw, detection.state.yaw_rate};
    },
    [](const CtraDetection & detection) {
      return Point{
        detection.state.position_x, detection.state.position_y, detection.state.velocity,
        detection.state.yaw, detection.state.yaw_rate};
    },
    [](const auto &) { throw std::runtime_error("cannot make point from detection"); }};

  return std::visit(visitor, detection);
}

}  // namespace detail

template <typename Detection>
auto cluster_detections(std::vector<Detection> detections, double distance_threshold)
  -> std::vector<Cluster<Detection>>
{
  std::vector<Cluster<Detection>> clusters;

  while (!detections.empty()) {
    Cluster<Detection> cluster;

    const auto origin_detection{detections.back()};
    const auto origin_point{detail::make_point(origin_detection)};

    cluster.add_detection(origin_detection);
    detections.pop_back();

    for (auto it{std::begin(detections)}; it != std::end(detections);) {
      if (detail::squared_euclidean_distance(origin_point, *it) < distance_threshold) {
        cluster.add_detection(*it);
        detections.erase(it);
      } else {
        ++it;
      }
    }

    clusters.push_back(std::move(cluster));
  }

  return clusters;
}

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CLUSTERING_HPP
