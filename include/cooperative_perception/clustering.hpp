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
  auto is_empty() const noexcept -> bool { return std::size(detections_) == 0; }
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

  auto get_detections() const noexcept -> const std::unordered_map<Uuid, Detection> &
  {
    return detections_;
  }

  auto get_centroid() const -> Point
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
auto get_closest_mean_index(const std::vector<Point> & means, const Detection & detection)
  -> std::size_t
{
  std::size_t best_index{0U};
  double best_distance{std::numeric_limits<double>::max()};

  for (auto i{0U}; i < std::size(means); ++i) {
    const auto dist{squared_euclidean_distance(means.at(i), detection)};

    if (dist < best_distance) {
      best_distance = dist;
      best_index = i;
    }
  }

  return best_index;
}

auto initialize_means(std::size_t num_means)
{
  std::random_device device;
  std::default_random_engine engine{device()};
  std::uniform_real_distribution distribution;

  std::vector<Point> points;
  for (auto i{0U}; i < num_means; ++i) {
    Point point;

    point.position_x = units::length::meter_t{distribution(engine)};
    point.position_y = units::length::meter_t{distribution(engine)};
    point.velocity = units::velocity::meters_per_second_t{distribution(engine)};
    point.yaw.set_angle(units::angle::radian_t{distribution(engine)});
    point.yaw_rate = units::angular_velocity::radians_per_second_t{distribution(engine)};

    points.push_back(std::move(point));
  }

  return points;
}

template <typename Detection>
auto initialize_means(std::size_t num_means, const std::vector<Detection> & detections)
{
  std::vector<Point> means(num_means);
  std::vector<Cluster<Detection>> clusters(num_means);

  for (auto c{0U}; c < std::size(clusters); ++c) {
    clusters.at(c).add_detection(detections.at(c));
  }

  for (auto i{0U}; i < std::size(clusters); ++i) {
    means.at(i) = clusters.at(i).get_centroid();
    clusters.at(i).clear();
  }

  return means;
}

template <typename Detection>
auto get_cluster_assignments(const std::vector<Cluster<Detection>> & clusters)
{
  std::vector<std::vector<Uuid>> assignments;

  for (const auto & cluster : clusters) {
    std::vector<Uuid> uuids;
    for (const auto & [uuid, detection] : cluster.get_detections()) {
      uuids.push_back(uuid);
    }

    assignments.push_back(std::move(uuids));
  }

  return assignments;
}

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

// template <typename Detection>
// auto cluster_detections(const std::vector<Detection> & detections, std::size_t num_clusters)
//   -> std::vector<Cluster<Detection>>
// {
//   if (num_clusters > std::size(detections)) {
//     throw std::logic_error(
//       "number of clusters ('" + std::to_string(num_clusters) +
//       "') must be less than or equal to number of detections ('" +
//       std::to_string(std::size(detections)) + "')");
//   }

//   std::vector<Cluster<Detection>> clusters(num_clusters);
//   auto means{detail::initialize_means(num_clusters, detections)};
//   auto previous_assignments{detail::get_cluster_assignments(clusters)};

//   do {
//     previous_assignments = detail::get_cluster_assignments(clusters);

//     for (auto & cluster : clusters) {
//       cluster.clear();
//     }

//     for (const auto & detection : detections) {
//       clusters.at(detail::get_closest_mean_index(means, detection)).add_detection(detection);
//     }

//     for (auto i{0U}; i < std::size(clusters); ++i) {
//       means.at(i) = clusters.at(i).get_centroid();
//     }
//   } while (detail::get_cluster_assignments(clusters) != previous_assignments);

//   return clusters;
// }

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_CLUSTERING_HPP
