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

#ifndef COOPERATIVE_PERCEPTION_JSON_PARSING_HPP
#define COOPERATIVE_PERCEPTION_JSON_PARSING_HPP

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "cooperative_perception/ctra_model.hpp"
#include "cooperative_perception/ctrv_model.hpp"
#include "cooperative_perception/uuid.hpp"

template <>
struct nlohmann::adl_serializer<cooperative_perception::Uuid>
{
  static auto to_json(json & j, const cooperative_perception::Uuid & uuid) -> void {}

  static auto from_json(const json & j, cooperative_perception::Uuid & uuid) -> void
  {
    uuid = cooperative_perception::Uuid{j.template get<std::string>()};
  }
};

template <>
struct nlohmann::adl_serializer<cooperative_perception::CtrvState>
{
  static auto to_json(json & j, const cooperative_perception::CtrvState & state) -> void {}

  static auto from_json(const json & j, cooperative_perception::CtrvState & state) -> void
  {
    state.position_x = units::length::meter_t{j.at("position_x_m").template get<double>()};
    state.position_y = units::length::meter_t{j.at("position_y_m")};
    state.velocity = units::velocity::meters_per_second_t{j.at("velocity_mps")};
    state.yaw = units::angle::radian_t{j.at("yaw_rad")};
    state.yaw_rate = units::angular_velocity::radians_per_second_t{j.at("yaw_rate_rad_per_s")};
  }
};

template <>
struct nlohmann::adl_serializer<cooperative_perception::CtraState>
{
  static auto to_json(json & j, const cooperative_perception::CtraState & state) -> void {}

  static auto from_json(const json & j, cooperative_perception::CtraState & state) -> void
  {
    state.position_x = units::length::meter_t{j.at("position_x_m").template get<double>()};
    state.position_y = units::length::meter_t{j.at("position_y_m")};
    state.velocity = units::velocity::meters_per_second_t{j.at("velocity_mps")};
    state.yaw = units::angle::radian_t{j.at("yaw_rad")};
    state.yaw_rate = units::angular_velocity::radians_per_second_t{j.at("yaw_rate_rad_per_s")};
    state.acceleration =
      units::acceleration::meters_per_second_squared_t(j.at("acceleration_mps_sq"));
  }
};

template <>
struct nlohmann::adl_serializer<cooperative_perception::CtrvStateCovariance>
{
  static auto to_json(json & j, const cooperative_perception::CtrvStateCovariance & covariance)
    -> void
  {
  }

  static auto from_json(const json & j, cooperative_perception::CtrvStateCovariance & covariance)
    -> void
  {
    auto data = j.template get<std::vector<float>>();
    covariance = Eigen::Map<Eigen::Matrix<float, 5, 5>>(data.data()).transpose();
  }
};

template <>
struct nlohmann::adl_serializer<cooperative_perception::CtraStateCovariance>
{
  static auto to_json(json & j, const cooperative_perception::CtraStateCovariance & covariance)
    -> void
  {
  }

  static auto from_json(const json & j, cooperative_perception::CtraStateCovariance & covariance)
    -> void
  {
    auto data = j.template get<std::vector<float>>();
    covariance = Eigen::Map<Eigen::Matrix<float, 6, 6>>(data.data()).transpose();
  }
};

template <>
struct nlohmann::adl_serializer<cooperative_perception::CtrvDetection>
{
  static auto to_json(json & j, const cooperative_perception::CtrvDetection & detection) -> void {}

  static auto from_json(const json & j, cooperative_perception::CtrvDetection & detection) -> void
  {
    detection.timestamp = units::time::second_t{j.at("timestamp_sec").template get<double>()};
    j.at("state").get_to(detection.state);
    j.at("covariance").get_to(detection.covariance);
    j.at("uuid").get_to(detection.uuid);
  }
};

template <>
struct nlohmann::adl_serializer<cooperative_perception::CtraDetection>
{
  static auto to_json(json & j, const cooperative_perception::CtraDetection & detection) -> void {}

  static auto from_json(const json & j, cooperative_perception::CtraDetection & detection) -> void
  {
    detection.timestamp = units::time::second_t{j.at("timestamp_sec").template get<double>()};
    j.at("state").get_to(detection.state);
    j.at("covariance").get_to(detection.covariance);
    j.at("uuid").get_to(detection.uuid);
  }
};

template <>
struct nlohmann::adl_serializer<cooperative_perception::CtrvTrack>
{
  static auto to_json(json & j, const cooperative_perception::CtrvTrack & track) -> void {}

  static auto from_json(const json & j, cooperative_perception::CtrvTrack & track) -> void
  {
    track.timestamp = units::time::second_t{j.at("timestamp_sec").template get<double>()};
    j.at("state").get_to(track.state);
    j.at("covariance").get_to(track.covariance);
    j.at("uuid").get_to(track.uuid);
  }
};

template <>
struct nlohmann::adl_serializer<cooperative_perception::CtraTrack>
{
  static auto to_json(json & j, const cooperative_perception::CtraTrack & track) -> void {}

  static auto from_json(const json & j, cooperative_perception::CtraTrack & track) -> void
  {
    track.timestamp = units::time::second_t{j.at("timestamp_sec").template get<double>()};
    j.at("state").get_to(track.state);
    j.at("covariance").get_to(track.covariance);
    j.at("uuid").get_to(track.uuid);
  }
};

namespace cooperative_perception
{
namespace customization
{
template <typename T>
struct do_detections_from_json_file;

template <>
struct do_detections_from_json_file<CtrvDetection>
{
  static auto _(std::ifstream & file)
  {
    const auto data = nlohmann::json::parse(file);
    std::vector<CtrvDetection> detections;

    for (const auto & detection : data.at("detections")) {
      const auto motion_model{detection.at("motion_model").template get<std::string>()};

      if (motion_model == "ctrv") {
        detections.push_back(detection.template get<cooperative_perception::CtrvDetection>());
      } else {
        throw std::runtime_error("Unsupported motion model");
      }
    }

    return detections;
  }
};

template <>
struct do_detections_from_json_file<CtraDetection>
{
  static auto _(std::ifstream & file)
  {
    const auto data = nlohmann::json::parse(file);
    std::vector<CtraDetection> detections;

    for (const auto & detection : data.at("detections")) {
      const auto motion_model{detection.at("motion_model").template get<std::string>()};

      if (motion_model == "ctra") {
        detections.push_back(detection.template get<cooperative_perception::CtraDetection>());
      } else {
        throw std::runtime_error("Unsupported motion model");
      }
    }

    return detections;
  }
};

template <typename... Alternatives>
struct do_detections_from_json_file<std::variant<Alternatives...>>
{
  static auto _(std::ifstream & file)
  {
    const auto data = nlohmann::json::parse(file);
    std::vector<std::variant<Alternatives...>> detections;

    for (const auto & detection : data.at("detections")) {
      const auto motion_model{detection.at("motion_model").template get<std::string>()};

      if (motion_model == "ctrv") {
        detections.push_back(detection.template get<cooperative_perception::CtrvDetection>());
      } else if (motion_model == "ctra") {
        detections.push_back(detection.template get<cooperative_perception::CtraDetection>());
      } else {
        throw std::runtime_error("Unsupported motion model");
      }
    }

    return detections;
  }
};

template <typename T>
struct do_tracks_from_json_file;

template <>
struct do_tracks_from_json_file<CtrvTrack>
{
  static auto _(std::ifstream & file)
  {
    const auto data = nlohmann::json::parse(file);
    std::vector<CtrvTrack> tracks;

    for (const auto & track : data.at("tracks")) {
      const auto motion_model{track.at("motion_model").template get<std::string>()};

      if (motion_model == "ctrv") {
        tracks.push_back(track.template get<cooperative_perception::CtrvTrack>());
      } else {
        throw std::runtime_error("Unsupported motion model");
      }
    }

    return tracks;
  }
};

template <>
struct do_tracks_from_json_file<CtraTrack>
{
  static auto _(std::ifstream & file)
  {
    const auto data = nlohmann::json::parse(file);
    std::vector<CtraTrack> tracks;

    for (const auto & track : data.at("tracks")) {
      const auto motion_model{track.at("motion_model").template get<std::string>()};

      if (motion_model == "ctra") {
        tracks.push_back(track.template get<cooperative_perception::CtraTrack>());
      } else {
        throw std::runtime_error("Unsupported motion model");
      }
    }

    return tracks;
  }
};

template <typename... Alternatives>
struct do_tracks_from_json_file<std::variant<Alternatives...>>
{
  static auto _(std::ifstream & file)
  {
    const auto data = nlohmann::json::parse(file);
    std::vector<std::variant<Alternatives...>> tracks;

    for (const auto & track : data.at("tracks")) {
      const auto motion_model{track.at("motion_model").template get<std::string>()};

      if (motion_model == "ctrv") {
        tracks.push_back(track.template get<cooperative_perception::CtrvTrack>());
      } else if (motion_model == "ctra") {
        tracks.push_back(track.template get<cooperative_perception::CtraTrack>());
      } else {
        throw std::runtime_error("Unsupported motion model");
      }
    }

    return tracks;
  }
};

}  // namespace customization

template <typename T>
inline constexpr auto detections_from_json_file =
  [](std::ifstream & file) { return customization::do_detections_from_json_file<T>::_(file); };

template <typename T>
inline constexpr auto tracks_from_json_file =
  [](std::ifstream & file) { return customization::do_tracks_from_json_file<T>::_(file); };

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_JSON_PARSING_HPP
