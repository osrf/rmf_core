/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
 *
*/

#include <rmf_utils/math.hpp>

#include "DifferentialDriveMap.hpp"

#include "../internal_Interpolate.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
DifferentialDriveMapTypes::RouteInfo::RouteInfo(
  rmf_traffic::Time finish_time_,
  double finish_yaw_,
  std::vector<Route> routes_)
: finish_time(finish_time_),
  finish_yaw(finish_yaw_),
  routes(std::move(routes_))
{
  // Do nothing
}

//==============================================================================
FactoryInfo make_differential_drive_translate_factory(
    Eigen::Vector3d start,
    Eigen::Vector3d finish,
    KinematicLimits limits,
    double translation_thresh,
    double rotation_thresh,
    std::vector<std::string> maps)
{
  const auto dummy_start_time = rmf_traffic::Time(rmf_traffic::Duration(0));
  Trajectory trajectory;
  trajectory.insert(dummy_start_time, start, Eigen::Vector3d::Zero());
  internal::interpolate_translation(
        trajectory, limits.linear.velocity, limits.linear.acceleration,
        dummy_start_time, start, finish, translation_thresh);

  const double minimal_cost =
      rmf_traffic::time::to_seconds(trajectory.duration());

  auto factory =
      [start,
       finish,
       limits,
       translation_thresh,
       rotation_thresh,
       maps = std::move(maps)](
      rmf_traffic::Time start_time,
      double initial_yaw)
      -> DifferentialDriveMapTypes::RouteInfo
  {
    Trajectory trajectory;
    const Eigen::Vector3d pre_start{start.x(), start.y(), initial_yaw};
    trajectory.insert(start_time, pre_start, Eigen::Vector3d::Zero());

    internal::interpolate_rotation(
          trajectory, limits.angular.velocity, limits.angular.acceleration,
          start_time, pre_start, start, rotation_thresh);

    internal::interpolate_translation(
          trajectory, limits.linear.velocity, limits.linear.acceleration,
          trajectory.back().time(), start, finish, translation_thresh);

    if (trajectory.size() < 2)
      return {start_time, initial_yaw, {}};

    std::vector<Route> routes;
    routes.reserve(maps.size());
    for (const auto& map : maps)
      routes.push_back({map, trajectory});

    return {*trajectory.finish_time(), finish[2], routes};
  };

  auto factory_factory = [factory = std::move(factory)](
      std::optional<double>)
      -> DifferentialDriveMapTypes::RouteFactory
  {
    return [factory](
        const rmf_traffic::Time start_time,
        const double initial_yaw) -> DifferentialDriveMapTypes::RouteInfo
    {
      return factory(start_time, initial_yaw);
    };
  };

  return {minimal_cost, std::move(factory_factory)};
}

//==============================================================================
std::optional<FactoryInfo> make_rotate_factory(
    Eigen::Vector2d position,
    std::optional<double> start_yaw,
    std::optional<double> finish_yaw,
    KinematicLimits limits,
    double rotation_thresh,
    std::string map)
{
  double minimum_cost = 0.0;
  if (start_yaw.has_value() && finish_yaw.has_value())
  {
    const double yaw_dist =
        std::abs(rmf_utils::wrap_to_pi(*start_yaw - *finish_yaw));

    // If both yaws are known and they are within the rotation threshold of each
    // other, then no rotation will be needed.
    if (yaw_dist <= rotation_thresh)
      return std::nullopt;

    // If both yaws are known and a rotation is needed, then let's calculate the
    // time required for it.
    const auto dummy_start_time = rmf_traffic::Time(rmf_traffic::Duration(0));
    Trajectory trajectory;

    const Eigen::Vector3d start_p{position.x(), position.y(), *start_yaw};
    trajectory.insert(dummy_start_time, start_p, Eigen::Vector3d::Zero());

    const Eigen::Vector3d finish_p{position.x(), position.y(), *finish_yaw};
    internal::interpolate_rotation(
          trajectory, limits.angular.velocity, limits.angular.acceleration,
          dummy_start_time, start_p, finish_p, rotation_thresh);

    minimum_cost = rmf_traffic::time::to_seconds(trajectory.duration());
  }

  auto factory =
      [p = position,
       finish_yaw,
       limits = limits.angular,
       rotation_thresh,
       map = std::move(map)](
      rmf_traffic::Time start_time,
      double initial_yaw,
      std::optional<double> child_yaw)
      -> DifferentialDriveMapTypes::RouteInfo
  {
    Trajectory trajectory;
    const Eigen::Vector3d start_p = {p.x(), p.y(), initial_yaw};
    trajectory.insert(start_time, start_p, Eigen::Vector3d::Zero());

    if (finish_yaw.has_value())
    {
      const Eigen::Vector3d finish_p = {p.x(), p.y(), *finish_yaw};
      internal::interpolate_rotation(
            trajectory, limits.velocity, limits.acceleration, start_time,
            start_p, finish_p, rotation_thresh);
    }
    else if (child_yaw.has_value())
    {
      const Eigen::Vector3d finish_p = {p.x(), p.y(), *child_yaw};
      internal::interpolate_rotation(
            trajectory, limits.velocity, limits.acceleration, start_time,
            start_p, finish_p, rotation_thresh);
    }
    else
    {
      // If neither finish_yaw nor child_yaw has a value, then no rotation is
      // actually needed.
      return {start_time, initial_yaw, {}};
    }

    if (trajectory.size() < 2)
      return {start_time, initial_yaw, {}};

    const auto finish_time = *trajectory.finish_time();
    const auto finish_yaw = trajectory.back().position()[2];
    return {finish_time, finish_yaw, {{map, std::move(trajectory)}}};
  };

  auto factory_factory = [factory = std::move(factory)](
      std::optional<double> child_yaw)
      -> DifferentialDriveMapTypes::RouteFactory
  {
    return [factory, child_yaw](
        const rmf_traffic::Time start_time,
        const double initial_yaw) -> DifferentialDriveMapTypes::RouteInfo
    {
      return factory(start_time, initial_yaw, child_yaw);
    };
  };

  return std::optional<FactoryInfo>({minimum_cost, std::move(factory_factory)});
}

//==============================================================================
DifferentialDriveMapTypes::RouteFactoryFactory
make_hold_factory(
    Eigen::Vector2d position,
    std::optional<double> yaw_opt,
    rmf_traffic::Duration duration,
    KinematicLimits limits,
    double rotation_thresh,
    std::vector<std::string> maps)
{
  auto factory =
      [p = position,
       yaw_opt,
       duration,
       limits = limits.angular,
       rotation_thresh,
       maps = std::move(maps)](
      rmf_traffic::Time start_time,
      double initial_yaw,
      std::optional<double> child_yaw)
      -> DifferentialDriveMapTypes::RouteInfo
  {
    rmf_traffic::Trajectory trajectory;
    const Eigen::Vector3d start_position{p.x(), p.y(), initial_yaw};
    trajectory.insert(start_time, start_position, Eigen::Vector3d::Zero());

    Eigen::Vector3d yawed_position = start_position;
    if (yaw_opt.has_value())
    {
      yawed_position = {p.x(), p.y(), *yaw_opt};
      internal::interpolate_rotation(
            trajectory, limits.velocity, limits.acceleration,
            trajectory.back().time(), start_position, yawed_position,
            rotation_thresh);
    }
    else if (child_yaw.has_value())
    {
      yawed_position = {p.x(), p.y(), *child_yaw};
      internal::interpolate_rotation(
            trajectory, limits.velocity, limits.acceleration,
            trajectory.back().time(), yawed_position, yawed_position,
            rotation_thresh);
    }

    const auto desired_finish_time = start_time + duration;
    const auto remaining_duration =
        desired_finish_time - *trajectory.finish_time();

    if (remaining_duration > std::chrono::nanoseconds(0))
    {
      trajectory.insert(
        desired_finish_time,
        yawed_position,
        Eigen::Vector3d::Zero());
    }

    if (trajectory.size() < 2)
      return {start_time, initial_yaw, {}};

    std::vector<Route> routes;
    routes.reserve(maps.size());
    for (const auto& map : maps)
      routes.push_back({map, trajectory});

    const auto finish_time = *trajectory.finish_time();
    const auto finish_yaw = trajectory.back().position()[2];
    return {finish_time, finish_yaw, std::move(routes)};
  };

  return [factory = std::move(factory)](std::optional<double> child_yaw)
      -> DifferentialDriveMapTypes::RouteFactory
  {
    return [factory, child_yaw](
        rmf_traffic::Time start_time,
        double initial_yaw) -> DifferentialDriveMapTypes::RouteInfo
    {
      return factory(start_time, initial_yaw, child_yaw);
    };
  };
}

//==============================================================================
DifferentialDriveMapTypes::RouteFactoryFactory
make_recycling_factory(DifferentialDriveMapTypes::RouteFactory old_factory)
{
  return [old_factory = std::move(old_factory)](auto)
  {
    return old_factory;
  };
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
