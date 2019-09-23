/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic/track/Interpolate.hpp>

namespace rmf_traffic {
namespace track {

namespace {

//==============================================================================
double cross_2d(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1)
{
  return p0[0]*p1[1] - p0[1]*p1[0];
}

} // anonymous namespace

//==============================================================================
Trajectory Interpolate::positions(
    std::string map,
    Time start_time,
    const VehicleTraits& traits,
    const std::vector<Eigen::Vector3d>& input_positions,
    const Options& options)
{
  Trajectory trajectory{std::move(map)};

  if(input_positions.empty())
    return trajectory;

  trajectory.insert(
        start_time,
        nullptr,
        input_positions.front(),
        Eigen::Vector3d::Zero());

  const bool always_stop = options.always_stop();
  const double min_linear_distance = options.get_minimum_linear_distance();
  const double min_angular_distance = options.get_minimum_rotational_distance();
  const std::size_t N = input_positions.size();
  std::size_t last_stop_index = 0;
  for(std::size_t i=1; i < N; ++i)
  {
    const Eigen::Vector3d& last_position = input_positions[last_stop_index];
    const Eigen::Vector3d& next_position = input_positions[i];

    if(!always_stop)
    {
      bool can_skip = i+1 < N;

      const Eigen::Vector2d next_p = next_position.block<2,1>(0,0);
      const Eigen::Vector2d last_p = last_position.block<2,1>(0,0);

      if(can_skip && (next_p - last_p).norm() > min_linear_distance)
        can_skip = false;

      const double next_angle = next_position[2];
      const double last_angle = last_position[2];
      if(can_skip && std::abs(next_angle - last_angle) > min_angular_distance)
        can_skip = false;

      if(can_skip)
      {
        // Check if the corner is too sharp
        const Eigen::Vector2d future_p = input_positions[i+1].block<2,1>(0,0);
        const Eigen::Vector2d d_next_p = next_p - last_p;
        const Eigen::Vector2d d_future_p = future_p - next_p;

        const double cross = cross_2d(d_next_p, d_future_p);

      }
    }
  }


  return trajectory;
}

} // namespace track
} // namespace rmf_traffic
