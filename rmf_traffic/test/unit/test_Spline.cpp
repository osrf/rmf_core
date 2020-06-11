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

#include <src/rmf_traffic/Spline.hpp>
#include "utils_Trajectory.hpp"

#include <rmf_utils/catch.hpp>

#include <iostream>

SCENARIO("Test spline")
{
  using namespace std::chrono_literals;

  const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();

  rmf_traffic::Trajectory trajectory_a;
  trajectory_a.insert(
    begin_time,
    Eigen::Vector3d{-5.0, 0.0, 0.0},
    Eigen::Vector3d{ 1.0, 0.0, 0.0});

  trajectory_a.insert(
    begin_time + 10s,
    Eigen::Vector3d{ 5.0, 0.0, 0.0},
    Eigen::Vector3d{ 1.0, 0.0, 0.0});
  REQUIRE(trajectory_a.size() == 2);

  rmf_traffic::Spline spline_a(++trajectory_a.begin());
  for (const auto delta_t : {0s, 1s, 2s, 3s, 4s, 5s, 6s, 7s, 8s, 9s, 10s})
  {
    const Eigen::Vector3d p = spline_a.compute_position(begin_time + delta_t);
    CHECK(p[0] == Approx(delta_t.count() - 5.0));
  }

  rmf_traffic::Trajectory trajectory_b;
  trajectory_b.insert(
    begin_time,
    Eigen::Vector3d{ 0.0, -5.0, 0.0},
    Eigen::Vector3d{ 0.0, 1.0, 0.0});

  trajectory_b.insert(
    begin_time + 10s,
    Eigen::Vector3d{ 0.0, 5.0, 0.0},
    Eigen::Vector3d{ 0.0, 1.0, 0.0});
  REQUIRE(trajectory_b.size() == 2);

  rmf_traffic::Spline spline_b(++trajectory_b.begin());
  for (const auto delta_t : {0s, 1s, 2s, 3s, 4s, 5s, 6s, 7s, 8s, 9s, 10s})
  {
    const Eigen::Vector3d p = spline_b.compute_position(begin_time + delta_t);
    CHECK(p[1] == Approx(delta_t.count() - 5.0));
  }
}
