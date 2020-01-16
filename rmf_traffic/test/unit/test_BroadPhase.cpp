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

#include <rmf_utils/catch.hpp>

#include <rmf_traffic/Conflict.hpp>
#include "utils_Trajectory.hpp"

#include <iostream>

using namespace std::chrono_literals;

SCENARIO("Test Broad-Phase Collision Detection")
{
  auto begin_time = std::chrono::steady_clock::now();

  rmf_traffic::Trajectory trajectory_a("test_map");
  auto profile = make_test_profile(UnitCircle);
  trajectory_a.insert(
      begin_time,
      profile,
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 0.0});
  trajectory_a.insert(
      begin_time + 10s,
      profile,
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 0.0});

  REQUIRE(trajectory_a.size() == 2);

  rmf_traffic::Trajectory trajectory_b("test_map");
  trajectory_b.insert(
      begin_time,
      profile,
      Eigen::Vector3d{5.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 0.0});
  trajectory_b.insert(
      begin_time + 10s,
      profile,
      Eigen::Vector3d{5.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 0.0});

  REQUIRE(trajectory_b.size() == 2);

  CHECK(rmf_traffic::DetectConflict::broad_phase(
      trajectory_a,
      trajectory_b));
}