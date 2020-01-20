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
  auto profile = make_test_profile(UnitCircle);

  const bool test_performance = true;
  const std::size_t N = test_performance? 3000 : 1;

  GIVEN("Two stationary trajectories that do not overlap in space")
  {
    rmf_traffic::Trajectory trajectory_a("test_map");
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
        Eigen::Vector3d{10.0, 10.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_b.insert(
        begin_time + 10s,
        profile,
        Eigen::Vector3d{10.0, 10.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    REQUIRE(trajectory_b.size() == 2);


    const auto start_time = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < N; ++i)
    {
      CHECK_FALSE(rmf_traffic::DetectConflict::broad_phase(
          trajectory_a,
          trajectory_b));    
    }
    const auto end_time = std::chrono::steady_clock::now();

    if (test_performance)
    {
      const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
      std::cout << "Two stationary trajectories that do not overlap in space\n";
      std::cout << "Total: " << sec << std::endl;
      std::cout << "Per run: " << sec/N << std::endl;
    }
  }

  GIVEN("Two non-overlapping L-shaped trajectories")
  {
    rmf_traffic::Trajectory trajectory_a("test_map");
    trajectory_a.insert(
        begin_time,
        profile,
        Eigen::Vector3d{-10.0, 10.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_a.insert(
        begin_time + 10s,
        profile,
        Eigen::Vector3d{-5.0, 10.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_a.insert(
        begin_time + 15s,
        profile,
        Eigen::Vector3d{-5.0, 10.0, M_PI_2},
        Eigen::Vector3d{0.0, 0.0, 0.0});  
    trajectory_a.insert(
        begin_time + 25s,
        profile,
        Eigen::Vector3d{-5.0, 15.0, M_PI_2},
        Eigen::Vector3d{0.0, 0.0, 0.0});      
    REQUIRE(trajectory_a.size() == 4);

    rmf_traffic::Trajectory trajectory_b("test_map");
    trajectory_b.insert(
        begin_time,
        profile,
        Eigen::Vector3d{10.0, -10.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_b.insert(
        begin_time + 10s,
        profile,
        Eigen::Vector3d{10.0, -5.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_b.insert(
        begin_time + 15s,
        profile,
        Eigen::Vector3d{10.0, -5.0, M_PI},
        Eigen::Vector3d{0.0, 0.0, 0.0});  
    trajectory_b.insert(
        begin_time + 25s,
        profile,
        Eigen::Vector3d{5.0, -5.0, M_PI},
        Eigen::Vector3d{0.0, 0.0, 0.0});      
    REQUIRE(trajectory_b.size() == 4);

    const auto start_time = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < N; ++i)
    {
      CHECK_FALSE(rmf_traffic::DetectConflict::broad_phase(
          trajectory_a,
          trajectory_b));    
    }
    const auto end_time = std::chrono::steady_clock::now();

    if (test_performance)
    {
      const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
      std::cout << "Two non-overlapping L-shaped trajectories\n";
      std::cout << "Total: " << sec << std::endl;
      std::cout << "Per run: " << sec/N << std::endl;
    }
  }

  GIVEN("Two overlapping parallel straight-line trajectories with n segments")
  {
    int n_seg = 200;

    rmf_traffic::Trajectory trajectory_a("test_map");
    rmf_traffic::Trajectory trajectory_b("test_map");

    for (auto i = 0; i < n_seg; i++)
    {
      trajectory_a.insert(
      begin_time + std::chrono::seconds(i * 5),
      profile,
      Eigen::Vector3d{0.0 + i*10.0, 10.0, 0},
      Eigen::Vector3d{0.0, 0.0, 0.0});

      trajectory_b.insert(
      begin_time + std::chrono::seconds(i * 5),
      profile,
      Eigen::Vector3d{0.0 + i*10.0, 9.0, 0},
      Eigen::Vector3d{0.0, 0.0, 0.0});
    }
    REQUIRE(trajectory_a.size() == n_seg);
    REQUIRE(trajectory_b.size() == n_seg);

    const auto start_time = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < N; ++i)
    {
      CHECK(rmf_traffic::DetectConflict::broad_phase(
          trajectory_a,
          trajectory_b));    
    }
    const auto end_time = std::chrono::steady_clock::now();

    if (test_performance)
    {
      const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
      std::cout << "Straight-line trajectories with " << n_seg << " segments\n";
      std::cout << "Total: " << sec << std::endl;
      std::cout << "Per run: " << sec/N << std::endl;
    }
  }

  GIVEN("Time ranges and bounding boxes overlap but segments do not intersect")
  {
    rmf_traffic::Trajectory trajectory_a("test_map");
    trajectory_a.insert(
        begin_time,
        profile,
        Eigen::Vector3d{-10.0, 10.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_a.insert(
        begin_time + 5s,
        profile,
        Eigen::Vector3d{-5.0, 10.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_a.insert(
        begin_time + 10s,
        profile,
        Eigen::Vector3d{0.0, 10.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_a.insert(
        begin_time + 15s,
        profile,
        Eigen::Vector3d{0.0, 10.0, M_PI_2},
        Eigen::Vector3d{0.0, 0.0, 0.0});  
    trajectory_a.insert(
        begin_time + 30s,
        profile,
        Eigen::Vector3d{0.0, 15.0, M_PI_2},
        Eigen::Vector3d{0.0, 0.0, 0.0});      
    REQUIRE(trajectory_a.size() == 5);

    rmf_traffic::Trajectory trajectory_b("test_map");
    trajectory_b.insert(
        begin_time +6s,
        profile,
        Eigen::Vector3d{-8.0, 0.0, M_PI_2},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_b.insert(
        begin_time + 15s,
        profile,
        Eigen::Vector3d{-8.0, 15.0, M_PI_2},
        Eigen::Vector3d{0.0, 0.0, 0.0});
    trajectory_b.insert(
        begin_time + 25s,
        profile,
        Eigen::Vector3d{-8.0, 15.0, M_PI},
        Eigen::Vector3d{0.0, 0.0, 0.0});  
    trajectory_b.insert(
        begin_time + 30s,
        profile,
        Eigen::Vector3d{-15.0, 15.0, M_PI},
        Eigen::Vector3d{0.0, 0.0, 0.0});   
    REQUIRE(trajectory_b.size() == 4);

    const auto start_time = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < N; ++i)
    {
      CHECK_FALSE(rmf_traffic::DetectConflict::broad_phase(
          trajectory_a,
          trajectory_b));    
    }
    const auto end_time = std::chrono::steady_clock::now();

    if (test_performance)
    {
      const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
      std::cout << "Overlapping trajectory bounding boxes but segments do not intersect\n";
      std::cout << "Total: " << sec << std::endl;
      std::cout << "Per run: " << sec/N << std::endl;
    }
  }
}