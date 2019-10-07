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

#include <rmf_traffic/Conflict.hpp>

#include "utils_Trajectory.hpp"

#include <rmf_utils/catch.hpp>

#include <iostream>

SCENARIO("Test conflicts")
{
  using namespace std::chrono_literals;

  GIVEN("A straight 2-point trajectory")
  {
    const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();

    rmf_traffic::Trajectory trajectory_a("test_map");
    trajectory_a.insert(
          begin_time,
          make_test_profile(UnitBox),
          Eigen::Vector3d{-5.0,  0.0,  0.0},
          Eigen::Vector3d{ 0.0,  0.0,  0.0});

    trajectory_a.insert(
          begin_time + 10s,
          make_test_profile(UnitBox),
          Eigen::Vector3d{ 5.0,  0.0,  0.0},
          Eigen::Vector3d{ 0.0,  0.0,  0.0});
    REQUIRE(trajectory_a.size() == 2);

    WHEN("Checked against a conflicting Trajectory")
    {
      rmf_traffic::Trajectory trajectory_b("test_map");

      trajectory_b.insert(
            begin_time,
            make_test_profile(UnitBox),
            Eigen::Vector3d{ 0.0, -5.0,  0.0},
            Eigen::Vector3d{ 0.0,  0.0,  0.0});

      trajectory_b.insert(
            begin_time + 10s,
            make_test_profile(UnitBox),
            Eigen::Vector3d{ 0.0,  5.0,  0.0},
            Eigen::Vector3d{ 0.0,  0.0,  0.0});

      REQUIRE(trajectory_b.size() == 2);

      CHECK(rmf_traffic::DetectConflict::broad_phase(
              trajectory_a, trajectory_b));

      const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
            trajectory_a, trajectory_b);
      REQUIRE(conflicts.size() == 1);

      // Note: The expected time in this case is the root of the polynomial
      // equation in the range t=[0,10]:
      // -0.02 t^3 + 0.3 t^2 - 5 + d = 0
      // where d is the dimension of the box
      const double expected_time = 4.32931077;
      const double computed_time = rmf_traffic::time::to_seconds(
            conflicts.front().get_time() - begin_time);

      // Note: FCL is able to calculate the collision time to very high
      // precision, but it requires many iterations (~1000000 for a precision of
      // 1e-5s) which is far more expensive than the default (10 iterations for
      // a precision of ~0.2), and the exact moment in time is not really
      // important, as long as it falls within the relevant segment (which it
      // always should).
      CHECK(computed_time == Approx(expected_time).margin(0.5));
    }

    WHEN("Checked against a Trajectory that does not conflict")
    {
      rmf_traffic::Trajectory trajectory_c("test_map");

      // This trajectory is parallel to trajectory_a
      trajectory_c.insert(
            begin_time,
            make_test_profile(UnitBox),
            Eigen::Vector3d{-5.0,  5.0,  0.0},
            Eigen::Vector3d{ 0.0,  0.0,  0.0});

      trajectory_c.insert(
            begin_time + 10s,
            make_test_profile(UnitBox),
            Eigen::Vector3d{ 5.0,  5.0,  0.0},
            Eigen::Vector3d{ 0.0,  0.0,  0.0});

      REQUIRE(trajectory_c.size() == 2);

      CHECK(rmf_traffic::DetectConflict::broad_phase(
              trajectory_a, trajectory_c));

      const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
            trajectory_a, trajectory_c);
      CHECK(conflicts.empty());
    }
  }

  GIVEN("A curving 2-point trajectory")
  {
    const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();

    const double R = 1.0;

    // FIXME(MXG): FCL is not good enough for a radius this small. Consider
    // replacing FCL with Bullet.
//    const double R = 0.1;

    const auto small_circle = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(R);

    const auto small_profile =
        rmf_traffic::Trajectory::Profile::make_strict(small_circle);

    rmf_traffic::Trajectory trajectory_a("test_map");
    trajectory_a.insert(
          begin_time,
          small_profile,
          Eigen::Vector3d{-5.0,  0.0,  0.0},
          Eigen::Vector3d{ 1.0,  0.0,  0.0});

    trajectory_a.insert(
          begin_time + 10s,
          small_profile,
          Eigen::Vector3d{ 0.0, -5.0, 0.0},
          Eigen::Vector3d{ 0.0, -1.0, 0.0});
    REQUIRE(trajectory_a.size() == 2);

    WHEN("Checked against a conflicting Trajectory")
    {
      rmf_traffic::Trajectory trajectory_b("test_map");
      trajectory_b.insert(
            begin_time,
            small_profile,
            Eigen::Vector3d{ -5.0, -5.0,  0.0},
            Eigen::Vector3d{  1.0,  0.0,  0.0});

      trajectory_b.insert(
            begin_time + 10s,
            small_profile,
            Eigen::Vector3d{  0.0,  0.0,  0.0},
            Eigen::Vector3d{  0.0,  1.0,  0.0});
      REQUIRE(trajectory_b.size() == 2);

      CHECK(rmf_traffic::DetectConflict::broad_phase(
              trajectory_a, trajectory_b));

      const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
            trajectory_a, trajectory_b);
      REQUIRE(conflicts.size() == 1);

      // Note: The expected collision time, calculated by hand, is the root of:
      // -0.1 t^2 + 5 - 2r = 0
      const double expected_time = std::sqrt(50.0 - 20.0*R);
      const double computed_time = rmf_traffic::time::to_seconds(
            conflicts.front().get_time() - begin_time);

      CHECK(computed_time == Approx(expected_time).margin(0.2));
    }

    WHEN("Checked against a Trajectory that does not conflict")
    {
      rmf_traffic::Trajectory trajectory_c("test_map");
      trajectory_c.insert(
            begin_time,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{  5.0,  0.0,  0.0},
            Eigen::Vector3d{ -1.0,  0.0,  0.0});

      trajectory_c.insert(
            begin_time + 10s,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{  0.0,  5.0,  0.0},
            Eigen::Vector3d{  0.0,  1.0,  0.0});
      REQUIRE(trajectory_c.size() == 2);

      CHECK(rmf_traffic::DetectConflict::broad_phase(
              trajectory_a, trajectory_c));

      const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
            trajectory_a, trajectory_c);

      CHECK(conflicts.empty());
    }
  }

  GIVEN("Trajectories with very small profiles")
  {
    const double w = 0.001;
    const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();
    const auto small_box = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Box>(w, w);

    const auto small_profile =
        rmf_traffic::Trajectory::Profile::make_strict(small_box);

    rmf_traffic::Trajectory trajectory_a("test_map");
    trajectory_a.insert(
          begin_time,
          small_profile,
          Eigen::Vector3d::Zero(),
          1.0 * Eigen::Vector3d::UnitX());
    trajectory_a.insert(
          begin_time + 10s,
          small_profile,
          10.0 * Eigen::Vector3d::UnitX(),
          1.0 * Eigen::Vector3d::UnitX());

    rmf_traffic::Trajectory trajectory_b("test_map");
    trajectory_b.insert(
          begin_time,
          small_profile,
          10.0 * Eigen::Vector3d::UnitX(),
          -1.0 * Eigen::Vector3d::UnitX());
    trajectory_b.insert(
          begin_time + 10s,
          small_profile,
          Eigen::Vector3d::Zero(),
          -1.0 * Eigen::Vector3d::UnitX());

    const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
          trajectory_a, trajectory_b);

    REQUIRE(!conflicts.empty());

    // Note: The hand-calculated collision time is (10-w)/2.0
    const double expected_time = (10.0 - w)/2.0;
    const double computed_time = rmf_traffic::time::to_seconds(
          conflicts.front().get_time() - begin_time);

    CHECK(computed_time == Approx(expected_time).margin(0.2));
  }
}

/// Remaining test suggestions:
/// * multi-segment trajectories
/// * trajectories that only partially overlap in time
/// * trajectories that do not overlap in time at all to test
///   DetectConflict::between() and DetectConflict::broad_phase()
/// * trajectories that include rotating boxes which collide due to rotating

// A useful website for playing with 2D cubic splines: https://www.desmos.com/calculator/
