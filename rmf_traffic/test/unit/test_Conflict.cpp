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

#include "utils_Conflict.hpp"
#include "utils_Trajectory.hpp"
#include "src/rmf_traffic/DetectConflictInternal.hpp"

#include <rmf_utils/catch.hpp>
#include <iostream>

using namespace std::chrono_literals;

/*
####################################################################################################
NOTE: When merging with schedule_tests, the profiles will need to be redefined with finalized shapes
####################################################################################################
*/

SCENARIO("DetectConflict unit tests")
{
  // We will call the reference trajectory t1, and comparison trajectory t2
  const double fcl_error_margin = 0.5;

  GIVEN(
    "A 2-point trajectory t1 with unit square box profile (stationary robot)")
  {
    const double profile_scale = 1.0;
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const auto shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(profile_scale, profile_scale);
    rmf_traffic::Profile profile{shape};
    Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
    rmf_traffic::Trajectory t1;
    t1.insert(time, pos, vel);
    t1.insert(time + 10s, pos, vel);

    WHEN("t1 and t2 are identical and overlapping")
    {
      rmf_traffic::Trajectory t2;
      t2.insert(time, pos, vel);
      t2.insert(time + 10s, pos, vel);

      THEN("The robots are not approaching each other, so it is not a conflict")
      {
        const auto conflicts = get_conflicts(profile, t1, profile, t2);
        CHECK(conflicts.empty());

        std::vector<ConflictDataParams> expected_conflicts;
        expected_conflicts.push_back({
            time,            // Start Time
            time,            // Expected Conflict Time
            ++t1.begin(),    // Waypoint from Trajectory 1 that will conflict
            ++t2.begin(),    // Waypoint from Trajectory 2 that will conflict
            fcl_error_margin // Error Margin for computing collision time
          });

        CHECK_ConflictList(conflicts, expected_conflicts);
        CHECK_between_is_commutative(profile, t1, profile, t2);
        CHECK_ConflictList(conflicts, expected_conflicts);
      }
    }

    WHEN(
      "t2 is t1 but displaced positionally, with bare overlap at the profile within error margin")
    {
      rmf_traffic::Trajectory t2;
      const double dx = 0.8;
      t2.insert(time, Eigen::Vector3d(dx, dx, dx), vel);
      t2.insert(time + 10s, Eigen::Vector3d(dx, dx, dx), vel);

      THEN("The robots are not approaching each other, so it is not a conflict")
      {
        const auto conflicts = get_conflicts(profile, t1, profile, t2);
        CHECK(conflicts.empty());

        std::vector<ConflictDataParams> expected_conflicts;
        expected_conflicts.push_back({
            time,            // Start Time
            time,            // Expected Conflict Time
            ++t1.begin(),    // Waypoint from Trajectory 1 that will conflict
            ++t2.begin(),    // Waypoint from Trajectory 2 that will conflict
            fcl_error_margin // Error Margin for computing collision time
          });

        CHECK_ConflictList(conflicts, expected_conflicts);
        CHECK_between_is_commutative(profile, t1, profile, t2);
      }
    }

    WHEN("t2 is t1 but displaced positionally outside error margin")
    {
      rmf_traffic::Trajectory t2;
      const double dx = 3;
      const Eigen::Vector3d new_pos = Eigen::Vector3d(dx, dx, dx);
      t2.insert(time, new_pos, vel);
      t2.insert(time + 10s, new_pos, vel);
      REQUIRE(t2.size() == 2);
    }

    WHEN("t1 and t2 overlap positionally but not in timing")
    {
      rmf_traffic::Trajectory t2;
      t2.insert(time+11s, pos, vel);
      t2.insert(time+20s, pos, vel);
      REQUIRE(t2.size() == 2);

      THEN("DetectConflict::between should return empty")
      {
        const auto conflicts = get_conflicts(profile, t1, profile, t2);
        REQUIRE(conflicts.empty());
        CHECK_between_is_commutative(profile, t1, profile, t2);
      }
    }
  }

  GIVEN("A stationary trajectory with Box profile")
  {
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    rmf_traffic::Trajectory t1;
    const double box_x_length = 2;
    const double box_y_length = 1;
    const auto shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(box_x_length, box_y_length);
    const rmf_traffic::Profile profile{shape};

    Eigen::Vector3d pos = Eigen::Vector3d{0, 0, 0};
    Eigen::Vector3d vel = Eigen::Vector3d{0, 0, 0};
    t1.insert(time, pos, vel);
    t1.insert(time + 10s, pos, vel);
    REQUIRE(t1.size() == 2);

    WHEN("Checked against Trajctory t2 that does not conflict")
    {
      rmf_traffic::Trajectory t2;
      t2.insert(time, Eigen::Vector3d{0.0, 1.5, 0}, Eigen::Vector3d{0, 0, 0});
      t2.insert(time+10s, Eigen::Vector3d{0, 1.5, 0}, Eigen::Vector3d{0, 0, 0});
      REQUIRE(t2.size() == 2);

      THEN("narrow_phase should return empty")
      {
        CHECK(!rmf_traffic::DetectConflict::between(profile, t1, profile, t2));
        CHECK_between_is_commutative(profile, t1, profile, t2);
      }
    }

// TODO(MXG): This test is currently failing because of errors with
// FCL continuous collision detection. We should only use circular
// robot profiles until this is resolved.
    // WHEN("Checked against Trajctory t3 that conflicts due to robot rotation")
    // {
    //   rmf_traffic::Trajectory t3("test_map");
    //   t3.insert(time, profile, Eigen::Vector3d{0, 1.5, 0},Eigen::Vector3d{0, 0, 0});
    //   t3.insert(time+10s, profile, Eigen::Vector3d{0, 1.5, M_PI_2}, Eigen::Vector3d{0, 0, 0});
    //   REQUIRE(t3.size() == 2);

    //   THEN("broad_phase should detect collision")
    //   {
    //     REQUIRE(rmf_traffic::DetectConflict::broad_phase(t1, t3));
    //     CHECK_broad_phase_is_commutative(t1, t3);

    //     THEN("narrow_phase should return ConflictData")
    //     {
    //       const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(t1, t3);
    //       REQUIRE_FALSE(conflicts.empty());
    //       REQUIRE(conflicts.size() == 1);
    //       CHECK_narrow_phase_is_commutative(t1, t3);
    //       //th(t)=-3.14(t/10)^3 + 4.71(t/10)^2
    //       //sin(th(t))+0.5cos(th(t))=1
    //       //th(t)~=0.65 => t~=4.42
    //       //box rotating on top collides when the y coordinate of its bottom-left corner equals half the box height
    //       const double expected_time = 4.42;
    //       const double computed_time = rmf_traffic::time::to_seconds(conflicts.front().get_time() - time);
    //       CHECK(computed_time == Approx(expected_time).margin(fcl_error_margin));
    //     }
    //   }

    //   THEN("between should return ConflictData")
    //   {
    //     const auto conflicts=rmf_traffic::DetectConflict::narrow_phase(t1, t3);
    //     REQUIRE_FALSE(conflicts.empty());
    //     REQUIRE(conflicts.size() == 1);
    //     CHECK_between_is_commutative(t1, t3);

    //   }
    // }
  }


  GIVEN("A straight 2-point trajectory")
  {
    const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();
    const auto profile = create_test_profile(UnitBox);

    rmf_traffic::Trajectory trajectory_a;
    trajectory_a.insert(
      begin_time,
      Eigen::Vector3d{-5.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 0.0});

    trajectory_a.insert(
      begin_time + 10s,
      Eigen::Vector3d{5.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 0.0});
    REQUIRE(trajectory_a.size() == 2);

    WHEN("Checked against a conflicting Trajectory")
    {
      rmf_traffic::Trajectory trajectory_b;
      trajectory_b.insert(
        begin_time,
        Eigen::Vector3d{0.0, -5.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});

      trajectory_b.insert(
        begin_time + 10s,
        Eigen::Vector3d{0.0, 5.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});

      REQUIRE(trajectory_b.size() == 2);

      CHECK(rmf_traffic::DetectConflict::between(
          profile, trajectory_a, profile, trajectory_b));

      const auto conflicts = get_conflicts(
        profile, trajectory_a, profile, trajectory_b);
      REQUIRE(conflicts.size() == 1);

      // Note: The expected time in this case is the root of the polynomial
      // equation in the range t=[0,10]:
      // -0.02 t^3 + 0.3 t^2 - 4 t = 0
      const double expected_time = 4.32931077;
      const double computed_time = rmf_traffic::time::to_seconds(
        conflicts.front().time - begin_time);

      // Note: FCL is able to calculate the collision time to very high
      // precision, but it requires many iterations (~1000000 for a precision of
      // 1e-5s) which is far more expensive than the default (10 iterations for
      // a precision of ~0.2), and the exact moment in time is not really
      // important, as long as it falls within the relevant segment (which it
      // always should).
      CHECK(computed_time == Approx(expected_time).margin(fcl_error_margin));
    }

    WHEN("Checked against a Trajectory that does not conflict")
    {
      const auto profile = create_test_profile(UnitBox);

      rmf_traffic::Trajectory trajectory_c;

      // This trajectory is parallel to trajectory_a
      trajectory_c.insert(
        begin_time,
        Eigen::Vector3d{-5.0, 5.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});

      trajectory_c.insert(
        begin_time + 10s,
        Eigen::Vector3d{5.0, 5.0, 0.0},
        Eigen::Vector3d{0.0, 0.0, 0.0});

      REQUIRE(trajectory_c.size() == 2);

      const auto conflicts = get_conflicts(
        profile, trajectory_a, profile, trajectory_c);
      CHECK(conflicts.empty());
    }
  }

  GIVEN("A curving 2-point trajectory")
  {
    const auto profile = create_test_profile(UnitCircle);
    const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();

    rmf_traffic::Trajectory trajectory_a;
    trajectory_a.insert(
      begin_time,
      Eigen::Vector3d{-5.0, 0.0, 0.0},
      Eigen::Vector3d{1.0, 0.0, 0.0});

    trajectory_a.insert(
      begin_time + 10s,
      Eigen::Vector3d{0.0, -5.0, 0.0},
      Eigen::Vector3d{0.0, -1.0, 0.0});
    REQUIRE(trajectory_a.size() == 2);

    WHEN("Checked against a conflicting Trajectory")
    {
      rmf_traffic::Trajectory trajectory_b;
      trajectory_b.insert(
        begin_time,
        Eigen::Vector3d{-5.0, -5.0, 0.0},
        Eigen::Vector3d{1.0, 0.0, 0.0});

      trajectory_b.insert(
        begin_time + 10s,
        Eigen::Vector3d{0.0, 0.0, 0.0},
        Eigen::Vector3d{0.0, 1.0, 0.0});
      REQUIRE(trajectory_b.size() == 2);

      const auto conflicts = get_conflicts(
        profile, trajectory_a, profile, trajectory_b);
      REQUIRE(conflicts.size() == 1);

      // Note: The expected collision time, calculated by hand, is sqrt(30)
      const double expected_time = std::sqrt(30.0);
      const double computed_time = rmf_traffic::time::to_seconds(
        conflicts.front().time - begin_time);

      CHECK(computed_time == Approx(expected_time).margin(0.2));
    }

    WHEN("Checked against a Trajectory that does not conflict")
    {
      const auto profile = create_test_profile(UnitCircle);

      rmf_traffic::Trajectory trajectory_c;
      trajectory_c.insert(
        begin_time,
        Eigen::Vector3d{5.0, 0.0, 0.0},
        Eigen::Vector3d{-1.0, 0.0, 0.0});

      trajectory_c.insert(
        begin_time + 10s,
        Eigen::Vector3d{0.0, 5.0, 0.0},
        Eigen::Vector3d{0.0, 1.0, 0.0});
      REQUIRE(trajectory_c.size() == 2);

      const auto conflicts = get_conflicts(
        profile, trajectory_a, profile, trajectory_c);

      CHECK(conflicts.empty());
    }
  }

  GIVEN("A multi-segment trajectory with straight segments")
  {
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    rmf_traffic::Trajectory t1;
    double box_x_length = 1;
    double box_y_length = 1;
    const auto shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(box_x_length, box_y_length);
    const rmf_traffic::Profile profile{shape};

    //L shaped trajectory
    t1.insert(time, Eigen::Vector3d{0, 5, 0}, Eigen::Vector3d::Zero());
    t1.insert(time + 10s, Eigen::Vector3d{0, -5, 0}, Eigen::Vector3d::Zero());
    t1.insert(time + 20s, Eigen::Vector3d{0, -5, M_PI_2},
      Eigen::Vector3d::Zero());
    t1.insert(time + 30s, Eigen::Vector3d{10, -5, M_PI_2},
      Eigen::Vector3d::Zero());
    REQUIRE(t1.size() == 4);

    WHEN("Checked with a trajectory that intersects first segment of t1")
    {
      rmf_traffic::Trajectory t2;
      t2.insert(time, Eigen::Vector3d{-5, 0, 0}, Eigen::Vector3d{0, 0, 0});
      t2.insert(time + 10s, Eigen::Vector3d{5, 0, 0}, Eigen::Vector3d{0, 0, 0});
      REQUIRE(t2.size() == 2);

      auto conflicts = get_conflicts(profile, t1, profile, t2);
      CHECK(conflicts.size() == 1);
      CHECK(conflicts.front().a_it == ++t1.begin()); //segment with the conflict

      const double expected_time = 4.32931077;
      const double computed_time =
        rmf_traffic::time::to_seconds(conflicts.front().time - time);
      CHECK(computed_time == Approx(expected_time).margin(fcl_error_margin));
    }

    WHEN("Checked with a trajectory that intersects last segment of t1")
    {
      rmf_traffic::Trajectory t2;
      t2.insert(time + 20s, Eigen::Vector3d{5, 0, 0}, Eigen::Vector3d{0, 0, 0});
      t2.insert(time + 30s, Eigen::Vector3d{5, -10, 0},
        Eigen::Vector3d{0, 0, 0});
      REQUIRE(t2.size() == 2);

      const auto conflicts = get_conflicts(profile, t1, profile, t2);
      CHECK(conflicts.size() == 1);
      CHECK(conflicts.front().a_it == --t1.end()); //segment with the conflict

      const double expected_time = 24.32931077;
      const double computed_time =
        rmf_traffic::time::to_seconds(conflicts.front().time - time);

      CHECK(computed_time == Approx(expected_time).margin(fcl_error_margin));
    }

    WHEN(
      "Checked with a multi-segment trajectory that intersects t1 at two-points")
    {
      rmf_traffic::Trajectory t2;
      t2.insert(time, Eigen::Vector3d{-5, -0, -M_PI_4},
        Eigen::Vector3d{0, 0, 0});
      t2.insert(time + 2s, Eigen::Vector3d{0, 0, -M_PI_4}, Eigen::Vector3d{0, 0,
          0});
      t2.insert(time + 10s, Eigen::Vector3d{0, 0, -M_PI_4}, Eigen::Vector3d{0,
          0, 0});
      t2.insert(time + 20s, Eigen::Vector3d{5, -5, M_PI_4}, Eigen::Vector3d{0,
          0, 0});
      t2.insert(time + 30s, Eigen::Vector3d{5, -5, M_PI_4}, Eigen::Vector3d{0,
          0, 0});

      REQUIRE(t2.size() == 5);

      auto conflicts = get_conflicts(profile, t1, profile, t2);
      CHECK(conflicts.size() == 2);
      CHECK(conflicts.front().a_it == ++t1.begin()); // segment with the conflict
      CHECK(conflicts.back().a_it == --t1.end()); // segment with the conflict
    }

    WHEN("Checked with a trajectory that overlaps partially in time")
    {
      rmf_traffic::Trajectory t2;
      t2.insert(time + 25s, Eigen::Vector3d{-5, 5, 0},
        Eigen::Vector3d{0, 0, 0});
      t2.insert(time + 35s, Eigen::Vector3d{5, 5, 0}, Eigen::Vector3d{0, 0, 0});
      REQUIRE(t2.size() == 2);

      CHECK_FALSE(rmf_traffic::DetectConflict::between(profile, t1, profile,
        t2));
    }
  }

  GIVEN("A trajectory with a curve")
  {
    const double r = 1.0;
    const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();
    const auto circle = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(r);

    const rmf_traffic::Profile profile{circle};

    rmf_traffic::Trajectory trajectory;
    trajectory.insert(
      begin_time,
      Eigen::Vector3d(-10.0, -5.0, 0.0),
      Eigen::Vector3d(0.5, 0.0, 0.0));

    trajectory.insert(
      begin_time + 10s,
      Eigen::Vector3d(-5.0, -5.0, 0.0),
      Eigen::Vector3d(0.5, 0.0, 0.0));

    trajectory.insert(
      begin_time + 30s,
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.5, 0.0));

    trajectory.insert(
      begin_time + 40s,
      Eigen::Vector3d(0.0, 5.0, 0.0),
      Eigen::Vector3d(0.0, 0.5, 0.0));

    rmf_traffic::internal::Spacetime region{
      nullptr, nullptr,
      Eigen::Isometry2d(Eigen::Translation2d(Eigen::Vector2d(0.0, -5.0))),
      circle
    };

    WHEN("The spacetime range spans all time")
    {
      std::vector<ConflictData> conflicts;
      const bool has_conflicts = rmf_traffic::internal::detect_conflicts(
        profile, trajectory, region, &conflicts);
      CHECK(has_conflicts);

      REQUIRE(conflicts.size() == 1);
      CHECK(conflicts.front().a_it == ++(++trajectory.begin()));
      CHECK(conflicts.front().a_it == trajectory.find(begin_time + 20s));
    }

    WHEN("The time range begins and ends before the collision")
    {
      const rmf_traffic::Time region_start_time = begin_time - 20s;
      const rmf_traffic::Time region_finish_time = begin_time + 12s;

      region.lower_time_bound = &region_start_time;
      region.upper_time_bound = &region_finish_time;

      const bool has_conflicts = rmf_traffic::internal::detect_conflicts(
        profile, trajectory, region, nullptr);
      CHECK_FALSE(has_conflicts);
    }

    WHEN("The time range begins and ends after the collision")
    {
      const rmf_traffic::Time region_start_time = begin_time + 28s;
      const rmf_traffic::Time region_finish_time = begin_time + 60s;

      region.lower_time_bound = &region_start_time;
      region.upper_time_bound = &region_finish_time;

      const bool has_conflicts = rmf_traffic::internal::detect_conflicts(
        profile, trajectory, region, nullptr);
      CHECK_FALSE(has_conflicts);
    }

    WHEN("The time range begins before and ends after the collision")
    {
      const rmf_traffic::Time region_start_time = begin_time + 12s;
      const rmf_traffic::Time region_finish_time = begin_time + 28s;

      region.lower_time_bound = &region_start_time;
      region.upper_time_bound = &region_finish_time;

      std::vector<ConflictData> conflicts;
      const bool has_conflicts = rmf_traffic::internal::detect_conflicts(
        profile, trajectory, region, &conflicts);
      CHECK(has_conflicts);

      REQUIRE(conflicts.size() == 1);
      CHECK(conflicts.front().a_it == ++(++trajectory.begin()));
      CHECK(conflicts.front().a_it == trajectory.find(begin_time + 20s));
    }
  }
}

// A useful website for playing with 2D cubic splines: https://www.desmos.com/calculator/
