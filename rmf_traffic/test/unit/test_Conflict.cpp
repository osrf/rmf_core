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
#include <rmf_utils/catch.hpp>
#include <iostream>

using namespace std::chrono_literals;

SCENARIO("DetectConflict unit tests")
{
      GIVEN("A 2-point trajectory t1 (stationary robot)")
      {
            const rmf_traffic::Time time = std::chrono::steady_clock::now();
            std::shared_ptr<rmf_traffic::geometry::Box> shape = std::make_shared<rmf_traffic::geometry::Box>(1.0, 1.0);
            rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(shape);
            Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
            Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
            rmf_traffic::Trajectory t1("test_map");
            t1.insert(time, profile, pos, vel);
            t1.insert(time + 10s, profile, pos, vel);

            WHEN("Stacking t1 and an identical 2-point trajectory t2 ( stationary robot )")
            {
                  rmf_traffic::Trajectory t2("test_map");
                  t2.insert(time, profile, pos, vel);
                  t2.insert(time + 10s, profile, pos, vel);

                  THEN("The broad phase function detects a conflict")
                  {
                        CHECK(rmf_traffic::DetectConflict::broad_phase(t1, t2));
                        CHECK_broad_phase_is_commutative(t1, t2);

                        THEN("The narrow phase function reports the details of conflict")
                        {
                              auto narrow_phase_conflicts = rmf_traffic::DetectConflict::narrow_phase(t1, t2);
                              CHECK(narrow_phase_conflicts.size() == 1);

                              std::vector<ConflictDataParams> expected_conflicts;
                              expected_conflicts.push_back({
                                  time,         // Start Time
                                  time,         // Expected Conflict Time
                                  ++t1.begin(), // Segment from Trajectory 1 that will conflict
                                  ++t2.begin(), // Segment from Trajectory 2 that will conflict
                                  0.2           // Error Margin
                              });

                              CHECK_ConflictList(narrow_phase_conflicts, expected_conflicts);
                              CHECK_narrow_phase_is_commutative(t1, t2);

                              THEN("The between function gives the same output")
                              {
                                    auto between_conflicts = rmf_traffic::DetectConflict::between(t1, t2);
                                    CHECK(between_conflicts.size() == 1);
                                    CHECK_ConflictList(between_conflicts, expected_conflicts);

                              }
                        }
                  }
            }
      }
}
// SCENARIO("Test conflicts")
// {
//       using namespace std::chrono_literals;

//       GIVEN("A straight 2-point trajectory")
//       {
//             const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();

//             rmf_traffic::Trajectory trajectory_a("test_map");
//             trajectory_a.insert(
//                 begin_time,
//                 make_test_profile(UnitBox),
//                 Eigen::Vector3d{-5.0, 0.0, 0.0},
//                 Eigen::Vector3d{0.0, 0.0, 0.0});

//             trajectory_a.insert(
//                 begin_time + 10s,
//                 make_test_profile(UnitBox),
//                 Eigen::Vector3d{5.0, 0.0, 0.0},
//                 Eigen::Vector3d{0.0, 0.0, 0.0});
//             REQUIRE(trajectory_a.size() == 2);

//             WHEN("Checked against a conflicting Trajectory")
//             {
//                   rmf_traffic::Trajectory trajectory_b("test_map");

//                   trajectory_b.insert(
//                       begin_time,
//                       make_test_profile(UnitBox),
//                       Eigen::Vector3d{0.0, -5.0, 0.0},
//                       Eigen::Vector3d{0.0, 0.0, 0.0});

//                   trajectory_b.insert(
//                       begin_time + 10s,
//                       make_test_profile(UnitBox),
//                       Eigen::Vector3d{0.0, 5.0, 0.0},
//                       Eigen::Vector3d{0.0, 0.0, 0.0});

//                   REQUIRE(trajectory_b.size() == 2);

//                   CHECK(rmf_traffic::DetectConflict::broad_phase(
//                       trajectory_a, trajectory_b));

//                   const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
//                       trajectory_a, trajectory_b);
//                   REQUIRE(conflicts.size() == 1);

//                   // Note: The expected time in this case is the root of the polynomial
//                   // equation in the range t=[0,10]:
//                   // -0.02 t^3 + 0.3 t^2 - 4 t = 0
//                   const double expected_time = 4.32931077;
//                   const double computed_time = rmf_traffic::time::to_seconds(
//                       conflicts.front().get_time() - begin_time);

//                   // Note: FCL is able to calculate the collision time to very high
//                   // precision, but it requires many iterations (~1000000 for a precision of
//                   // 1e-5s) which is far more expensive than the default (10 iterations for
//                   // a precision of ~0.2), and the exact moment in time is not really
//                   // important, as long as it falls within the relevant segment (which it
//                   // always should).
//                   CHECK(computed_time == Approx(expected_time).margin(0.2));
//             }

//             WHEN("Checked against a Trajectory that does not conflict")
//             {
//                   rmf_traffic::Trajectory trajectory_c("test_map");

//                   // This trajectory is parallel to trajectory_a
//                   trajectory_c.insert(
//                       begin_time,
//                       make_test_profile(UnitBox),
//                       Eigen::Vector3d{-5.0, 5.0, 0.0},
//                       Eigen::Vector3d{0.0, 0.0, 0.0});

//                   trajectory_c.insert(
//                       begin_time + 10s,
//                       make_test_profile(UnitBox),
//                       Eigen::Vector3d{5.0, 5.0, 0.0},
//                       Eigen::Vector3d{0.0, 0.0, 0.0});

//                   REQUIRE(trajectory_c.size() == 2);

//                   CHECK(rmf_traffic::DetectConflict::broad_phase(
//                       trajectory_a, trajectory_c));

//                   const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
//                       trajectory_a, trajectory_c);
//                   CHECK(conflicts.empty());
//             }
//       }

//       GIVEN("A curving 2-point trajectory")
//       {
//             const rmf_traffic::Time begin_time = std::chrono::steady_clock::now();

//             rmf_traffic::Trajectory trajectory_a("test_map");
//             trajectory_a.insert(
//                 begin_time,
//                 make_test_profile(UnitCircle),
//                 Eigen::Vector3d{-5.0, 0.0, 0.0},
//                 Eigen::Vector3d{1.0, 0.0, 0.0});

//             trajectory_a.insert(
//                 begin_time + 10s,
//                 make_test_profile(UnitCircle),
//                 Eigen::Vector3d{0.0, -5.0, 0.0},
//                 Eigen::Vector3d{0.0, -1.0, 0.0});
//             REQUIRE(trajectory_a.size() == 2);

//             WHEN("Checked against a conflicting Trajectory")
//             {
//                   rmf_traffic::Trajectory trajectory_b("test_map");
//                   trajectory_b.insert(
//                       begin_time,
//                       make_test_profile(UnitCircle),
//                       Eigen::Vector3d{-5.0, -5.0, 0.0},
//                       Eigen::Vector3d{1.0, 0.0, 0.0});

//                   trajectory_b.insert(
//                       begin_time + 10s,
//                       make_test_profile(UnitCircle),
//                       Eigen::Vector3d{0.0, 0.0, 0.0},
//                       Eigen::Vector3d{0.0, 1.0, 0.0});
//                   REQUIRE(trajectory_b.size() == 2);

//                   CHECK(rmf_traffic::DetectConflict::broad_phase(f
//                       trajectory_a, trajectory_b));

//                   const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
//                       trajectory_a, trajectory_b);
//                   REQUIRE(conflicts.size() == 1);

//                   // Note: The expected collision time, calculated by hand, is sqrt(30)
//                   const double expected_time = std::sqrt(30.0);
//                   const double computed_time = rmf_traffic::time::to_seconds(
//                       conflicts.front().get_time() - begin_time);

//                   CHECK(computed_time == Approx(expected_time).margin(0.2));
//             }

//             WHEN("Checked against a Trajectory that does not conflict")
//             {
//                   rmf_traffic::Trajectory trajectory_c("test_map");
//                   trajectory_c.insert(
//                       begin_time,
//                       make_test_profile(UnitCircle),
//                       Eigen::Vector3d{5.0, 0.0, 0.0},
//                       Eigen::Vector3d{-1.0, 0.0, 0.0});

//                   trajectory_c.insert(
//                       begin_time + 10s,
//                       make_test_profile(UnitCircle),
//                       Eigen::Vector3d{0.0, 5.0, 0.0},
//                       Eigen::Vector3d{0.0, 1.0, 0.0});
//                   REQUIRE(trajectory_c.size() == 2);

//                   CHECK(rmf_traffic::DetectConflict::broad_phase(
//                       trajectory_a, trajectory_c));

//                   const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
//                       trajectory_a, trajectory_c);

//                   CHECK(conflicts.empty());
//             }
//       }
// }

/// Remaining test suggestions:
/// * multi-segment trajectories
/// * trajectories that only partially overlap in time
/// * trajectories that do not overlap in time at all to test
///   DetectConflict::between() and DetectConflict::broad_phase()
/// * trajectories that include rotating boxes which collide due to rotating

// A useful website for playing with 2D cubic splines: https://www.desmos.com/calculator/
