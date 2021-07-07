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

#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

//#include <rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/geometry/Box.hpp>

#include <rmf_utils/catch.hpp>

using namespace std::chrono_literals;

SCENARIO("Testing Construction")
{
  using Profile = rmf_traffic::Profile;

  WHEN("Vicinity is not passed into the constructor")
  {
    const auto circle_1 = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);
    const auto circle_2 = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(2.0);

    auto profile = Profile{circle_1};

    auto& footprint = profile.footprint();
    auto& vicinity = profile.vicinity();

    CHECK(footprint == circle_1);
    CHECK(vicinity == circle_1);
    CHECK(footprint->get_characteristic_length() == Approx(1.0));
    CHECK(vicinity->get_characteristic_length() == Approx(1.0));

    WHEN("Only footprint is updated")
    {
      profile.footprint(circle_2);
      CHECK(footprint == circle_2);
      CHECK(profile.footprint()->get_characteristic_length() == Approx(2.0));
      // profile.vicinity() should return footprint
      CHECK(profile.vicinity() == profile.footprint());
      CHECK(profile.vicinity()->get_characteristic_length() == Approx(2.0));
    }

    WHEN("Only vicinity is updated")
    {
      profile.vicinity(circle_2);
      CHECK(vicinity != profile.vicinity());
      CHECK(profile.vicinity() == circle_2);
      CHECK(footprint == circle_1);
      CHECK(profile.footprint()->get_characteristic_length() == Approx(1.0));
      CHECK(profile.vicinity()->get_characteristic_length() == Approx(2.0));
    }

    WHEN("Both footprint and vicinity are updated")
    {
      profile.footprint(circle_2);
      profile.vicinity(circle_2);
      CHECK(footprint == circle_2);
      CHECK(vicinity == circle_2);
      CHECK(profile.footprint()->get_characteristic_length() == Approx(2.0));
      CHECK(profile.vicinity()->get_characteristic_length() == Approx(2.0));
    }
  }

  WHEN("Vicinity is passed into the constructor")
  {
    const auto circle_1 = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);
    const auto circle_2 = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(2.0);

    auto profile = Profile{circle_1, circle_2};

    auto& footprint = profile.footprint();
    auto& vicinity = profile.vicinity();

    CHECK(footprint == circle_1);
    CHECK(vicinity == circle_2);
    CHECK(footprint->get_characteristic_length() == Approx(1.0));
    CHECK(vicinity->get_characteristic_length() == Approx(2.0));

    WHEN("Only footprint is updated")
    {
      profile.footprint(circle_2);
      CHECK(footprint == circle_2);
      CHECK(vicinity == circle_2);
      CHECK(profile.footprint()->get_characteristic_length() == Approx(2.0));
      CHECK(profile.vicinity()->get_characteristic_length() == Approx(2.0));
    }

    WHEN("Only vicinity is updated")
    {
      profile.vicinity(circle_1);
      CHECK(vicinity == circle_1);
      CHECK(footprint == circle_1);
      CHECK(profile.footprint()->get_characteristic_length() == Approx(1.0));
      CHECK(profile.vicinity()->get_characteristic_length() == Approx(1.0));
    }

    WHEN("Both footprint and vicinity are updated")
    {
      profile.footprint(circle_2);
      profile.vicinity(circle_1);
      CHECK(footprint == circle_2);
      CHECK(vicinity == circle_1);
      CHECK(profile.footprint()->get_characteristic_length() == Approx(2.0));
      CHECK(profile.vicinity()->get_characteristic_length() == Approx(1.0));
    }
  }

  WHEN("Additional footprint shapes are added")
  {
    const auto circle_1 = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);
    const auto circle_2 = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(2.0);

    auto profile = Profile{circle_1, circle_2};
    profile.add_extra_footprint(circle_1, Eigen::Vector3d(0, 2, 0));
    CHECK(profile.extra_footprint_count() == 1);

    profile.clear_extra_footprints();
    CHECK(profile.extra_footprint_count() == 0);
  }
}

SCENARIO("Testing conflicts", "[close_start]")
{
  using Trajecotry = rmf_traffic::Trajectory;

  const auto start_time = std::chrono::steady_clock::now();

  const auto circle_1 = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const auto circle_2 = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(2.0);
  const auto box_1 = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(1.0, 1.0);
  const auto box_2 = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(2.0, 2.0);
  const auto box_3 = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(3.0, 3.0);

  GIVEN("Two stationary trajectories with non-overlapping vicinities")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {5, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {5, 0, 0}, {0, 0, 0});

    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {circle_1},
        t1,
        {circle_1},
        t2));
  }

  GIVEN("Two moving trajectories with non-overlapping vicinities")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {0, 10, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {10, 10, 0}, {0, 0, 0});

    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {circle_1},
        t1,
        {circle_1},
        t2));
  }

  GIVEN("Overlapping vicinities but not footprints")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {0, 3.8, 0}, {10, 3.8, 0});
    t2.insert(start_time + 10s, {10, 10, 0}, {0, 0, 0});

    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {circle_1, circle_2},
        t1,
        {circle_1, circle_2},
        t2));
  }

  GIVEN("Footprint overlaps with vicinity from the start")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {2.8, 0, 0}, {0, 0, 0});

    const rmf_traffic::Profile profile{circle_1, circle_2};

    // When a robot starts in another's vicinity, a conflict only happens if the
    // robots move closer to each other.

    WHEN("Vehicles sit still")
    {
      t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {2.8, 0, 0}, {0, 0, 0});
      CHECK_FALSE(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }

    WHEN("Vehicles move in parallel")
    {
      t1.insert(start_time + 10s, {0, 10, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {2.8, 10, 0}, {0, 0, 0});
      CHECK_FALSE(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }

    WHEN("One vehicle moves away")
    {
      t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});
      CHECK_FALSE(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }

    WHEN("Vehicles move apart slightly")
    {
      t1.insert(start_time + 10s, {0, 10, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {3, 10, 0}, {0, 0, 0});
      CHECK_FALSE(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }

    WHEN("Vehicles move apart orthogonally")
    {
      t1.insert(start_time + 10s, {0, 10, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});
      CHECK_FALSE(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }

    WHEN("Vehicles move apart in opposite shearing directions")
    {
      t1.insert(start_time + 10s, {0, 10, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {2.8, -10, 0}, {0, 0, 0});
      CHECK_FALSE(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }

    WHEN("Vehicles move directly apart from each other")
    {
      t1.insert(start_time + 10s, {-10, 0, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});
      CHECK_FALSE(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }

    WHEN("Vehicles move directly across each other")
    {
      t1.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {-10, 0, 0}, {0, 0, 0});
      CHECK(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }

    WHEN("Vehicles move a little closer")
    {
      t1.insert(start_time + 10s, {0, 10, 0}, {0, 0, 0});
      t2.insert(start_time + 10s, {2.79, 10, 0}, {0, 0, 0});
      CHECK(rmf_traffic::DetectConflict::between(
          profile, t1, profile, t2));
    }
  }

  GIVEN("Footprint overlaps with vicinity and then leaves")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {2.8, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {5.0, 0, 0}, {0, 0, 0});

    // A conflict only exists if the footprint entered the vicinity after the
    // start of the trajectory
    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {circle_1, circle_2},
        t1,
        {circle_1},
        t2));
  }

  GIVEN("Footprint overlaps with vicinity at the end, but not at the start")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {5.0, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {2.8, 0, 0}, {0, 0, 0});

    // Since the footprint started outside the other's vicinity, it will trigger
    // a conflict when it enters.
    CHECK(rmf_traffic::DetectConflict::between(
        {circle_1, circle_2},
        t1,
        {circle_1},
        t2));
  }

  GIVEN("Overlapping box vicinities")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {2.8, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {2.8, 0, 0}, {0, 0, 0});

    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {circle_1, box_3},
        t1,
        {circle_1, box_3},
        t2));
  }

  GIVEN("Overlapping box vicinity and circle footprint from the start")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {2.3, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {2.3, 0, 0}, {0, 0, 0});

    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {circle_1, box_3},
        t1,
        {circle_1, circle_1},
        t2));
  }

  GIVEN("Overlapping circle vicinity and box footprint from the start")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {2.8, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {2.8, 0, 0}, {0, 0, 0});

    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {box_1, circle_2},
        t1,
        {box_2, box_2},
        t2));
  }

  GIVEN("Overlapping box footprints")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {1.8, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {1.8, 0, 0}, {0, 0, 0});

    CHECK(rmf_traffic::DetectConflict::between(
        {box_2, box_2},
        t1,
        {box_2, box_2},
        t2));
  }
}
