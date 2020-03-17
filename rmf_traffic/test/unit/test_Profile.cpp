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
#include <rmf_traffic/geometry/Box.hpp>

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

    CHECK((footprint->get_characteristic_length() - 1.0) == Approx(0.0).margin(1e-6));
    CHECK((vicinity->get_characteristic_length() - 1.0) == Approx(0.0).margin(1e-6));

    // Footprint and vicinity are updated
    profile.vicinity(circle_2);
    profile.footprint(circle_2);

    CHECK((profile.footprint()->get_characteristic_length() - 2.0) == Approx(0.0).margin(1e-6));
    CHECK((profile.vicinity()->get_characteristic_length() - 2.0) == Approx(0.0).margin(1e-6));

  }

  WHEN ("Vicinity is passed into the constructor")
  {
    const auto circle_1 = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(1.0);
    const auto circle_2 = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(2.0);

    auto profile = Profile{circle_1, circle_2};

    auto& footprint = profile.footprint();
    auto& vicinity = profile.vicinity();

    CHECK((footprint->get_characteristic_length() - 1.0) == Approx(0.0).margin(1e-6));
    CHECK((vicinity->get_characteristic_length() - 2.0) == Approx(0.0).margin(1e-6));

    profile.footprint(circle_2);
    profile.vicinity(circle_1);

    CHECK((profile.footprint()->get_characteristic_length() - 2.0)
        == Approx(0.0).margin(1e-6));
    CHECK((profile.vicinity()->get_characteristic_length() - 1.0)
        == Approx(0.0).margin(1e-6));
  }

}

SCENARIO ("Testing conflicts")
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

  GIVEN("Overlapping footprints and vicinities")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {0, 2, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {10, 2, 0}, {0, 0, 0});

    CHECK(rmf_traffic::DetectConflict::between(
        {circle_1, circle_2},
        t1,
        {circle_1, circle_2},
        t2));
  }

  GIVEN("Footprint overlaps with vicinity")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {2.8, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {2.8, 0, 0}, {0, 0, 0});

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

  GIVEN("Overlapping box vicinity and circle footprint")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {2.3, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {2.3, 0, 0}, {0, 0, 0});

    CHECK(rmf_traffic::DetectConflict::between(
        {circle_1, box_3},
        t1,
        {circle_1, circle_1},
        t2));
  }

  GIVEN("Overlapping circle vicinity and box footprint")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {2.8, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {2.8, 0, 0}, {0, 0, 0});

    CHECK(rmf_traffic::DetectConflict::between(
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