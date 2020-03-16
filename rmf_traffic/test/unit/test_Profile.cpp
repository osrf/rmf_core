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

#include <rmf_utils/catch.hpp>

using namespace std::chrono_literals;

SCENARIO("Testing Construction")
{
  using Profile = rmf_traffic::Profile;

  WHEN("Vicinity is not passed into the constructor")
  {
    const auto shape_1 = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(1.0);
    const auto shape_2 = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(2.0);

    auto profile = Profile{shape_1};

    auto& footprint = profile.footprint();
    auto& vicinity = profile.vicinity();

    CHECK((footprint->get_characteristic_length() - 1.0) == Approx(0.0).margin(1e-6));
    CHECK((vicinity->get_characteristic_length() - 1.0) == Approx(0.0).margin(1e-6));

    // Footprint and vicinity are updated
    profile.vicinity(shape_2);
    profile.footprint(shape_2);

    CHECK((profile.footprint()->get_characteristic_length() - 2.0) == Approx(0.0).margin(1e-6));
    CHECK((profile.vicinity()->get_characteristic_length() - 2.0) == Approx(0.0).margin(1e-6));

  }

  WHEN ("Vicinity is passed into the constructor")
  {
    const auto shape_1 = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(1.0);
    const auto shape_2 = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(2.0);

    auto profile = Profile{shape_1, shape_2};

    auto& footprint = profile.footprint();
    auto& vicinity = profile.vicinity();

    CHECK((footprint->get_characteristic_length() - 1.0) == Approx(0.0).margin(1e-6));
    CHECK((vicinity->get_characteristic_length() - 2.0) == Approx(0.0).margin(1e-6));

    profile.footprint(shape_2);
    profile.vicinity(shape_1);

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

  const auto shape_1 = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);
  const auto shape_2 = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(2.0);

  GIVEN("Two stationary trajectories with non-overlapping vicinities")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {0, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {5, 0, 0}, {0, 0, 0});
    t2.insert(start_time + 10s, {5, 0, 0}, {0, 0, 0});

    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {shape_1},
        t1,
        {shape_1},
        t2));
  }

  GIVEN("Two moving trajectories with non-overlapping vicinities")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {0, 10, 0}, {10, 10, 0});
    t2.insert(start_time + 10s, {10, 10, 0}, {0, 0, 0});

    CHECK_FALSE(rmf_traffic::DetectConflict::between(
        {shape_1},
        t1,
        {shape_1},
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
        {shape_1, shape_2},
        t1,
        {shape_1, shape_2},
        t2));
  }

  GIVEN("Overlapping footprints and vicinities")
  {
    Trajecotry t1;
    t1.insert(start_time, {0, 0, 0}, {0, 0, 0});
    t1.insert(start_time + 10s, {10, 0, 0}, {0, 0, 0});

    Trajecotry t2;
    t2.insert(start_time, {0, 2, 0}, {10, 2, 0});
    t2.insert(start_time + 10s, {10, 10, 0}, {0, 0, 0});

    CHECK(rmf_traffic::DetectConflict::between(
        {shape_1, shape_2},
        t1,
        {shape_1, shape_2},
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
        {shape_1, shape_2},
        t1,
        {shape_1},
        t2));
  }
  
}