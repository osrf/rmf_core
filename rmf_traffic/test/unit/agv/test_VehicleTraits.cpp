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

#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rmf_utils/catch.hpp>

void test_independence(
  rmf_traffic::agv::VehicleTraits& original,
  rmf_traffic::agv::VehicleTraits& copy)
{
  // Check that the copy has the same properties as the original
  CHECK(copy.linear().get_nominal_velocity() == Approx(0.1));
  CHECK(copy.linear().get_nominal_acceleration() == Approx(0.2));
  CHECK(copy.rotational().get_nominal_velocity() == Approx(0.3));
  CHECK(copy.rotational().get_nominal_acceleration() == Approx(0.4));
  CHECK(!copy.get_differential()->is_reversible());

  copy.linear()
  .set_nominal_velocity(1.0)
  .set_nominal_acceleration(2.0);

  copy.rotational()
  .set_nominal_velocity(3.0)
  .set_nominal_acceleration(4.0);

  copy.get_differential()->set_reversible(true);

  // Check that we can succesfully change the properties of the copy
  CHECK(copy.linear().get_nominal_velocity() == Approx(1.0));
  CHECK(copy.linear().get_nominal_acceleration() == Approx(2.0));
  CHECK(copy.rotational().get_nominal_velocity() == Approx(3.0));
  CHECK(copy.rotational().get_nominal_acceleration() == Approx(4.0));
  CHECK(copy.get_differential()->is_reversible());

  // Check that changing the copy does not alter the original
  CHECK(original.linear().get_nominal_velocity() == Approx(0.1));
  CHECK(original.linear().get_nominal_acceleration() == Approx(0.2));
  CHECK(original.rotational().get_nominal_velocity() == Approx(0.3));
  CHECK(original.rotational().get_nominal_acceleration() == Approx(0.4));
  CHECK(!original.get_differential()->is_reversible());

  original.linear()
  .set_nominal_velocity(10.0)
  .set_nominal_acceleration(20.0);

  original.rotational()
  .set_nominal_velocity(30.0)
  .set_nominal_acceleration(40.0);

  original.get_differential()->set_reversible(true);

  // Check that we can still change the original
  CHECK(original.linear().get_nominal_velocity() == Approx(10.0));
  CHECK(original.linear().get_nominal_acceleration() == Approx(20.0));
  CHECK(original.rotational().get_nominal_velocity() == Approx(30.0));
  CHECK(original.rotational().get_nominal_acceleration() == Approx(40.0));
  CHECK(original.get_differential()->is_reversible());

  // Check that changing the original does impact the copy
  CHECK(copy.linear().get_nominal_velocity() == Approx(1.0));
  CHECK(copy.linear().get_nominal_acceleration() == Approx(2.0));
  CHECK(copy.rotational().get_nominal_velocity() == Approx(3.0));
  CHECK(copy.rotational().get_nominal_acceleration() == Approx(4.0));
  CHECK(copy.get_differential()->is_reversible());
}

SCENARIO("Test VehicleTraits")
{
  rmf_traffic::agv::VehicleTraits original(
    {0.0, 0.0}, {0.0, 0.0}, {nullptr, nullptr});

  // Check that the initial properties are all zero
  CHECK(original.linear().get_nominal_velocity() == Approx(0.0));
  CHECK(original.linear().get_nominal_acceleration() == Approx(0.0));
  CHECK(original.rotational().get_nominal_velocity() == Approx(0.0));
  CHECK(original.rotational().get_nominal_acceleration() == Approx(0.0));
  CHECK(original.get_differential()->is_reversible());

  original.linear()
  .set_nominal_velocity(0.1)
  .set_nominal_acceleration(0.2);

  original.rotational()
  .set_nominal_velocity(0.3)
  .set_nominal_acceleration(0.4);

  original.get_differential()->set_reversible(false);

  // Check that changing the properties worked
  CHECK(original.linear().get_nominal_velocity() == Approx(0.1));
  CHECK(original.linear().get_nominal_acceleration() == Approx(0.2));
  CHECK(original.rotational().get_nominal_velocity() == Approx(0.3));
  CHECK(original.rotational().get_nominal_acceleration() == Approx(0.4));
  CHECK(!original.get_differential()->is_reversible());

  SECTION("Copy constructor")
  {
    rmf_traffic::agv::VehicleTraits copy{original};
    test_independence(original, copy);
  }

  SECTION("Copy operator")
  {
    rmf_traffic::agv::VehicleTraits copy(
      {0.0, 0.0}, {0.0, 0.0}, {nullptr, nullptr});

    copy = original;
    test_independence(original, copy);
  }

  SECTION("Move constructor")
  {
    rmf_traffic::agv::VehicleTraits moved = original;
    rmf_traffic::agv::VehicleTraits copy = std::move(moved);
    moved = original;
    test_independence(moved, copy);
  }

  SECTION("Move operator")
  {
    rmf_traffic::agv::VehicleTraits moved = original;
    rmf_traffic::agv::VehicleTraits copy(
      {0.0, 0.0}, {0.0, 0.0}, {nullptr, nullptr});

    copy = std::move(moved);
    moved = original;
    test_independence(moved, copy);
  }
}
