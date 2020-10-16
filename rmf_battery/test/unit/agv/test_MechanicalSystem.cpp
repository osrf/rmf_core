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

#include <rmf_battery/agv/MechanicalSystem.hpp>

#include <rmf_utils/catch.hpp>



SCENARIO("Test MechanicalSystem")
{
  using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
  MechanicalSystem mechanical_system{70.0, 30.0, 0.3};
  REQUIRE(mechanical_system.valid());
  CHECK(mechanical_system.mass() - 70.0 == Approx(0.0));
  CHECK(mechanical_system.inertia() - 30.0 == Approx(0.0));
  CHECK(mechanical_system.friction_coefficient() - 0.3 == Approx(0.0));

  WHEN("Mass is set")
  {
    mechanical_system.mass(75.0);
    CHECK(mechanical_system.valid());
    CHECK(mechanical_system.mass() - 75.0 == Approx(0.0));
    CHECK(mechanical_system.inertia() - 30.0 == Approx(0.0));
    CHECK(mechanical_system.friction_coefficient() - 0.3 == Approx(0.0));
  }

  WHEN("Inertia is set")
  {
    mechanical_system.inertia(35.0);
    CHECK(mechanical_system.valid());
    CHECK(mechanical_system.mass() - 70.0 == Approx(0.0));
    CHECK(mechanical_system.inertia() - 35.0 == Approx(0.0));
    CHECK(mechanical_system.friction_coefficient() - 0.3 == Approx(0.0));
  }
    
  WHEN("Friction coefficient is set")
  {
    mechanical_system.friction_coefficient(0.4);
    CHECK(mechanical_system.valid());
    CHECK(mechanical_system.mass() - 70.0 == Approx(0.0));
    CHECK(mechanical_system.inertia() - 30.0 == Approx(0.0));
    CHECK(mechanical_system.friction_coefficient() - 0.4 == Approx(0.0));
  }

  WHEN("A negative value is set")
  {
    mechanical_system.mass(-10.0);
    CHECK_FALSE(mechanical_system.valid());
  }
}
