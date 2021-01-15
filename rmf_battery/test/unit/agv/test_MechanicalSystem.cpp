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
  
  WHEN("Valid values are supplied to make()")
  {
    auto mechanical_system = MechanicalSystem::make(10.0, 20.0, 0.3);
    REQUIRE(mechanical_system);
    CHECK(mechanical_system->mass() == 10.0);
    CHECK(mechanical_system->moment_of_inertia() == 20.0);
    CHECK(mechanical_system->friction_coefficient() == 0.3);
  }

  WHEN("In-valid mass is supplied to make()")
  {
    auto mechanical_system = MechanicalSystem::make(-10.0, 20.0, 0.3);
    CHECK_FALSE(mechanical_system);
  }

  WHEN("In-valid inertia is supplied to make()")
  {
    auto mechanical_system = MechanicalSystem::make(10.0, -20.0, 0.3);
    CHECK_FALSE(mechanical_system);
  }

  WHEN("In-valid friction coefficient is supplied to make()")
  {
    auto mechanical_system = MechanicalSystem::make(10.0, 20.0, -0.3);
    CHECK_FALSE(mechanical_system);
  }

  WHEN("In-valid values are supplied to make()")
  {
    auto mechanical_system = MechanicalSystem::make(-10.0, -20.0, -0.3);
    CHECK_FALSE(mechanical_system);
  }
}
