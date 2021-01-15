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

#include <rmf_battery/agv/BatterySystem.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test BatterySystem")
{
  using BatterySystem = rmf_battery::agv::BatterySystem;

  WHEN("Valid values are supplied to make()")
  {
    auto battery_system = BatterySystem::make(12.0, 20.0, 6.0);
    REQUIRE(battery_system);
    CHECK(battery_system->nominal_voltage() == 12.0);
    CHECK(battery_system->capacity() == 20.0);
    CHECK(battery_system->charging_current() == 6.0);
  }

  WHEN("In-valid values are supplied to make()")
  {
    auto battery_system = BatterySystem::make(-12.0, 20.0, 6.0);
    CHECK_FALSE(battery_system);
  }
}
