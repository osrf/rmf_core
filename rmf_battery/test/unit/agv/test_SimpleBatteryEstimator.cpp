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

#include <rmf_battery/agv/SimpleBatteryEstimator.hpp>
#include <rmf_battery/agv/SystemTraits.hpp>

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

SCENARIO("Test SimpleBatteryEstimator")
{
    using SystemTraits = rmf_battery::agv::SystemTraits;
    using SimpleBatteryEstimator = rmf_battery::agv::SimpleBatteryEstimator;
    using namespace std::chrono_literals;

    // Initializing system traits
    SystemTraits::BatterySystem battery_system{12, 20, 2};
    REQUIRE(battery_system.valid());
    SystemTraits::MechanicalSystem mechanical_system{20, 10, 0.03};
    REQUIRE(mechanical_system.valid());
    SystemTraits::PowerSystem power_system{100, 24};
    REQUIRE(power_system.valid());
    SystemTraits system_traits{
    mechanical_system, battery_system, {power_system}};
    REQUIRE(system_traits.valid());

    auto battery_estimator = SimpleBatteryEstimator{system_traits};

    // Initializing vehicle traits
    const rmf_traffic::agv::VehicleTraits traits(
        {0.7, 0.5}, {0.3, 0.25}, {nullptr, nullptr});    
    
    WHEN("Robot moves 100m along a straight line")
    {
      const auto start_time = std::chrono::steady_clock::now();
      const std::vector<Eigen::Vector3d> positions = {
          Eigen::Vector3d{0.0, 0.0, 0.0},
          Eigen::Vector3d{100, 0.0, 0.0},
      };
      rmf_traffic::Trajectory trajectory =
        rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

      auto remaining_soc = battery_estimator.compute_state_of_charge(
          trajectory, 1.0);
      REQUIRE(remaining_soc > 0.99);
    }
}