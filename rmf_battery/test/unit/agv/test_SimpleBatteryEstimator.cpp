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
    SystemTraits::MechanicalSystem mechanical_system{60, 10, 0.3};
    REQUIRE(mechanical_system.valid());
    SystemTraits::PowerSystem power_system{100, 24};
    REQUIRE(power_system.valid());
    SystemTraits system_traits{
    mechanical_system, battery_system, {power_system}};
    REQUIRE(system_traits.valid());

    auto battery_estimator = SimpleBatteryEstimator{system_traits};

    WHEN("Robot moves 100m along a straight line")
    {
        rmf_traffic::Trajectory trajectory;
        const auto start_time = std::chrono::steady_clock::now();
        trajectory.insert(start_time, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
        trajectory.insert(start_time + 50s, {50.0, 0.0, 0.0}, {0.5, 0.0, 0.0});
        trajectory.insert(start_time + 100s, {100.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
        REQUIRE(trajectory.size() == 3);

        auto remaining_soc = battery_estimator.compute_state_of_charge(trajectory, 0.9);
        std::cout << "Start time: " << start_time.time_since_epoch().count() << " soc: " << remaining_soc << std::endl;
    }
}