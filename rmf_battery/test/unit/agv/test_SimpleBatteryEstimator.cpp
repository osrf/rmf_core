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
#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

SCENARIO("Test SimpleBatteryEstimator with RobotA")
{
  using SystemTraits = rmf_battery::agv::SystemTraits;
  using SimpleBatteryEstimator = rmf_battery::agv::SimpleBatteryEstimator;
  using PowerMap = rmf_battery::EstimateBattery::PowerMap;
  using namespace std::chrono_literals;

  // Initializing system traits
  SystemTraits::BatterySystem battery_system{12, 24, 2};
  REQUIRE(battery_system.valid());
  SystemTraits::MechanicalSystem mechanical_system{20, 10, 0.3};
  REQUIRE(mechanical_system.valid());
  SystemTraits::PowerSystem power_system_1{"processor", 10, 5};
  REQUIRE(power_system_1.valid());
  SystemTraits::PowerSystems power_systems;
  power_systems.insert({power_system_1.name(), power_system_1});
  SystemTraits system_traits{
    mechanical_system, battery_system, power_systems};
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

    rmf_utils::optional<PowerMap> power_map = PowerMap();
    power_map.value().insert({"processor", trajectory});

    auto remaining_soc = battery_estimator.compute_state_of_charge(
      trajectory, 1.0, power_map);

    std::cout << "Remaining soc: " << remaining_soc << std::endl;
    const bool ok = remaining_soc > 0.99 && remaining_soc < 1.0;
    CHECK(ok);
  }

  WHEN("Robot moves 15km along a straight line")
  {
    const auto start_time = std::chrono::steady_clock::now();
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{15000, 0.0, 0.0},
    };
    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    rmf_utils::optional<PowerMap> power_map = PowerMap();
    power_map.value().insert({"processor", trajectory});

    auto remaining_soc = battery_estimator.compute_state_of_charge(
      trajectory, 1.0, power_map);

    std::cout << "Remaining soc: " << remaining_soc << std::endl;
    const bool ok = remaining_soc > -1 && remaining_soc < 0.05;
    CHECK(ok);
  }
}

SCENARIO("Test SimpleBatteryEstimator with RobotB")
{
  using SystemTraits = rmf_battery::agv::SystemTraits;
  using SimpleBatteryEstimator = rmf_battery::agv::SimpleBatteryEstimator;
  using PowerMap = rmf_battery::EstimateBattery::PowerMap;
  using namespace std::chrono_literals;

  // Initializing system traits
  SystemTraits::BatterySystem battery_system{24, 40, 2};
  REQUIRE(battery_system.valid());
  SystemTraits::MechanicalSystem mechanical_system{70, 40, 0.2};
  REQUIRE(mechanical_system.valid());
  SystemTraits::PowerSystem power_system_1{"processor", 20, 5};
  REQUIRE(power_system_1.valid());
  SystemTraits::PowerSystems power_systems;
  power_systems.insert({power_system_1.name(), power_system_1});
  SystemTraits system_traits{
    mechanical_system, battery_system, power_systems};
  REQUIRE(system_traits.valid());

  auto battery_estimator = SimpleBatteryEstimator{system_traits};

  // Initializing vehicle traits
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, {nullptr, nullptr});

  WHEN("Robot moves 100m along a straight line")
  {
    const auto start_time = std::chrono::steady_clock::now();
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{100, 0.0, 0.0},
    };
    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    rmf_utils::optional<PowerMap> power_map = PowerMap();
    power_map.value().insert({"processor", trajectory});

    auto remaining_soc = battery_estimator.compute_state_of_charge(
      trajectory, 1.0, power_map);

    std::cout << "Remaining soc: " << remaining_soc << std::endl;
    const bool ok = remaining_soc > 0.98 && remaining_soc < 1.0;
    CHECK(ok);
  }

  WHEN("Robot moves 20km along a horizontal straight line")
  {
    const auto start_time = std::chrono::steady_clock::now();
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{20000, 0.0, 0.0},
    };
    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    rmf_utils::optional<PowerMap> power_map = PowerMap();
    power_map.value().insert({"processor", trajectory});

    auto remaining_soc = battery_estimator.compute_state_of_charge(
      trajectory, 1.0, power_map);

    std::cout << "Remaining soc: " << remaining_soc << std::endl;
    const bool ok = remaining_soc > -1.0 && remaining_soc < 0.10;
    CHECK(ok);
  }

  WHEN("Robot moves 20km along a vertical straight line")
  {
    const auto start_time = std::chrono::steady_clock::now();
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 20000.0, 0.0},
    };
    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    rmf_utils::optional<PowerMap> power_map = PowerMap();
    power_map.value().insert({"processor", trajectory});

    auto remaining_soc = battery_estimator.compute_state_of_charge(
      trajectory, 1.0, power_map);

    std::cout << "Remaining soc: " << remaining_soc << std::endl;
    const bool ok = remaining_soc > -1.0 && remaining_soc < 0.10;
    CHECK(ok);
  }

  WHEN("Robot moves 20km along a square perimeter")
  {
    const auto start_time = std::chrono::steady_clock::now();
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{5000, 0.0, 0.0},
      Eigen::Vector3d{5000, 5000.0, 0.0},
      Eigen::Vector3d{0.0, 5000.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 0.0}
    };
    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    rmf_utils::optional<PowerMap> power_map = PowerMap();
    power_map.value().insert({"processor", trajectory});

    auto remaining_soc = battery_estimator.compute_state_of_charge(
      trajectory, 1.0, power_map);

    std::cout << "Remaining soc: " << remaining_soc << std::endl;
  }

  WHEN("Testing turning on a spot")
  {
    const auto start_time = std::chrono::steady_clock::now();
    rmf_traffic::Trajectory trajectory;
    trajectory.insert(
      start_time,
      {0, 0, 0},
      {0, 0, 0});
    trajectory.insert(
      start_time + 5s,
      {5, 0, M_PI},
      {0, 0, 0});
    REQUIRE(trajectory.size() == 2);

    rmf_utils::optional<PowerMap> power_map = PowerMap();
    power_map.value().insert({"processor", trajectory});

    auto remaining_soc = battery_estimator.compute_state_of_charge(
      trajectory, 1.0, power_map);

    std::cout << "Remaining soc: " << remaining_soc << std::endl;
    const bool ok = remaining_soc > 0.99 && remaining_soc < 1.0;
    CHECK(ok);
  }
}
