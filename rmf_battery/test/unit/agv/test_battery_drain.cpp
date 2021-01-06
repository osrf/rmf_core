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

#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/MechanicalSystem.hpp>
#include <rmf_battery/agv/PowerSystem.hpp>


#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test battery drain with RobotA")
{
  using BatterySystem = rmf_battery::agv::BatterySystem;
  using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
  using PowerSystem = rmf_battery::agv::PowerSystem;
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;
  using namespace std::chrono_literals;

  // Initializing system traits
  auto battery_system_optional = BatterySystem::make(12.0, 24.0, 2.0);
  REQUIRE(battery_system_optional);
  const BatterySystem& battery_system = *battery_system_optional;

  auto mechanical_system_optional = MechanicalSystem::make(20.0, 10.0, 0.3);
  REQUIRE(mechanical_system_optional);
  const MechanicalSystem& mechanical_system = *mechanical_system_optional;

  auto power_system_optional = PowerSystem::make(10.0);
  REQUIRE(power_system_optional);
  const PowerSystem& power_system_processor = *power_system_optional;
  
  const SimpleMotionPowerSink motion_power_sink{battery_system, mechanical_system};
  const SimpleDevicePowerSink device_power_sink{battery_system, power_system_processor};
  

  // Initializing vehicle traits
  const rmf_traffic::agv::VehicleTraits traits(
    {0.7, 0.5}, {0.3, 0.25}, {nullptr, nullptr});

  const double initial_soc = 1.0;

  WHEN("Robot moves 100m along a straight line")
  {
    const auto start_time = std::chrono::steady_clock::now();
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{100, 0.0, 0.0},
    };
    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    const double dSOC_motion = motion_power_sink.compute_change_in_charge(
      trajectory);
    const double dSOC_device = device_power_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(trajectory.duration()));

    const double remaining_soc = initial_soc - dSOC_motion - dSOC_device;
    // We expect less than 1% depletion of the battery level after this robot 
    // travels a distance of 100m.
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

    const double dSOC_motion = motion_power_sink.compute_change_in_charge(
      trajectory);
    const double dSOC_device = device_power_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(trajectory.duration()));

    const double remaining_soc = initial_soc - dSOC_motion - dSOC_device;
    // We expect the robot's battery to be completely drained as the mileage
    // for this robot is around 15km.
    const bool ok = std::abs(remaining_soc) < 0.06;
    CHECK(ok);
  }
}

SCENARIO("Test SimpleBatteryEstimator with RobotB")
{
  using BatterySystem = rmf_battery::agv::BatterySystem;
  using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
  using PowerSystem = rmf_battery::agv::PowerSystem;
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;
  using namespace std::chrono_literals;

  // Initializing system traits
  auto battery_system_optional = BatterySystem::make(24.0, 40.0, 8.8);
  REQUIRE(battery_system_optional);
  const BatterySystem& battery_system = *battery_system_optional;

  auto mechanical_system_optional = MechanicalSystem::make(70.0, 40.0, 0.22);
  REQUIRE(mechanical_system_optional);
  const MechanicalSystem& mechanical_system = *mechanical_system_optional;

  auto power_system_optional = PowerSystem::make(20.0);
  REQUIRE(power_system_optional);
  const PowerSystem& power_system_processor = *power_system_optional;

  const SimpleMotionPowerSink motion_power_sink{battery_system, mechanical_system};
  const SimpleDevicePowerSink device_power_sink{battery_system, power_system_processor};
  
  // Initializing vehicle traits
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, {nullptr, nullptr});

  const double initial_soc = 1.0;

  WHEN("Robot moves 100m along a straight line")
  {
    const auto start_time = std::chrono::steady_clock::now();
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{100, 0.0, 0.0},
    };
    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    const double dSOC_motion = motion_power_sink.compute_change_in_charge(
      trajectory);
    const double dSOC_device = device_power_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(trajectory.duration()));

    const double remaining_soc = initial_soc - dSOC_motion - dSOC_device;
    // We expect less than 2% depletion of the battery level after this robot 
    // travels a distance of 100m.
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

    const double dSOC_motion = motion_power_sink.compute_change_in_charge(
      trajectory);
    const double dSOC_device = device_power_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(trajectory.duration()));

    const double remaining_soc = initial_soc - dSOC_motion - dSOC_device;
    // We expect the robot's battery to be completely drained as the mileage
    // for this robot is around 20km.
    const bool ok = std::abs(remaining_soc) < 0.06;
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

    const double dSOC_motion = motion_power_sink.compute_change_in_charge(
      trajectory);
    const double dSOC_device = device_power_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(trajectory.duration()));

    const double remaining_soc = initial_soc - dSOC_motion - dSOC_device;
    // We expect the robot's battery to be completely drained as the mileage
    // for this robot is around 20km.
    const bool ok = std::abs(remaining_soc) < 0.06;
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

    const double dSOC_motion = motion_power_sink.compute_change_in_charge(
      trajectory);
    const double dSOC_device = device_power_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(trajectory.duration()));

    const double remaining_soc = initial_soc - dSOC_motion - dSOC_device;
    // We expect the robot's battery to be completely drained as the mileage
    // for this robot is around 20km.
    const bool ok = std::abs(remaining_soc) < 0.06;
    CHECK(ok);
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

    const double dSOC_motion = motion_power_sink.compute_change_in_charge(
      trajectory);
    const double dSOC_device = device_power_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(trajectory.duration()));

    const double remaining_soc = initial_soc - dSOC_motion - dSOC_device;
    // We expect less than 1% battery drain for this robot when it turns on the
    // spot for 5s.
    const bool ok = remaining_soc > 0.99 && remaining_soc < 1.0;
    CHECK(ok);
  }
}

SCENARIO("Testing Cleaning Request")
{
  using BatterySystem = rmf_battery::agv::BatterySystem;
  using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
  using PowerSystem = rmf_battery::agv::PowerSystem;
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;
  using namespace std::chrono_literals;

  // Initializing system traits
  auto battery_system_optional = BatterySystem::make(24.0, 40.0, 8.8);
  REQUIRE(battery_system_optional);
  const BatterySystem& battery_system = *battery_system_optional;

  auto mechanical_system_optional = MechanicalSystem::make(70.0, 40.0, 0.22);
  REQUIRE(mechanical_system_optional);
  const MechanicalSystem& mechanical_system = *mechanical_system_optional;

  auto power_system_optional = PowerSystem::make(20.0);
  REQUIRE(power_system_optional);
  const PowerSystem& power_system_processor = *power_system_optional;

  const SimpleMotionPowerSink motion_power_sink{battery_system, mechanical_system};
  const SimpleDevicePowerSink device_power_sink{battery_system, power_system_processor};
  
  // Initializing vehicle traits
  const rmf_traffic::agv::VehicleTraits traits(
    {0.7, 0.5}, {0.4, 1.0}, {nullptr, nullptr});

  const double initial_soc = 1.0;

  WHEN("Computing invariant drain for zone_3")
  {
    const auto start_time = std::chrono::steady_clock::now();
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{104.0, -46.92, -M_PI/2.0},
      Eigen::Vector3d{104.0, -48.55, 0.0},
      Eigen::Vector3d{159.8, -48.38, M_PI/2.0},
      Eigen::Vector3d{159.8, -46.73, M_PI},
      Eigen::Vector3d{105.4, -47.04, M_PI/2.0},
      Eigen::Vector3d{105.5, -45.37, 0.0},
      Eigen::Vector3d{159.8, -45.25, 0.0},
      Eigen::Vector3d{155.0, -46.79, -M_PI/2.0},
    };
    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    const double dSOC_motion = motion_power_sink.compute_change_in_charge(
      trajectory);
    const double dSOC_device = device_power_sink.compute_change_in_charge(
      rmf_traffic::time::to_seconds(trajectory.duration()));

    const double remaining_soc = initial_soc - dSOC_motion - dSOC_device;
    // TODO(YV): Determine the theoretical drain for this trajectory.
    REQUIRE(remaining_soc <= 1.0);
  }
}
