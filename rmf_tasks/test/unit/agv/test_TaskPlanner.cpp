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


#include <rmf_tasks/agv/TaskPlanner.hpp>
#include <rmf_tasks/agv/State.hpp>
#include <rmf_tasks/agv/StateConfig.hpp>
#include <rmf_tasks/requests/Delivery.hpp>
#include <rmf_tasks/requests/ChargeBattery.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/BatterySystem.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

//==============================================================================
inline void display_solution(
  std::string parent,
  const rmf_tasks::agv::TaskPlanner::Assignments& assignments,
  const double cost)
{
  std::cout << parent << " cost: " << cost << std::endl;
  std::cout << parent << " assignments:" << std::endl;
  for (std::size_t i = 0; i < assignments.size(); ++i)
  {
    std:: cout << "--Agent: " << i << std::endl;
    for (const auto& a : assignments[i])
    {
      const auto& s = a.state();
      std::cout << "    <" << a.task_id() << ": " << 100* s.battery_soc() 
                << "%>" << std::endl;
    }
  }
  std::cout << " ----------------------" << std::endl;
}

//==============================================================================
SCENARIO("Grid World")
{
  const int grid_size = 4;
  const double edge_length = 1000;
  const bool drain_battery = true;

  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;

  rmf_traffic::agv::Graph graph;
  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
  {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
  };

  const std::string map_name = "test_map";

  for (int i = 0; i < grid_size; ++i)
  {
    for (int j = 0; j < grid_size; ++j)
    {
      // const auto random = (double) rand() / RAND_MAX;
      const double random = 1.0;
      graph.add_waypoint(map_name, 
        {j*edge_length*random, -i*edge_length*random});
    }
  }

  for (int i = 0; i < grid_size*grid_size; ++i)
  {
    if ((i+1) % grid_size != 0)
      add_bidir_lane(i, i+1);
    if (i + grid_size < grid_size*grid_size)
      add_bidir_lane(i, i+4);
  }

  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const rmf_traffic::Profile profile{shape, shape};
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, profile);
  rmf_traffic::schedule::Database database;
  const auto default_options = rmf_traffic::agv::Planner::Options{
    nullptr};
    
  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
      rmf_traffic::agv::Planner::Configuration{graph, traits},
      default_options);

  rmf_battery::agv::BatterySystem battery_system{24.0, 40.0, 8.8};
  REQUIRE(battery_system.valid());
  rmf_battery::agv::MechanicalSystem mechanical_system{70.0, 40.0, 0.22};
  REQUIRE(mechanical_system.valid());
  rmf_battery::agv::PowerSystem power_system{"processor", 20.0};
  REQUIRE(power_system.valid());

  std::shared_ptr<SimpleMotionPowerSink> motion_sink =
    std::make_shared<SimpleMotionPowerSink>(battery_system, mechanical_system);
  std::shared_ptr<SimpleDevicePowerSink> device_sink =
    std::make_shared<SimpleDevicePowerSink>(battery_system, power_system);

  auto charge_battery_task = rmf_tasks::requests::ChargeBattery::make(
    battery_system,
    motion_sink, device_sink,
    planner,
    drain_battery);
  
  WHEN("Planning for 3 requests and 2 agents")
  {
    const auto now = std::chrono::steady_clock::now();

    std::vector<rmf_tasks::agv::State> initial_states =
    {
      rmf_tasks::agv::State{13, 13},
      rmf_tasks::agv::State{2, 2}
    };

    std::vector<rmf_tasks::agv::StateConfig> state_configs =
    {
      rmf_tasks::agv::StateConfig{0.2},
      rmf_tasks::agv::StateConfig{0.2}
    };

    std::vector<rmf_tasks::Request::SharedPtr> requests =
    {
      rmf_tasks::requests::Delivery::make(
        1,
        0,
        3,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(0)),

      rmf_tasks::requests::Delivery::make(
        2,
        15,
        2,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(0)),

      rmf_tasks::requests::Delivery::make(
        3,
        7,
        9,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(0))
    };

    std::shared_ptr<rmf_tasks::agv::TaskPlanner::Configuration>  task_config =
      std::make_shared<rmf_tasks::agv::TaskPlanner::Configuration>(charge_battery_task);
    rmf_tasks::agv::TaskPlanner task_planner(task_config);

    const auto greedy_assignments = task_planner.greedy_plan(
      initial_states, state_configs, requests);
    const double greedy_cost = task_planner.compute_cost(greedy_assignments);

    const auto optimal_assignments = task_planner.optimal_plan(
      initial_states, state_configs, requests, nullptr);
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    
    display_solution("Greedy", greedy_assignments, greedy_cost);
    display_solution("Optimal", optimal_assignments, optimal_cost);

    REQUIRE(optimal_cost <= greedy_cost);
  }

  WHEN("Planning for 11 requests and 2 agents")
  {
    const auto now = std::chrono::steady_clock::now();

    std::vector<rmf_tasks::agv::State> initial_states =
    {
      rmf_tasks::agv::State{13, 13},
      rmf_tasks::agv::State{2, 2}
    };

    std::vector<rmf_tasks::agv::StateConfig> state_configs =
    {
      rmf_tasks::agv::StateConfig{0.2},
      rmf_tasks::agv::StateConfig{0.2}
    };

    std::vector<rmf_tasks::Request::SharedPtr> requests =
    {
      rmf_tasks::requests::Delivery::make(
        1,
        0,
        3,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(0)),

      rmf_tasks::requests::Delivery::make(
        2,
        15,
        2,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(0)),

      rmf_tasks::requests::Delivery::make(
        3,
        7,
        9,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(0)),

      rmf_tasks::requests::Delivery::make(
        3,
        7,
        9,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(0)),

      rmf_tasks::requests::Delivery::make(
        4,
        8,
        11,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(50000)),

      rmf_tasks::requests::Delivery::make(
        5,
        10,
        0,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(50000)),

      rmf_tasks::requests::Delivery::make(
        6,
        4,
        8,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_tasks::requests::Delivery::make(
        7,
        8,
        14,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_tasks::requests::Delivery::make(
        8,
        5,
        11,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_tasks::requests::Delivery::make(
        9,
        9,
        0,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_tasks::requests::Delivery::make(
        10,
        1,
        3,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(60000)),

      rmf_tasks::requests::Delivery::make(
        11,
        0,
        12,
        motion_sink,
        device_sink,
        planner,
        drain_battery,
        now + rmf_traffic::time::from_seconds(60000))
    };

    std::shared_ptr<rmf_tasks::agv::TaskPlanner::Configuration>  task_config =
      std::make_shared<rmf_tasks::agv::TaskPlanner::Configuration>(charge_battery_task);
    rmf_tasks::agv::TaskPlanner task_planner(task_config);

    const auto greedy_assignments = task_planner.greedy_plan(
      initial_states, state_configs, requests);
    const double greedy_cost = task_planner.compute_cost(greedy_assignments);

    const auto optimal_assignments = task_planner.optimal_plan(
      initial_states, state_configs, requests, nullptr);
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
  
    display_solution("Greedy", greedy_assignments, greedy_cost);
    display_solution("Optimal", optimal_assignments, optimal_cost);

    REQUIRE(optimal_cost <= greedy_cost);
  }
}