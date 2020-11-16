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


#include <rmf_task/agv/TaskPlanner.hpp>
#include <rmf_task/agv/State.hpp>
#include <rmf_task/agv/StateConfig.hpp>
#include <rmf_task/requests/Delivery.hpp>
#include <rmf_task/requests/ChargeBattery.hpp>

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
inline bool check_implicit_charging_task_start(
  const rmf_task::agv::TaskPlanner::Assignments& assignments,
  const double initial_soc)
{
  bool implicit_charging_task_added = false;
  for (const auto& agent : assignments)
  {
    if (!agent.size())
    {
      continue;
    }

    const auto& s = agent[0].state();
    auto is_charge_request =
      std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
        agent[0].request());

    // No task should consume more charge than (1.0 - initial_soc)
    // in the current test, so we are guaranted to find any occurrence
    // of an implicit charging task.
    if (!is_charge_request && s.battery_soc() > initial_soc)
    {
      implicit_charging_task_added = true;
      break;
    }
  }

  return implicit_charging_task_added;
}

//==============================================================================
inline void display_solution(
  std::string parent,
  const rmf_task::agv::TaskPlanner::Assignments& assignments,
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
      const double request_seconds = a.request()->earliest_start_time().time_since_epoch().count()/1e9;
      const double start_seconds = a.deployment_time().time_since_epoch().count()/1e9;
      const rmf_traffic::Time finish_time = s.finish_time();
      const double finish_seconds = finish_time.time_since_epoch().count()/1e9;
      std::cout << "    <" << a.request()->id() << ": " << request_seconds
                << ", " << start_seconds 
                << ", "<< finish_seconds << ", " << 100* s.battery_soc() 
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

  WHEN("Planning for 3 requests and 2 agents")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
      rmf_task::agv::State{second_location, 2, 1.0}
    };

    std::vector<rmf_task::agv::StateConfig> state_configs =
    {
      rmf_task::agv::StateConfig{0.2},
      rmf_task::agv::StateConfig{0.2}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        "1",
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "2",
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "3",
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery)
    };

    std::shared_ptr<rmf_task::agv::TaskPlanner::Configuration>  task_config =
      std::make_shared<rmf_task::agv::TaskPlanner::Configuration>(
        battery_system,
        motion_sink,
        device_sink,
        planner);
    rmf_task::agv::TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_assignments = task_planner.greedy_plan(
      now, initial_states, state_configs, requests);
    const double greedy_cost = task_planner.compute_cost(greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    std::cout << "Greedy solution found in: "
              << (finish_time - start_time).count() / 1e9 << std::endl;
    display_solution("Greedy", greedy_assignments, greedy_cost);

    // Create new TaskPlanner to reset cache so that measured run times
    // remain independent of one another
    task_planner = rmf_task::agv::TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_assignments = task_planner.optimal_plan(
      now, initial_states, state_configs, requests, nullptr);
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    std::cout << "Optimal solution found in: "
              << (finish_time - start_time).count() / 1e9 << std::endl;
    display_solution("Optimal", optimal_assignments, optimal_cost);

    REQUIRE(optimal_cost <= greedy_cost);
  }

  WHEN("Planning for 11 requests and 2 agents")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, 1.0},
      rmf_task::agv::State{second_location, 2, 1.0}
    };

    std::vector<rmf_task::agv::StateConfig> state_configs =
    {
      rmf_task::agv::StateConfig{0.2},
      rmf_task::agv::StateConfig{0.2}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        "1",
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "2",
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "3",
        7,
        "dispenser",
        9,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "4",
        8,
        "dispenser",
        11,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(50000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "5",
        10,
        "dispenser",
        0,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(50000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "6",
        4,
        "dispenser",
        8,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(60000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "7",
        8,
        "dispenser",
        14,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(60000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "8",
        5,
        "dispenser",
        11,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(60000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "9",
        9,
        "dispenser",
        0,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(60000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "10",
        1,
        "dispenser",
        3,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(60000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "11",
        0,
        "dispenser",
        12,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(60000),
        drain_battery)
    };

    std::shared_ptr<rmf_task::agv::TaskPlanner::Configuration>  task_config =
      std::make_shared<rmf_task::agv::TaskPlanner::Configuration>(
        battery_system,
        motion_sink,
        device_sink,
        planner);
    rmf_task::agv::TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_assignments = task_planner.greedy_plan(
      now, initial_states, state_configs, requests);
    const double greedy_cost = task_planner.compute_cost(greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    std::cout << "Greedy solution found in: "
              << (finish_time - start_time).count() / 1e9 << std::endl;
    display_solution("Greedy", greedy_assignments, greedy_cost);

    task_planner = rmf_task::agv::TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_assignments = task_planner.optimal_plan(
      now, initial_states, state_configs, requests, nullptr);
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    std::cout << "Optimal solution found in: "
              << (finish_time - start_time).count() / 1e9 << std::endl;
    display_solution("Optimal", optimal_assignments, optimal_cost);

    REQUIRE(optimal_cost <= greedy_cost);
  }

  WHEN("Initial charge is low")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;
    const double initial_soc = 0.3;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 13, initial_soc},
      rmf_task::agv::State{second_location, 2, initial_soc}
    };

    std::vector<rmf_task::agv::StateConfig> state_configs =
    {
      rmf_task::agv::StateConfig{0.2},
      rmf_task::agv::StateConfig{0.2}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        "1",
        0,
        "dispenser",
        3,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "2",
        15,
        "dispenser",
        2,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "3",
        9,
        "dispenser",
        4,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "4",
        8,
        "dispenser",
        11,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(50000),
        drain_battery)
    };

    std::shared_ptr<rmf_task::agv::TaskPlanner::Configuration>  task_config =
      std::make_shared<rmf_task::agv::TaskPlanner::Configuration>(
        battery_system,
        motion_sink,
        device_sink,
        planner);
    rmf_task::agv::TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_assignments = task_planner.greedy_plan(
      now, initial_states, state_configs, requests);
    const double greedy_cost = task_planner.compute_cost(greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    std::cout << "Greedy solution found in: "
              << (finish_time - start_time).count() / 1e9 << std::endl;
    display_solution("Greedy", greedy_assignments, greedy_cost);

    task_planner = rmf_task::agv::TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_assignments = task_planner.optimal_plan(
      now, initial_states, state_configs, requests, nullptr);
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    std::cout << "Optimal solution found in: "
              << (finish_time - start_time).count() / 1e9 << std::endl;
    display_solution("Optimal", optimal_assignments, optimal_cost);


    REQUIRE(optimal_cost <= greedy_cost);

    // Checks if Assignments take into account a charging task in the beginning
    // without explicitly including the task in Assignments.
    bool implicit_charging_task_added = check_implicit_charging_task_start(
      greedy_assignments, initial_soc);
    REQUIRE(!implicit_charging_task_added);

    implicit_charging_task_added = check_implicit_charging_task_start(
      optimal_assignments, initial_soc);
    REQUIRE(!implicit_charging_task_added);
  }

  WHEN("Planning for 11 requests and 2 agents no.2")
  {
    const auto now = std::chrono::steady_clock::now();
    const double default_orientation = 0.0;

    rmf_traffic::agv::Plan::Start first_location{now, 13, default_orientation};
    rmf_traffic::agv::Plan::Start second_location{now, 2, default_orientation};

    std::vector<rmf_task::agv::State> initial_states =
    {
      rmf_task::agv::State{first_location, 9, 1.0},
      rmf_task::agv::State{second_location, 2, 1.0}
    };

    std::vector<rmf_task::agv::StateConfig> state_configs =
    {
      rmf_task::agv::StateConfig{0.2},
      rmf_task::agv::StateConfig{0.2}
    };

    std::vector<rmf_task::ConstRequestPtr> requests =
    {
      rmf_task::requests::Delivery::make(
        "1",
        6,
        "dispenser",
        3,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "2",
        10,
        "dispenser",
        7,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "3",
        2,
        "dispenser",
        12,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(0),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "4",
        8,
        "dispenser",
        11,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(50000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "5",
        10,
        "dispenser",
        6,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(50000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "6",
        2,
        "dispenser",
        9,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(70000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "7",
        3,
        "dispenser",
        4,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(70000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "8",
        5,
        "dispenser",
        11,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(70000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "9",
        9,
        "dispenser",
        1,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(70000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "10",
        1,
        "dispenser",
        5,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(70000),
        drain_battery),

      rmf_task::requests::Delivery::make(
        "11",
        13,
        "dispenser",
        10,
        "ingestor",
        {},
        motion_sink,
        device_sink,
        planner,
        now + rmf_traffic::time::from_seconds(70000),
        drain_battery)
    };

    std::shared_ptr<rmf_task::agv::TaskPlanner::Configuration>  task_config =
      std::make_shared<rmf_task::agv::TaskPlanner::Configuration>(
        battery_system,
        motion_sink,
        device_sink,
        planner);
    rmf_task::agv::TaskPlanner task_planner(task_config);

    auto start_time = std::chrono::steady_clock::now();
    const auto greedy_assignments = task_planner.greedy_plan(
      now, initial_states, state_configs, requests);
    const double greedy_cost = task_planner.compute_cost(greedy_assignments);
    auto finish_time = std::chrono::steady_clock::now();
    std::cout << "Greedy solution found in: "
              << (finish_time - start_time).count() / 1e9 << std::endl;
    display_solution("Greedy", greedy_assignments, greedy_cost);

    task_planner = rmf_task::agv::TaskPlanner(task_config);
    start_time = std::chrono::steady_clock::now();
    const auto optimal_assignments = task_planner.optimal_plan(
      now, initial_states, state_configs, requests, nullptr);
    const double optimal_cost = task_planner.compute_cost(optimal_assignments);
    finish_time = std::chrono::steady_clock::now();
    std::cout << "Optimal solution found in: "
              << (finish_time - start_time).count() / 1e9 << std::endl;
    display_solution("Optimal", optimal_assignments, optimal_cost);


    REQUIRE(optimal_cost <= greedy_cost);
  }

}