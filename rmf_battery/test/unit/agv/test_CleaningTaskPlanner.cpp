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

#include <rmf_battery/agv/CleaningTaskPlanner.hpp>
#include <rmf_battery/agv/SystemTraits.hpp>

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>


#include <rmf_utils/catch.hpp>
#include <rmf_utils/clone_ptr.hpp>

rmf_utils::clone_ptr<rmf_traffic::agv::ScheduleRouteValidator>
make_test_schedule_validator(
  const rmf_traffic::schedule::Viewer& viewer,
  rmf_traffic::Profile profile)
{
  return rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
    viewer,
    std::numeric_limits<rmf_traffic::schedule::ParticipantId>::max(),
    std::move(profile));
}

SCENARIO("Test CleaningTaskPlanner")
{
  using namespace std::chrono_literals;
  using Graph = rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using Interpolate = rmf_traffic::agv::Interpolate;
  using Planner = rmf_traffic::agv::Planner;
  using CleaningTaskPlanner = rmf_battery::agv::CleaningTaskPlanner;
  using SystemTraits = rmf_battery::agv::SystemTraits;

  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const rmf_traffic::Profile profile{shape, shape};

  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, profile);

  rmf_traffic::schedule::Database database;

  const auto default_options = rmf_traffic::agv::Planner::Options{
    make_test_schedule_validator(database, profile)};

  // Setup graph
  Graph graph;
  // 3--------0------1
  //          |
  //          |
  //          2
  const std::string map_name = "L1";
  graph.add_waypoint(map_name, {0, 0}).set_holding_point(true); // 0
  graph.add_waypoint(map_name, {10000, 0}).set_holding_point(true); // 1
  graph.add_waypoint(map_name, {0, -10000}).set_holding_point(true); // 2
  graph.add_waypoint(map_name, {-20000, 0}).set_holding_point(true); // 3

  graph.add_lane(0, 1);
  graph.add_lane(1, 0);
  graph.add_lane(0, 2);
  graph.add_lane(2, 0);
  graph.add_lane(0, 3);
  graph.add_lane(3, 0);


  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options    
  };

  SystemTraits::BatterySystem battery_system{24, 40, 2};
  SystemTraits::MechanicalSystem mechanical_system{70, 40, 0.22};
  SystemTraits::PowerSystem power_system_1{"processor", 20, 5};
  SystemTraits::PowerSystem power_system_2{"cleaning", 30, 5};
  SystemTraits::PowerSystems power_systems;
  power_systems.insert({power_system_1.name(), power_system_1});
  power_systems.insert({power_system_2.name(), power_system_2});
  SystemTraits system_traits{
    mechanical_system, battery_system, power_systems};
  REQUIRE(system_traits.valid());

  CleaningTaskPlanner cleaning_planner{
    system_traits, planner, power_system_2.name()};

  const auto start_time = std::chrono::steady_clock::now();

  WHEN("Cleaning zone 1")
  {
    Planner::Start start{start_time, 0, 0.0}; // Start at 0
    rmf_traffic::Trajectory cleaning_trajectory = 
    rmf_traffic::agv::Interpolate::positions(
      traits,
      start_time + 15000s,
      {{15000, 0, 0}, {15000, 100, 0}, {15000, 0, 0}}
    );

    const auto itinerary = cleaning_planner.plan(
      start,
      1.0,
      1,
      cleaning_trajectory,
      1,
      0
    );

    CHECK(itinerary.size() > 0);


  }

}