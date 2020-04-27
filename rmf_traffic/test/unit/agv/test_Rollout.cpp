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

#include <rmf_traffic/agv/Rollout.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

SCENARIO("Test Rollout")
{
  rmf_traffic::schedule::Database database;

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  auto p0 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_0",
          "test_Rollout",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto p1 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_1",
          "test_Rollout",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0}, true);  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}, true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}, true); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}, true); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}, true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                  10
   *                   |
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(1, 5);
  add_bidir_lane(2, 6);
  add_bidir_lane(3, 4);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(6, 7);
  add_bidir_lane(5, 8);
  add_bidir_lane(6, 9);
  add_bidir_lane(8, 9);
  add_bidir_lane(8, 10);

  // Create a conflict
  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  using namespace std::chrono_literals;
  const rmf_traffic::Duration wait_time = 1s;

  rmf_traffic::agv::Planner::Configuration configuration{graph, traits};
  rmf_traffic::agv::Planner planner{
    configuration,
    rmf_traffic::agv::Planner::Options{nullptr, wait_time}
  };

  rmf_traffic::agv::Planner::Options options_0{
    rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
          database,
          p0.id(),
          profile),
    wait_time
  };

  rmf_traffic::agv::Planner::Options options_1{
    rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
          database,
          p1.id(),
          profile),
    wait_time
  };

  const auto start_time = std::chrono::steady_clock::now();

  const auto plan_0 = planner.plan(
        rmf_traffic::agv::Plan::Start(start_time, 7, 0.0),
        rmf_traffic::agv::Plan::Goal(3), options_0);
  CHECK(plan_0);
  p0.set(plan_0->get_itinerary());

  const auto plan_1 = planner.plan(
        rmf_traffic::agv::Plan::Start(start_time, 3, 0.0),
        rmf_traffic::agv::Plan::Goal(7), options_1);
  CHECK_FALSE(plan_1);
  CHECK(std::find(plan_1.blockers().begin(), plan_1.blockers().end(), p0.id())
        != plan_1.blockers().end());

  rmf_traffic::agv::Rollout rollout_1(plan_1);
  const auto alternatives = rollout_1.expand(
        p0.id(), 30s, rmf_traffic::agv::Planner::Options{nullptr, 10s});

//  std::cout << "Identified " << alternatives.size() << " alternatives"
//            << std::endl;
//  for (const auto& itinerary : alternatives)
//  {
//    std::cout << "Start ";
//    for (const auto& route : itinerary)
//    {
//      for (const auto& wp : route->trajectory())
//        std::cout << " -- (" << wp.position().transpose() << ")";
//    }

//    std::cout << "\n" << std::endl;
//  }
}
