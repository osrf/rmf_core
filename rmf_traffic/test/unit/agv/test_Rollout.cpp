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
#include <rmf_traffic/DetectConflict.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

class MockValidator : public rmf_traffic::agv::RouteValidator
{
public:

  MockValidator(
      rmf_traffic::Profile profile,
      ParticipantId other_participant,
      rmf_traffic::Profile other_profile,
      rmf_traffic::schedule::Itinerary other_itinerary)
    : _profile(std::move(profile)),
      _other_participant(other_participant),
      _other_profile(std::move(other_profile)),
      _other_itinerary(std::move(other_itinerary))
  {
    // Do nothing
  }

  rmf_utils::optional<Conflict> find_conflict(
      const Route& route) const final
  {
    for (const auto& blocking_route : _other_itinerary)
    {
      if (route.map() != blocking_route->map())
        continue;

      if (blocking_route->trajectory().size() < 2)
        continue;

      if (const auto time = rmf_traffic::DetectConflict::between(
            _profile,
            route.trajectory(),
            _other_profile,
            blocking_route->trajectory()))
        return Conflict{_other_participant, *time};
    }

    return rmf_utils::nullopt;
  }

  std::unique_ptr<RouteValidator> clone() const final
  {
    return std::make_unique<MockValidator>(*this);
  }

  rmf_traffic::Profile _profile;
  ParticipantId _other_participant;
  rmf_traffic::Profile _other_profile;
  rmf_traffic::schedule::Itinerary _other_itinerary;
};

SCENARIO("Test Rollout on graph with side routes")
{
  const auto database = std::make_shared<rmf_traffic::schedule::Database>();

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
  std::size_t start_0;
  std::size_t goal_0;
  std::size_t start_1;
  std::size_t goal_1;

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  GIVEN("Nav graph with side routes")
  {
    graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
    graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
    graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true); // 2
    graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
    graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
    graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
    graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
    graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
    graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
    graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
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

    start_0 = 7;
    start_1 = 3;

    goal_0 = start_1;
    goal_1 = start_0;

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
  }

  GIVEN("Nav graph with only a pullover")
  {
    /*
     *        3
     *        |
     *        |
     * 0------1------2
     *
     **/

    graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 0
    graph.add_waypoint(test_map_name, { 0.0, 0.0}); // 1
    graph.add_waypoint(test_map_name, { 5.0, 0.0}); // 2
    graph.add_waypoint(test_map_name, { 0.0, 5.0}).set_holding_point(true); // 3

    start_0 = 0;
    start_1 = 2;

    goal_0 = start_1;
    goal_1 = start_0;

    add_bidir_lane(0, 1);
    add_bidir_lane(1, 2);
    add_bidir_lane(1, 3);
  }

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
        rmf_traffic::agv::Plan::Start(start_time, start_0, 0.0),
        rmf_traffic::agv::Plan::Goal(goal_0), options_0);
  CHECK(plan_0);
  p0.set(plan_0->get_itinerary());

  const auto plan_1 = planner.plan(
        rmf_traffic::agv::Plan::Start(start_time, start_1, 0.0),
        rmf_traffic::agv::Plan::Goal(goal_1), options_1);
  CHECK_FALSE(plan_1);
  CHECK(std::find(plan_1.blockers().begin(), plan_1.blockers().end(), p0.id())
        != plan_1.blockers().end());

  rmf_traffic::agv::Rollout rollout_1(plan_1);
  const auto alternatives = rollout_1.expand(
        p0.id(), 30s, rmf_traffic::agv::Planner::Options{nullptr, 10s});

  bool found_plan = false;
//  std::size_t alterantive_count = 0;
//  std::cout << "Found " << alternatives.size() << " alterantives" << std::endl;
  for (const auto& itinerary : alternatives)
  {
//      std::cout << "Trying alternative #" << ++alterantive_count << std::endl;
//      for (const auto& r : itinerary)
//        std::cout << "(" << r->trajectory().front().position().transpose()
//                  << ") --> (" << r->trajectory().back().position().transpose()
//                  << ") --> ";
//      std::cout << "(end)\n" << std::endl;

    const auto new_plan_0 = plan_0.replan(
          plan_0.get_starts(),
          rmf_traffic::agv::Planner::Options{
            rmf_utils::make_clone<MockValidator>(
              profile,
              p1.id(),
              profile,
              itinerary),
            wait_time
          });

    if (new_plan_0)
    {
      found_plan = true;
      p0.set(new_plan_0->get_itinerary());

//      std::cout << "Found plan:\n(start) --> ";
//      for (const auto& wp : new_plan_0->get_waypoints())
//      {
//        if (wp.graph_index())
//          std::cout << *wp.graph_index() << " --> ";
//        else
//          std::cout << "(no index) --> ";
//      }
//      std::cout << "(end)\n" << std::endl;

      break;
    }
  }

  REQUIRE(found_plan);

  const auto new_plan_1 = plan_1.replan(plan_1.get_starts());
  REQUIRE(new_plan_1);
  p1.set(new_plan_1->get_itinerary());

  const auto new_plan_0 = plan_0.replan(plan_0.get_starts());
  CHECK(new_plan_0);
//  std::cout << "Found plan:\n(start) --> ";
//  for (const auto& wp : new_plan_0->get_waypoints())
//  {
//    if (wp.graph_index())
//      std::cout << *wp.graph_index() << " --> ";
//    else
//      std::cout << "(no index) --> ";
//  }
//  std::cout << "(end)\n" << std::endl;
}
