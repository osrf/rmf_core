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

#include "utils_NegotiationRoom.hpp"

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/DetectConflict.hpp>

#include <rmf_traffic/agv/debug/Planner.hpp>

Eigen::Vector3d get_location(
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Graph& graph)
{
  if (start.location())
  {
    const auto& p = *start.location();
    return {p[0], p[1], start.orientation()};
  }

  const auto& p = graph.get_waypoint(start.waypoint()).get_location();
  return {p[0], p[1], start.orientation()};
}

void check_start_compatibility(
    const rmf_traffic::agv::Graph& graph_a,
    const rmf_traffic::Profile& profile_a,
    const std::vector<rmf_traffic::agv::Plan::Start>& a_starts,
    const rmf_traffic::agv::Graph& graph_b,
    const rmf_traffic::Profile& profile_b,
    const std::vector<rmf_traffic::agv::Plan::Start>& b_starts)
{
  std::cout << "a_starts: " << a_starts.size()
            << " | b_starts: " << b_starts.size()
            << std::endl;

  using namespace std::chrono_literals;
  const auto zero = Eigen::Vector3d::Zero();

  for (const auto& a : a_starts)
  {
    rmf_traffic::Trajectory a_traj;
    const auto p_a = get_location(a, graph_a);
    a_traj.insert(a.time(), p_a, zero);
    a_traj.insert(a.time() + 10s, p_a, zero);

    for (const auto& b : b_starts)
    {
      rmf_traffic::Trajectory b_traj;
      const auto p_b = get_location(b, graph_b);
      b_traj.insert(b.time(), p_b, zero);
      b_traj.insert(b.time() + 10s, p_b, zero);

      if (const auto time = rmf_traffic::DetectConflict::between(
            profile_a, a_traj,
            profile_b, b_traj))
      {
        std::cout << "CONFLICT FOUND" << std::endl;
      }
    }
  }

}

SCENARIO("Test difficult 3-way scenarios")
{
  const std::string test_map_name = "test_map";

  rmf_traffic::agv::Graph graph_a;
  graph_a.add_waypoint(test_map_name, {18.013, -15.518}); // 0
  graph_a.add_waypoint(test_map_name, {12.417, -15.536}); // 1
  graph_a.add_waypoint(test_map_name, { 9.990, -15.543}); // 2
  graph_a.add_waypoint(test_map_name, { 9.482, -15.534}); // 3
  graph_a.add_waypoint(test_map_name, {17.934, -20.165}); // 4
  graph_a.add_waypoint(test_map_name, {17.943, -21.654}); // 5
  graph_a.add_waypoint(test_map_name, {22.370, -21.593}); // 6
  graph_a.add_waypoint(test_map_name, {22.370, -22.660}); // 7
  graph_a.add_waypoint(test_map_name, {22.399, -23.540}); // 8
  graph_a.add_waypoint(test_map_name, {18.968, -20.183}); // 9
  graph_a.add_waypoint(test_map_name, {20.137, -20.188}); // 10
  graph_a.add_waypoint(test_map_name, { 8.185, -15.535}); // 11

  graph_a.add_lane(0, 1);
  graph_a.add_lane(1, 0);
  graph_a.add_lane(1, 2);
  graph_a.add_lane(2, 1);
  graph_a.add_lane(2, 3);
  graph_a.add_lane(3, 2);
  graph_a.add_lane(4, 5);
  graph_a.add_lane(5, 4);
  graph_a.add_lane(5, 6);
  graph_a.add_lane(6, 5);
  graph_a.add_lane(0, 4);
  graph_a.add_lane(4, 0);
  graph_a.add_lane(6, 7);
  graph_a.add_lane(7, 6);
  graph_a.add_lane(7, 8);
  graph_a.add_lane(8, 7);
  graph_a.add_lane(4, 9);
  graph_a.add_lane(9, 4);
  graph_a.add_lane(9, 10);
  graph_a.add_lane(10, 9);
  graph_a.add_lane(3, 11);
  graph_a.add_lane(11, 3);




  rmf_traffic::agv::Graph graph_b;
  graph_b.add_waypoint(test_map_name, {14.052, -23.840}); // 0
  graph_b.add_waypoint(test_map_name, {14.081, -21.636}); // 1
  graph_b.add_waypoint(test_map_name, {10.430, -23.964}); // 2
  graph_b.add_waypoint(test_map_name, {10.432, -21.788}); // 3
  graph_b.add_waypoint(test_map_name, {10.429, -25.995}); // 4
  graph_b.add_waypoint(test_map_name, { 8.093, -23.870}); // 5
  graph_b.add_waypoint(test_map_name, { 8.055, -21.818}); // 6
  graph_b.add_waypoint(test_map_name, { 8.068, -25.961}); // 7
  graph_b.add_waypoint(test_map_name, {14.920, -21.585}); // 8
  graph_b.add_waypoint(test_map_name, {14.918, -23.846}); // 9
  graph_b.add_waypoint(test_map_name, {18.435, -21.633}); // 10
  graph_b.add_waypoint(test_map_name, {18.511, -17.278}); // 11
  graph_b.add_waypoint(test_map_name, {16.830, -17.278}); // 12
  graph_b.add_waypoint(test_map_name, {11.829, -21.687}); // 13
  graph_b.add_waypoint(test_map_name, {11.892, -15.379}); // 14
  graph_b.add_waypoint(test_map_name, {16.876, -15.313}); // 15
  graph_b.add_waypoint(test_map_name, {11.829, -19.079}); // 16

  graph_b.add_lane(0, 1);
  graph_b.add_lane(1, 0);
  graph_b.add_lane(1, 2);
  graph_b.add_lane(2, 1);
  graph_b.add_lane(2, 3);
  graph_b.add_lane(3, 2);
  graph_b.add_lane(2, 4);
  graph_b.add_lane(4, 2);
  graph_b.add_lane(2, 5);
  graph_b.add_lane(5, 2);
  graph_b.add_lane(5, 6);
  graph_b.add_lane(6, 5);
  graph_b.add_lane(5, 7);
  graph_b.add_lane(7, 5);
  graph_b.add_lane(1, 8);
  graph_b.add_lane(8, 1);
  graph_b.add_lane(8, 9);
  graph_b.add_lane(9, 8);
  graph_b.add_lane(8, 10);
  graph_b.add_lane(10, 8);
  graph_b.add_lane(10, 11);
  graph_b.add_lane(11, 10);
  graph_b.add_lane(11, 12);
  graph_b.add_lane(12, 11);
  graph_b.add_lane(13, 1);
  graph_b.add_lane(1, 13);
  graph_b.add_lane(14, 15);
  graph_b.add_lane(15, 14);
  graph_b.add_lane(15, 12);
  graph_b.add_lane(12, 15);
  graph_b.add_lane(14, 16);
  graph_b.add_lane(16, 14);
  graph_b.add_lane(16, 13);
  graph_b.add_lane(13, 16);

  const auto profile_a = rmf_traffic::Profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(0.5),
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(2.0)
  };

  const auto traits_a = rmf_traffic::agv::VehicleTraits{
    {0.45, 0.5}, {0.3, 1.5}, profile_a
  };

  const auto profile_b = rmf_traffic::Profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(0.3),
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.5)
  };

  const auto traits_b = rmf_traffic::agv::VehicleTraits{
    {0.4, 3.0}, {0.4, 4.0}, profile_b
  };

  const rmf_traffic::agv::Planner::Configuration config_a{graph_a, traits_a};
  const rmf_traffic::agv::Planner::Configuration config_b{graph_b, traits_b};

  auto database = std::make_shared<rmf_traffic::schedule::Database>();

  auto a0 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "a0",
      "fleet_a",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile_a
    }, database);

  auto b1 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "b1",
      "fleet_b",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile_b
    }, database);

  auto b2 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "b2",
      "fleet_b",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile_b
    }, database);

  GIVEN("Case 1")
  {
    const auto time = std::chrono::steady_clock::now();

    auto a0_starts = rmf_traffic::agv::compute_plan_starts(
      graph_a, test_map_name, {14.006982, -15.530105, -3.137865}, time);
    auto a0_goal = rmf_traffic::agv::Plan::Goal(1);

    auto b1_starts = rmf_traffic::agv::compute_plan_starts(
          graph_b, test_map_name, {11.892134, -15.398828, 2.631314}, time);
    auto b1_goal = rmf_traffic::agv::Plan::Goal(11);

    auto b2_starts = rmf_traffic::agv::compute_plan_starts(
          graph_b, test_map_name, {13.057442, -15.363754, -3.128299}, time);
    auto b2_goal = rmf_traffic::agv::Plan::Goal(13);

    NegotiationRoom::Intentions intentions;
    intentions.insert({
      a0.id(),
      NegotiationRoom::Intention{std::move(a0_starts), a0_goal, config_a} });

    intentions.insert({
      b1.id(),
      NegotiationRoom::Intention{std::move(b1_starts), b1_goal, config_b}});

    intentions.insert({
      b2.id(),
      NegotiationRoom::Intention{std::move(b2_starts), b2_goal, config_b}});

    auto room = NegotiationRoom(database, intentions, 5.5, 200);
    auto proposal = room.solve();
    REQUIRE(proposal);
  }
}
