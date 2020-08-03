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

#include "../mock/MockTrafficLightCommand.hpp"

#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rclcpp/context.hpp>

#include <rmf_utils/catch.hpp>

#include "../thread_cooldown.hpp"

//==============================================================================
std::vector<rmf_fleet_adapter::agv::Waypoint> make_path(
    const rmf_traffic::agv::Graph& graph,
    const std::vector<std::size_t>& wp_indices,
    const double orientation)
{
  std::vector<rmf_fleet_adapter::agv::Waypoint> path;
  for (auto index : wp_indices)
  {
    const auto& wp = graph.get_waypoint(index);
    Eigen::Vector3d p;
    p[2] = orientation;
    p.block<2,1>(0,0) = wp.get_location();
    path.emplace_back(wp.get_map_name(), p);
  }

  return path;
}

//==============================================================================
SCENARIO("Test timing")
{
  rmf_fleet_adapter_test::thread_cooldown = true;
  using namespace std::chrono_literals;

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                   10
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

  add_bidir_lane(0, 1);  // 0   1
  add_bidir_lane(1, 2);  // 2   3
  add_bidir_lane(1, 5);  // 4   5
  add_bidir_lane(2, 6);  // 6   7
  add_bidir_lane(3, 4);  // 8   9
  add_bidir_lane(4, 5);  // 10 11
  add_bidir_lane(5, 6);  // 12 13
  add_bidir_lane(6, 7);  // 14 15
  add_bidir_lane(5, 8);  // 16 17
  add_bidir_lane(6, 9);  // 18 19
  add_bidir_lane(8, 9);  // 20 21
  add_bidir_lane(8, 10); // 22 23

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  auto rcl_context = std::make_shared<rclcpp::Context>();
  rcl_context->init(0, nullptr);
  rmf_fleet_adapter::agv::test::MockAdapter adapter(
        "test_TrafficLight", rclcpp::NodeOptions().context(rcl_context));

  auto command_0 =
      std::make_shared<rmf_fleet_adapter_test::MockTrafficLightCommand>();
  auto update_0 = adapter.add_traffic_light(
        command_0, "fleet_0", "robot_0", traits, profile);

  auto command_1 =
      std::make_shared<rmf_fleet_adapter_test::MockTrafficLightCommand>();
  auto update_1 = adapter.add_traffic_light(
        command_1, "fleet_1", "robot_1", traits, profile);

  adapter.start();

  WHEN("Crossing paths")
  {
    const auto now = adapter.node()->now();

    const auto path_0 = make_path(graph, {0, 1, 5, 8, 10}, M_PI/2.0);
    update_0->update_path(path_0);
    std::unique_lock<std::mutex> lock_0(command_0->mutex);
    command_0->cv.wait_for(
          lock_0, 100ms,
          [command_0](){ return command_0->current_version.has_value(); });
    REQUIRE(command_0->current_version.has_value());
    CHECK(path_0.size() == command_0->current_timing.size());

    const auto path_1 = make_path(graph, {3, 4, 5, 6, 7}, 0.0);
    update_1->update_path(path_1);
    std::unique_lock<std::mutex> lock_1(command_1->mutex);
    command_1->cv.wait_for(
          lock_1, 100ms,
          [command_1](){ return command_1->current_version.has_value(); });
    REQUIRE(command_1->current_version.has_value());
    CHECK(path_1.size() == command_1->current_timing.size());

    REQUIRE(command_0->current_timing.size()
            == command_1->current_timing.size());

    // They would normally collide at index==2, so from index==2 onwards,
    // the path of command_1 must be lagging behind the path of command_0 by
    // at least the planner's default minimum holding time, or else the
    // planner might not have actually avoided the collision as intended.
    for (std::size_t i=2; i < command_0->current_timing.size(); ++i)
    {
      CHECK(command_1->current_timing[i] - command_0->current_timing[i]
            >= rmf_traffic::agv::Planner::Options::DefaultMinHoldingTime);
    }
  }
}
