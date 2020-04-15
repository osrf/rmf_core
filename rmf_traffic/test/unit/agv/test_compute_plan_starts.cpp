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

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test computing Starts from coordinates")
{
  using namespace std::chrono_literals;
  using rmf_traffic::agv::Graph;

  const std::string test_map_name = "test_map";
  Graph graph;

  const rmf_traffic::Time initial_time = std::chrono::steady_clock::now();

  double max_waypoint_merging_distance = 0.1;
  double max_lane_merging_distance = 1.0;
  double min_lane_length = 1e-8;

  WHEN("Location is on a waypoint, that is part of both a unidirectional and "
    "bidirectional lane")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {0.05, 0.05}); // 1
    graph.add_waypoint(test_map_name, {10, 10}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(1, 2); // 2
    REQUIRE(graph.num_lanes() == 3);

    Eigen::Vector3d robot_loc = {0, 0, 0};

    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);

    CHECK(!start_set.empty());
    CHECK(start_set.size() == 1);
    CHECK_FALSE(start_set[0].location());
    CHECK_FALSE(start_set[0].lane());
    CHECK(start_set[0].waypoint() == 0);
  }

  WHEN("Location is near a waypoint, within the waypoint merging distance. "
    "Waypoint is part of both a unidirectional and a bidirectional lane.")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {0.1, 0.1}); // 1
    graph.add_waypoint(test_map_name, {10, 10}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(1, 2); // 2
    REQUIRE(graph.num_lanes() == 3);

    {
      Eigen::Vector3d robot_loc = {0.1 + 1e-8, 0.1, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].location());
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].waypoint() == 1);
    }
    {
      Eigen::Vector3d robot_loc = {0.1 - 1e-8, 0.1, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].location());
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].waypoint() == 1);
    }
    {
      Eigen::Vector3d robot_loc = {0.1, 0.1 + 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].location());
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].waypoint() == 1);
    }
    {
      Eigen::Vector3d robot_loc = {0.1, 0.1 - 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].location());
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].waypoint() == 1);
    }
  }

  WHEN("Location is right on the border of merging to a waypoint, that is "
    "connected to both a unidirectional and bidirectional lane.")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {0.1, 0.1}); // 1
    graph.add_waypoint(test_map_name, {10, 10}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(1, 2); // 2
    REQUIRE(graph.num_lanes() == 3);

    {
      Eigen::Vector3d robot_loc = {0.1 + 0.1 - 1e-8, 0.1, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].location());
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].waypoint() == 1);
    }
    {
      Eigen::Vector3d robot_loc = {0.1 - 0.1 + 1e-8, 0.1, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].location());
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].waypoint() == 1);
    }
    {
      Eigen::Vector3d robot_loc = {0.1, 0.1 + 0.1 - 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].location());
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].waypoint() == 1);
    }
    {
      Eigen::Vector3d robot_loc = {0.1, 0.1 - 0.1 + 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].location());
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].waypoint() == 1);
    }
  }

  WHEN("Location is near multiple waypoints, could be merged to either one. "
    "Waypoints are either connected to both unidirectional and bidirectional "
    "lanes.")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {0.01, 0.01}); // 1
    graph.add_waypoint(test_map_name, {-0.01, -0.01}); // 2
    graph.add_waypoint(test_map_name, {0.01, -0.01}); // 3
    graph.add_waypoint(test_map_name, {-0.01, 0.01}); // 4
    graph.add_waypoint(test_map_name, {10, 10}); // 5
    REQUIRE(graph.num_waypoints() == 6);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(0, 2); // 2
    graph.add_lane(0, 3); // 3
    graph.add_lane(3, 0); // 4
    graph.add_lane(0, 4); // 5
    graph.add_lane(0, 5); // 6
    REQUIRE(graph.num_lanes() == 7);

    Eigen::Vector3d robot_loc = {0.05, 0.05, 0};

    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);

    CHECK(!start_set.empty());
    CHECK(start_set.size() == 1);
    CHECK_FALSE(start_set[0].location());
    CHECK_FALSE(start_set[0].lane());
    CHECK((start_set[0].waypoint() == 0 || start_set[0].waypoint() == 1
      || start_set[0].waypoint() == 2 || start_set[0].waypoint() == 3
      || start_set[0].waypoint() == 4));
  }

  WHEN("Location is not within merging with waypoints, within lane merging, "
    "not between lane entry and lane exit, right outside of waypoint "
    "merging distance. Waypoints are connected to both unidirectional or "
    "bidirectional lanes.")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {1, 0}); // 1
    graph.add_waypoint(test_map_name, {10.0, 0}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(1, 2); // 2
    REQUIRE(graph.num_lanes() == 3);

    {
      Eigen::Vector3d robot_loc = {
        0 - max_waypoint_merging_distance - 1e-8, 0, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }

    const double slightly_larger_radius_x_y =
      sqrt(pow(max_waypoint_merging_distance + 1e-8, 2.0) / 2.0);
    {
      Eigen::Vector3d robot_loc = {
        0 - slightly_larger_radius_x_y, 0 + slightly_larger_radius_x_y, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(
        graph, robot_loc, initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }
    {
      Eigen::Vector3d robot_loc = {
        0 - slightly_larger_radius_x_y, 0 - slightly_larger_radius_x_y, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }

    const double very_near_to_border_y =
      sqrt(pow(max_waypoint_merging_distance + 1e-8, 2.0) - pow(1e-8, 2.0));
    {
      Eigen::Vector3d robot_loc = {0 - 1e-8, 0 + very_near_to_border_y, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }
    {
      Eigen::Vector3d robot_loc = {0 - 1e-8, 0 - very_near_to_border_y, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }
  }

  WHEN("Location is not within merging with waypoints, within lane merging, "
    "not between lane entry and lane exit, right within lane merging "
    "distance. Waypoint is connected to both unidirectional and bidirectional"
    " lanes.")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {1, 0}); // 1
    graph.add_waypoint(test_map_name, {10.0, 0}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(1, 2); // 2
    REQUIRE(graph.num_lanes() == 3);

    {
      Eigen::Vector3d robot_loc = {0 - max_lane_merging_distance + 1e-8, 0, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }

    const double slightly_smaller_than_border_x_y =
      sqrt(pow(max_lane_merging_distance - 1e-8, 2.0) / 2.0);
    {
      Eigen::Vector3d robot_loc = {
        0 - slightly_smaller_than_border_x_y,
        0 + slightly_smaller_than_border_x_y,
        0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }
    {
      Eigen::Vector3d robot_loc = {
        0 - slightly_smaller_than_border_x_y,
        0 - slightly_smaller_than_border_x_y,
        0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(
        graph, robot_loc, initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }

    const double very_near_to_border_y =
      sqrt(pow(max_lane_merging_distance - 1e-8, 2.0) - pow(1e-8, 2.0));
    {
      Eigen::Vector3d robot_loc = {0 - 1e-8, 0 + very_near_to_border_y, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }
    {
      Eigen::Vector3d robot_loc = {0 - 1e-8, 0 - very_near_to_border_y, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(
        graph, robot_loc, initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK_FALSE(start_set[0].lane());
      CHECK(start_set[0].location());
      CHECK(start_set[0].waypoint() == 0);
    }
  }

  WHEN("Location is on unidirectional lane")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {0.5, 0.5}); // 1
    graph.add_waypoint(test_map_name, {10.0, 10.0}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 2); // 1
    REQUIRE(graph.num_lanes() == 2);

    Eigen::Vector3d robot_loc = {5.5, 5.5, 0};

    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 1);
    CHECK(start_set[0].waypoint() == 2);
    CHECK(start_set[0].location());
    CHECK(start_set[0].lane());
    CHECK(start_set[0].lane().value() == 1);
  }

  WHEN("Location is on bidirectional lane")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {0.5, 0.5}); // 1
    graph.add_waypoint(test_map_name, {10.0, 10.0}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 2); // 1
    graph.add_lane(2, 1); // 2
    REQUIRE(graph.num_lanes() == 3);

    Eigen::Vector3d robot_loc = {5.5, 5.5, 0};

    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 2);

    std::unordered_set<std::size_t> lane_indices;
    std::unordered_set<std::size_t> waypoint_indices;
    for (std::size_t i = 0; i < 2; ++i)
    {
      std::size_t waypoint_index = start_set[i].waypoint();
      CHECK(waypoint_indices.insert(waypoint_index).second);
      CHECK((waypoint_index == 1 || waypoint_index == 2));
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);
      if (waypoint_index == 1)
        CHECK(lane_index == 2);
      else if (waypoint_index == 2)
        CHECK(lane_index == 1);
      else
        CHECK(false);
    }
  }

  WHEN("Location is not within merging with waypoints, within lane merging, "
    "between unidirectional lane entry and exit.")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {1, 0}); // 1
    graph.add_waypoint(test_map_name, {10, 0}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 2); // 1
    REQUIRE(graph.num_lanes() == 2);

    {
      Eigen::Vector3d robot_loc = {5, 0 + 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK(start_set[0].waypoint() == 2);
      CHECK(start_set[0].location());
      CHECK(start_set[0].lane());
      const std::size_t& lane_index = start_set[0].lane().value();
      CHECK(lane_index == 1);
    }
    {
      Eigen::Vector3d robot_loc = {5, 0 - 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK(start_set[0].location());
      CHECK(start_set[0].lane());
      const std::size_t& lane_index = start_set[0].lane().value();
      CHECK(lane_index == 1);
    }
    {
      Eigen::Vector3d robot_loc = {5, 0 + max_lane_merging_distance - 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK(start_set[0].location());
      CHECK(start_set[0].lane());
      const std::size_t& lane_index = start_set[0].lane().value();
      CHECK(lane_index == 1);
    }
    {
      Eigen::Vector3d robot_loc = {5, 0 - max_lane_merging_distance + 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 1);
      CHECK(start_set[0].location());
      CHECK(start_set[0].lane());
      const std::size_t& lane_index = start_set[0].lane().value();
      CHECK(lane_index == 1);
    }
  }

  WHEN("Location is not within merging with waypoints, within lane merging, "
    "between bidirectional lane entry and exit.")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {1, 0}); // 1
    graph.add_waypoint(test_map_name, {10, 0}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 2); // 1
    graph.add_lane(2, 1); // 2
    REQUIRE(graph.num_lanes() == 3);

    {
      Eigen::Vector3d robot_loc = {5, 0 + 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 2);

      std::unordered_set<std::size_t> wp_indices;
      std::unordered_set<std::size_t> lane_indices;
      for (std::size_t i = 0; i < 2; ++i)
      {
        std::size_t wp_index = start_set[i].waypoint();
        CHECK(wp_indices.insert(wp_index).second);
        CHECK((wp_index == 1 || wp_index == 2));
        CHECK(start_set[i].location());
        CHECK(start_set[i].lane());
        const std::size_t& lane_index = start_set[i].lane().value();
        CHECK(lane_indices.insert(lane_index).second);
        if (wp_index == 1)
          CHECK(lane_index == 2);
        else if (wp_index == 2)
          CHECK(lane_index == 1);
        else
          CHECK(false);
      }
    }
    {
      Eigen::Vector3d robot_loc = {5, 0 - 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 2);

      std::unordered_set<std::size_t> wp_indices;
      std::unordered_set<std::size_t> lane_indices;
      for (std::size_t i = 0; i < 2; ++i)
      {
        std::size_t wp_index = start_set[i].waypoint();
        CHECK(wp_indices.insert(wp_index).second);
        CHECK((wp_index == 1 || wp_index == 2));
        CHECK(start_set[i].location());
        CHECK(start_set[i].lane());
        const std::size_t& lane_index = start_set[i].lane().value();
        CHECK(lane_indices.insert(lane_index).second);
        if (wp_index == 1)
          CHECK(lane_index == 2);
        else if (wp_index == 2)
          CHECK(lane_index == 1);
        else
          CHECK(false);
      }
    }
    {
      Eigen::Vector3d robot_loc = {5, 0 + 1.0 - 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 2);

      std::unordered_set<std::size_t> wp_indices;
      std::unordered_set<std::size_t> lane_indices;
      for (std::size_t i = 0; i < 2; ++i)
      {
        std::size_t wp_index = start_set[i].waypoint();
        CHECK(wp_indices.insert(wp_index).second);
        CHECK((wp_index == 1 || wp_index == 2));
        CHECK(start_set[i].location());
        CHECK(start_set[i].lane());
        const std::size_t& lane_index = start_set[i].lane().value();
        CHECK(lane_indices.insert(lane_index).second);
        if (wp_index == 1)
          CHECK(lane_index == 2);
        else if (wp_index == 2)
          CHECK(lane_index == 1);
        else
          CHECK(false);
      }
    }
    {
      Eigen::Vector3d robot_loc = {5, 0 - 1.0 + 1e-8, 0};
      std::vector<rmf_traffic::agv::Plan::Start> start_set =
        rmf_traffic::agv::compute_plan_starts(graph,
          robot_loc,
          initial_time,
          max_waypoint_merging_distance,
          max_lane_merging_distance,
          min_lane_length);
      CHECK(!start_set.empty());
      CHECK(start_set.size() == 2);

      std::unordered_set<std::size_t> wp_indices;
      std::unordered_set<std::size_t> lane_indices;
      for (std::size_t i = 0; i < 2; ++i)
      {
        std::size_t wp_index = start_set[i].waypoint();
        CHECK(wp_indices.insert(wp_index).second);
        CHECK((wp_index == 1 || wp_index == 2));
        CHECK(start_set[i].location());
        CHECK(start_set[i].lane());
        const std::size_t& lane_index = start_set[i].lane().value();
        CHECK(lane_indices.insert(lane_index).second);
        if (wp_index == 1)
          CHECK(lane_index == 2);
        else if (wp_index == 2)
          CHECK(lane_index == 1);
        else
          CHECK(false);
      }
    }
  }

  WHEN("Location is on the cross section of 2 unidirectional lanes, within "
    "lane merging for both lanes")
  {
    graph.add_waypoint(test_map_name, {-10, 0}); // 0
    graph.add_waypoint(test_map_name, {10, 0}); // 1
    graph.add_waypoint(test_map_name, {0, 10}); // 2
    graph.add_waypoint(test_map_name, {0, -10}); // 3
    REQUIRE(graph.num_waypoints() == 4);

    graph.add_lane(0, 1); // 0
    graph.add_lane(2, 3); // 1
    REQUIRE(graph.num_lanes() == 2);

    Eigen::Vector3d robot_loc = {
      0 + max_lane_merging_distance - 1e-8,
      0 + max_lane_merging_distance - 1e-8,
      0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 2);

    std::unordered_set<std::size_t> wp_indices;
    std::unordered_set<std::size_t> lane_indices;
    for (std::size_t i = 0; i < 2; ++i)
    {
      std::size_t wp_index = start_set[i].waypoint();
      CHECK(wp_indices.insert(wp_index).second);
      CHECK((wp_index == 1 || wp_index == 3));
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);
      if (wp_index == 1)
        CHECK(lane_index == 0);
      else if (wp_index == 3)
        CHECK(lane_index == 1);
      else
        CHECK(false);
    }
  }

  WHEN("Location is on the cross section of a unidirectional lane, and a "
    "bidirectional lane, within lane merging for both lanes")
  {
    graph.add_waypoint(test_map_name, {-10, 0}); // 0
    graph.add_waypoint(test_map_name, {10, 0}); // 1
    graph.add_waypoint(test_map_name, {0, 10}); // 2
    graph.add_waypoint(test_map_name, {0, -10}); // 3
    REQUIRE(graph.num_waypoints() == 4);

    graph.add_lane(0, 1); // 0
    graph.add_lane(2, 3); // 1
    graph.add_lane(3, 2); // 2
    REQUIRE(graph.num_lanes() == 3);

    Eigen::Vector3d robot_loc = {
      0 + max_lane_merging_distance - 1e-8,
      0 + max_lane_merging_distance - 1e-8,
      0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 3);

    std::unordered_set<std::size_t> wp_indices;
    std::unordered_set<std::size_t> lane_indices;
    for (std::size_t i = 0; i < 3; ++i)
    {
      std::size_t wp_index = start_set[i].waypoint();
      CHECK(wp_indices.insert(wp_index).second);
      CHECK((wp_index == 1 || wp_index == 2 || wp_index == 3));
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);
      if (wp_index == 1)
        CHECK(lane_index == 0);
      else if (wp_index == 3)
        CHECK(lane_index == 1);
      else if (wp_index == 2)
        CHECK(lane_index == 2);
      else
        CHECK(false);
    }
  }

  WHEN("Location is on the cross section of 2 bidirectional lanes, within lane "
    "merging for both lanes")
  {
    graph.add_waypoint(test_map_name, {-10, 0}); // 0
    graph.add_waypoint(test_map_name, {10, 0}); // 1
    graph.add_waypoint(test_map_name, {0, 10}); // 2
    graph.add_waypoint(test_map_name, {0, -10}); // 3
    REQUIRE(graph.num_waypoints() == 4);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(2, 3); // 2
    graph.add_lane(3, 2); // 3
    REQUIRE(graph.num_lanes() == 4);

    Eigen::Vector3d robot_loc =
    {0 + max_lane_merging_distance - 1e-8,
      0 + max_lane_merging_distance - 1e-8,
      0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 4);

    std::unordered_set<std::size_t> wp_indices;
    std::unordered_set<std::size_t> lane_indices;
    for (std::size_t i = 0; i < 4; ++i)
    {
      std::size_t wp_index = start_set[i].waypoint();
      CHECK(wp_indices.insert(wp_index).second);
      CHECK((wp_index == 0 || wp_index == 1 || wp_index == 2 || wp_index == 3));
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);
      if (wp_index == 0)
        CHECK(lane_index == 1);
      else if (wp_index == 1)
        CHECK(lane_index == 0);
      else if (wp_index == 2)
        CHECK(lane_index == 3);
      else if (wp_index == 3)
        CHECK(lane_index == 2);
      else
        CHECK(false);
    }
  }

  WHEN("Location is near an elbow cross section of 2 unidirectional lanes, "
    "within lane merging for both lanes, lanes moving away from elbow")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {5, 0}); // 1
    graph.add_waypoint(test_map_name, {5, 2 * max_lane_merging_distance}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(0, 2); // 1
    REQUIRE(graph.num_lanes() == 2);

    Eigen::Vector3d robot_loc =
    {0 + max_waypoint_merging_distance + 1e-8, 0, 0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 2);

    std::unordered_set<std::size_t> wp_indices;
    std::unordered_set<std::size_t> lane_indices;
    for (std::size_t i = 0; i < 2; ++i)
    {
      std::size_t wp_index = start_set[i].waypoint();
      CHECK(wp_indices.insert(wp_index).second);
      CHECK((wp_index == 1 || wp_index == 2));
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);
      if (wp_index == 1)
        CHECK(lane_index == 0);
      else if (wp_index == 2)
        CHECK(lane_index == 1);
      else
        CHECK(false);
    }
  }

  WHEN("Location is near an elbow cross section of 2 unidirectional lanes, "
    "within lane merging for both lanes, lanes moving towards the elbow")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {5, 0}); // 1
    graph.add_waypoint(test_map_name, {5, 2 * max_lane_merging_distance}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(1, 0); // 0
    graph.add_lane(2, 0); // 1
    REQUIRE(graph.num_lanes() == 2);

    Eigen::Vector3d robot_loc =
    {0 + max_waypoint_merging_distance + 1e-8, 0, 0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 2);

    std::unordered_set<std::size_t> lane_indices;
    for (std::size_t i = 0; i < 2; ++i)
    {
      std::size_t wp_index = start_set[i].waypoint();
      CHECK(wp_index == 0);
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);
      CHECK((lane_index == 0 || lane_index == 1));
    }
  }

  WHEN("Location is near an elbow cross section of 2 unidirectional lanes, "
    "within lane merging for both lanes, one lane moving away from the elbow "
    "the other towards the elbow")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {5, 0}); // 1
    graph.add_waypoint(test_map_name, {5, 2 * max_lane_merging_distance}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(1, 0); // 0
    graph.add_lane(0, 2); // 1
    REQUIRE(graph.num_lanes() == 2);

    Eigen::Vector3d robot_loc = {
      0 + max_waypoint_merging_distance + 1e-8, 0, 0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 2);

    std::unordered_set<std::size_t> wp_indices;
    std::unordered_set<std::size_t> lane_indices;
    for (std::size_t i = 0; i < 2; ++i)
    {
      std::size_t wp_index = start_set[i].waypoint();
      CHECK(wp_indices.insert(wp_index).second);
      CHECK((wp_index == 0 || wp_index == 2));
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);
      if (wp_index == 0)
        CHECK(lane_index == 0);
      else if (wp_index == 2)
        CHECK(lane_index == 1);
      else
        CHECK(false);
    }
  }

  WHEN("Location is near an elbow cross section of one unidirectional lane "
    "towards the elbow, and one bidirectional lane, within lane merging for "
    "both lanes")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {5, 0}); // 1
    graph.add_waypoint(test_map_name, {5, 2 * max_lane_merging_distance}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(1, 0); // 0
    graph.add_lane(0, 2); // 1
    graph.add_lane(2, 0); // 2
    REQUIRE(graph.num_lanes() == 3);

    Eigen::Vector3d robot_loc = {
      0 + max_waypoint_merging_distance + 1e-8, 0, 0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc, initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 3);

    std::unordered_set<std::size_t> lane_indices;
    for (std::size_t i = 0; i < 3; ++i)
    {
      std::size_t wp_index = start_set[i].waypoint();
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);

      if (wp_index == 0)
        CHECK((lane_index == 0 || lane_index == 2));
      else if (wp_index == 2)
        CHECK(lane_index == 1);
      else
        CHECK(false);
    }
  }

  WHEN("Location is near an elbow cross section of two bidirectional lanes, "
    "within lane merging for both lanes")
  {
    graph.add_waypoint(test_map_name, {0, 0}); // 0
    graph.add_waypoint(test_map_name, {5, 0}); // 1
    graph.add_waypoint(test_map_name, {5, 2 * max_lane_merging_distance}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(1, 0); // 0
    graph.add_lane(0, 1); // 1
    graph.add_lane(2, 0); // 2
    graph.add_lane(0, 2); // 3
    REQUIRE(graph.num_lanes() == 4);

    Eigen::Vector3d robot_loc = {
      0 + max_waypoint_merging_distance + 1e-8, 0, 0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(!start_set.empty());
    CHECK(start_set.size() == 4);

    std::unordered_set<std::size_t> lane_indices;
    for (std::size_t i = 0; i < 4; ++i)
    {
      std::size_t wp_index = start_set[i].waypoint();
      CHECK(start_set[i].location());
      CHECK(start_set[i].lane());
      const std::size_t& lane_index = start_set[i].lane().value();
      CHECK(lane_indices.insert(lane_index).second);

      if (wp_index == 0)
        CHECK((lane_index == 0 || lane_index == 2));
      else if (wp_index == 1)
        CHECK(lane_index == 1);
      else if (wp_index == 2)
        CHECK(lane_index == 3);
      else
        CHECK(false);
    }
  }

  WHEN("Location is neither within waypoint merging nor lane merging")
  {
    graph.add_waypoint(test_map_name, {-10, 0}); // 0
    graph.add_waypoint(test_map_name, {10, 0}); // 1
    graph.add_waypoint(test_map_name, {0, 10}); // 2
    graph.add_waypoint(test_map_name, {0, -10}); // 3
    REQUIRE(graph.num_waypoints() == 4);

    graph.add_lane(0, 1); // 0
    graph.add_lane(2, 3); // 1
    REQUIRE(graph.num_lanes() == 2);

    Eigen::Vector3d robot_loc = {1 + 1e-8, 1 + 1e-8, 0.0};
    std::vector<rmf_traffic::agv::Plan::Start> start_set =
      rmf_traffic::agv::compute_plan_starts(graph,
        robot_loc,
        initial_time,
        max_waypoint_merging_distance,
        max_lane_merging_distance,
        min_lane_length);
    CHECK(start_set.empty());
  }
}
