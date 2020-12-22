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

#include <src/rmf_traffic/agv/planning/EuclideanHeuristic.hpp>

#include <rmf_utils/catch.hpp>

// TODO(MXG): It would be good to add tests to see that the cache is behaving
// as intended, caching and using the values in the way that it should. This
// could be done by adding a proprocessor token into the cache manager header
// that enables some debug info which tells us when the cache has been used.

//==============================================================================
SCENARIO("Euclidean Heuristic -- Single Floor")
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map = "test_map";
  graph.add_waypoint(test_map, {0, 0}); // 0
  graph.add_waypoint(test_map, {1, 0}); // 1
  graph.add_waypoint(test_map, {0, 1}); // 2
  graph.add_waypoint(test_map, {1, 1}); // 3

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph));

  const double max_speed = 2.0;

  rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::EuclideanHeuristicFactory> cache_map(
        std::make_shared<rmf_traffic::agv::planning::EuclideanHeuristicFactory>(
          supergraph, max_speed));

  const Eigen::Vector2d p0 = graph.get_waypoint(0).get_location();
  const Eigen::Vector2d p1 = graph.get_waypoint(1).get_location();
  const Eigen::Vector2d p2 = graph.get_waypoint(2).get_location();
  const Eigen::Vector2d p3 = graph.get_waypoint(3).get_location();

  const std::size_t goal = 0;

  using CacheOpt = std::optional<rmf_traffic::agv::planning::Cache<
    rmf_traffic::agv::planning::EuclideanHeuristic>>;
  CacheOpt cache = cache_map.get(goal)->get();
  auto value = cache->get(goal);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(0.0).margin(1e-12));

  value = cache->get(1);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx((p1 - p0).norm()/max_speed).margin(1e-12));

  cache.reset();
  cache = cache_map.get(goal)->get();

  value = cache->get(2);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx((p2 - p0).norm()/max_speed).margin(1e-12));

  value = cache->get(3);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx((p3 - p0).norm()/max_speed).margin(1e-12));
}

//==============================================================================
SCENARIO("Euclidean Heuristic -- Easy Multifloor")
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map_0 = "test_map_0";
  graph.add_waypoint(test_map_0, {0, 0}); // 0
  graph.add_waypoint(test_map_0, {1, 0}); // 1
  graph.add_waypoint(test_map_0, {0, 1}); // 2
  graph.add_waypoint(test_map_0, {1, 1}); // 3

  const std::string test_map_1 = "test_map_1";
  graph.add_waypoint(test_map_1, {0, 0}); // 4
  graph.add_waypoint(test_map_1, {1, 0}); // 5
  graph.add_waypoint(test_map_1, {0, 1}); // 6
  graph.add_waypoint(test_map_1, {1, 1}); // 7

  const std::string test_map_2 = "test_map_2";
  graph.add_waypoint(test_map_2, {0, 0}); // 8
  graph.add_waypoint(test_map_2, {1, 0}); // 9
  graph.add_waypoint(test_map_2, {0, 1}); // 10
  graph.add_waypoint(test_map_2, {1, 1}); // 11

  const auto lift_move_duration = std::chrono::seconds(10);
  auto lift_move = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::LiftMove(
          "does not matter", "does not matter", lift_move_duration));

  graph.add_lane({3, lift_move}, 7);
  graph.add_lane({7, lift_move}, 3);
  graph.add_lane({7, lift_move}, 11);
  graph.add_lane({11, lift_move}, 7);

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph));

  const double max_speed = 2.0;

  rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::EuclideanHeuristicFactory> cache_map(
        std::make_shared<rmf_traffic::agv::planning::EuclideanHeuristicFactory>(
          supergraph, max_speed));

  const Eigen::Vector2d p0 = graph.get_waypoint(0).get_location();
  const Eigen::Vector2d p1 = graph.get_waypoint(1).get_location();
  const Eigen::Vector2d p2 = graph.get_waypoint(2).get_location();
  const Eigen::Vector2d p3 = graph.get_waypoint(3).get_location();
  const Eigen::Vector2d p7 = graph.get_waypoint(7).get_location();
  const Eigen::Vector2d p8 = graph.get_waypoint(8).get_location();
  const Eigen::Vector2d p11 = graph.get_waypoint(11).get_location();

  const std::size_t goal = 8;

  using CacheOpt = std::optional<rmf_traffic::agv::planning::Cache<
    rmf_traffic::agv::planning::EuclideanHeuristic>>;
  CacheOpt cache = cache_map.get(goal)->get();

  const auto expected_cost = [&](const Eigen::Vector2d p)
  {
    return (p3-p).norm()/max_speed
        + 2 * rmf_traffic::time::to_seconds(lift_move_duration)
        + (p8 - p11).norm()/max_speed;
  };

  auto value = cache->get(0);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p0)).margin(1e-8));

  value = cache->get(1);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p1)).margin(1e-8));

  cache.reset();
  cache = cache_map.get(goal)->get();

  value = cache->get(2);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p2)).margin(1e-8));

  value = cache->get(3);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p3)).margin(1e-8));
}

//==============================================================================
SCENARIO("Euclidean Heuristic -- Complex Multifloor")
{
  /// In this test, the planner is required to take a very roundabout path to
  /// reach the goal. It needs to go through floors 0 -> 1 -> 3 -> 2. There are
  /// multiple possible paths for moving through these, but we know the optimal
  /// path, and we expect the heuristic to find it.

  rmf_traffic::agv::Graph graph;
  const std::string test_map_0 = "test_map_0";
  graph.add_waypoint(test_map_0, {0, 0}); // 0
  graph.add_waypoint(test_map_0, {1, 0}); // 1
  graph.add_waypoint(test_map_0, {0, 1}); // 2
  graph.add_waypoint(test_map_0, {1, 1}); // 3

  const std::string test_map_1 = "test_map_1";
  graph.add_waypoint(test_map_1, {0, 0}); // 4
  graph.add_waypoint(test_map_1, {1, 0}); // 5
  graph.add_waypoint(test_map_1, {0, 1}); // 6
  graph.add_waypoint(test_map_1, {1, 1}); // 7

  const std::string test_map_2 = "test_map_2";
  graph.add_waypoint(test_map_2, {0, 0}); // 8
  graph.add_waypoint(test_map_2, {1, 0}); // 9
  graph.add_waypoint(test_map_2, {0, 1}); // 10
  graph.add_waypoint(test_map_2, {1, 1}); // 11

  const auto lift_move_duration = std::chrono::seconds(10);
  auto lift_move = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::LiftMove(
          "does not matter", "does not matter", lift_move_duration));

  auto lift_double_move = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::LiftMove(
          "does not matter", "does not matter", 2*lift_move_duration));

  // These waypoints provide the optimal path that the planner should find
  const std::string test_map_3 = "test_map_3";
  graph.add_waypoint(test_map_0, {0, 5}); // 12
  graph.add_waypoint(test_map_1, {0, 5}); // 13
  graph.add_waypoint(test_map_1, {0, 15}); // 14
  graph.add_waypoint(test_map_3, {0, 15}); // 15
  graph.add_waypoint(test_map_3, {0, 5}); // 16
  graph.add_waypoint(test_map_2, {0, 5}); // 17

  graph.add_lane({12, lift_move}, 13);
  graph.add_lane({13, lift_move}, 12);
  graph.add_lane({14, lift_double_move}, 15);
  graph.add_lane({15, lift_double_move}, 14);
  graph.add_lane({16, lift_move}, 17);
  graph.add_lane({17, lift_move}, 16);

  // These lift lanes are decoys to trick the planner
  graph.add_lane({1, lift_move}, 5);
  graph.add_lane({5, lift_move}, 1);

  graph.add_waypoint(test_map_3, {1, 0}); // 18
  graph.add_lane({18, lift_move}, 9);
  graph.add_lane({9, lift_move}, 18);

  // We intentionally do not have a reverse of this lane
  graph.add_lane({9, lift_move}, 5);

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph));

  const double max_speed = 2.0;

  rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::EuclideanHeuristicFactory> cache_map(
        std::make_shared<rmf_traffic::agv::planning::EuclideanHeuristicFactory>(
          supergraph, max_speed));

  const Eigen::Vector2d p0 = graph.get_waypoint(0).get_location();
  const Eigen::Vector2d p1 = graph.get_waypoint(1).get_location();
  const Eigen::Vector2d p2 = graph.get_waypoint(2).get_location();
  const Eigen::Vector2d p3 = graph.get_waypoint(3).get_location();
  const Eigen::Vector2d p12 = graph.get_waypoint(12).get_location();
  const Eigen::Vector2d p14 = graph.get_waypoint(14).get_location();

  const std::size_t goal = 8;

  using CacheOpt = std::optional<rmf_traffic::agv::planning::Cache<
    rmf_traffic::agv::planning::EuclideanHeuristic>>;
  CacheOpt cache = cache_map.get(goal)->get();

  const auto expected_cost = [&](const Eigen::Vector2d p)
  {
    return (p12 - p).norm()/max_speed
        + 2.0 * (p14 - p12).norm()/max_speed
        + (p0 - p12).norm()/max_speed
        + 4 * rmf_traffic::time::to_seconds(lift_move_duration);
  };

  auto value = cache->get(0);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p0)).margin(1e-8));

  value = cache->get(2);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p2)).margin(1e-8));

  // Reset the cache so it reports its results upstream
  cache.reset();
  cache = cache_map.get(goal)->get();

  value = cache->get(3);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p3)).margin(1e-8));

  // Starting from waypoint 1 is a bit different because it can go directly up
  // the lift that it's on.
  const double expected_cost_wp1 =
      (p14 - p1).norm()/max_speed
      + (p12 - p14).norm()/max_speed
      + (p0 - p12).norm()/max_speed
      + 4 * rmf_traffic::time::to_seconds(lift_move_duration);

  value = cache->get(1);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost_wp1).margin(1e-8));
}

//==============================================================================
SCENARIO("Euclidean Heuristic -- No Connection")
{
  /// In this test, there is no sequence of map transitions that allow the
  /// planner to get from any of the start waypoints to the goal. The heuristic
  /// should identify this and return a nullopt.

  rmf_traffic::agv::Graph graph;
  const std::string test_map_0 = "test_map_0";
  graph.add_waypoint(test_map_0, {0, 0}); // 0
  graph.add_waypoint(test_map_0, {1, 0}); // 1
  graph.add_waypoint(test_map_0, {0, 1}); // 2
  graph.add_waypoint(test_map_0, {1, 1}); // 3

  const std::string test_map_1 = "test_map_1";
  graph.add_waypoint(test_map_1, {0, 0}); // 4
  graph.add_waypoint(test_map_1, {1, 0}); // 5
  graph.add_waypoint(test_map_1, {0, 1}); // 6
  graph.add_waypoint(test_map_1, {1, 1}); // 7

  const std::string test_map_2 = "test_map_2";
  graph.add_waypoint(test_map_2, {0, 0}); // 8
  graph.add_waypoint(test_map_2, {1, 0}); // 9
  graph.add_waypoint(test_map_2, {0, 1}); // 10
  graph.add_waypoint(test_map_2, {1, 1}); // 11

  const auto lift_move_duration = std::chrono::seconds(10);
  auto lift_move = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::LiftMove(
          "does not matter", "does not matter", lift_move_duration));

  graph.add_lane({1, lift_move}, 5);
  graph.add_lane({5, lift_move}, 1);
  graph.add_lane({3, lift_move}, 7);
  graph.add_lane({7, lift_move}, 3);
  graph.add_lane({9, lift_move}, 5);
  graph.add_lane({11, lift_move}, 7);

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph));

  const double max_speed = 2.0;

  rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::EuclideanHeuristicFactory> cache_map(
        std::make_shared<rmf_traffic::agv::planning::EuclideanHeuristicFactory>(
          supergraph, max_speed));

  const std::size_t goal = 8;

  using CacheOpt = std::optional<rmf_traffic::agv::planning::Cache<
    rmf_traffic::agv::planning::EuclideanHeuristic>>;
  CacheOpt cache = cache_map.get(goal)->get();

  // Waypoints on a different floor than the goal have no way to get there
  auto value = cache->get(0);
  CHECK(!value.has_value());

  value = cache->get(1);
  CHECK(!value.has_value());

  cache.reset();
  cache = cache_map.get(goal)->get();

  value = cache->get(2);
  CHECK(!value.has_value());

  value = cache->get(3);
  CHECK(!value.has_value());

  value = cache->get(4);
  CHECK(!value.has_value());

  value = cache->get(5);
  CHECK(!value.has_value());

  cache.reset();
  cache = cache_map.get(goal)->get();

  value = cache->get(6);
  CHECK(!value.has_value());

  value = cache->get(7);
  CHECK(!value.has_value());

  // Waypoints on the same floor as the goal should still give values
  const Eigen::Vector2d p8 = graph.get_waypoint(8).get_location();
  const Eigen::Vector2d p9 = graph.get_waypoint(9).get_location();
  const Eigen::Vector2d p10 = graph.get_waypoint(10).get_location();
  const Eigen::Vector2d p11 = graph.get_waypoint(11).get_location();

  const auto expected_cost = [&](const Eigen::Vector2d p)
  {
    return (p8 - p).norm()/max_speed;
  };

  value = cache->get(8);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p8)).margin(1e-8));

  value = cache->get(9);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p9)).margin(1e-8));

  cache.reset();
  cache = cache_map.get(goal)->get();

  value = cache->get(10);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p10)).margin(1e-8));

  value = cache->get(11);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(p11)).margin(1e-8));
}
