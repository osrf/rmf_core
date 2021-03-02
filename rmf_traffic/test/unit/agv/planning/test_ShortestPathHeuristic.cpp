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

#include <src/rmf_traffic/agv/planning/ShortestPathHeuristic.hpp>

#include "../../utils_Trajectory.hpp"

#include <rmf_utils/catch.hpp>

// TODO(MXG): It would be good to add tests to see that the cache is behaving
// as intended, caching and using the values in the way that it should. This
// could be done by adding a proprocessor token into the cache manager header
// that enables some debug info which tells us when the cache has been used.

//==============================================================================
SCENARIO("Shortest Path Heuristic -- Single Floor")
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map = "test_map";
  graph.add_waypoint(test_map, {0, 0}); // 0
  graph.add_waypoint(test_map, {1, 0}); // 1
  graph.add_waypoint(test_map, {0, 1}); // 2
  graph.add_waypoint(test_map, {1, 1}); // 3

  graph.add_lane(0, 1);
  graph.add_lane(1, 0);
  graph.add_lane(1, 3);
  graph.add_lane(3, 1);
  graph.add_lane(3, 2);
  graph.add_lane(2, 3);
  graph.add_lane(2, 0);

  const double max_speed = 2.0;
  const rmf_traffic::agv::VehicleTraits traits(
    {max_speed, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::ShortestPathHeuristicFactory> cache_map(
        std::make_shared<
          rmf_traffic::agv::planning::ShortestPathHeuristicFactory>(
            supergraph));

  using CacheOpt = std::optional<rmf_traffic::agv::planning::Cache<
    rmf_traffic::agv::planning::ShortestPathHeuristic>>;
  CacheOpt cache = cache_map.get(2)->get();

  auto value = cache->get(0);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(3.0/max_speed).margin(1e-12));

  value = cache->get(1);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(2.0/max_speed).margin(1e-12));

  cache.reset();
  cache = cache_map.get(2)->get();

  value = cache->get(2);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(0.0).margin(1e-12));

  value = cache->get(3);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(1.0/max_speed).margin(1e-12));

  cache = cache_map.get(0)->get();
  value = cache->get(2);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(1.0/max_speed).margin(1e-12));
}

//==============================================================================
SCENARIO("Shortest Path Heuristic -- Easy Multifloor")
{
  rmf_traffic::agv::Graph graph;

  const auto add_bidir_lane = [&](std::size_t i, std::size_t j)
  {
    graph.add_lane(i, j);
    graph.add_lane(j, i);
  };

  const std::string test_map_0 = "test_map_0";
  graph.add_waypoint(test_map_0, {0, 0}); // 0
  graph.add_waypoint(test_map_0, {1, 0}); // 1
  graph.add_waypoint(test_map_0, {0, 1}); // 2
  graph.add_waypoint(test_map_0, {1, 1}); // 3

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 3);
  // We intentionally leave out 2->3 to make the test more interesting
  graph.add_lane(3, 2);
  add_bidir_lane(2, 0);

  const std::string test_map_1 = "test_map_1";
  graph.add_waypoint(test_map_1, {0, 0}); // 4
  graph.add_waypoint(test_map_1, {1, 0}); // 5
  graph.add_waypoint(test_map_1, {0, 1}); // 6
  graph.add_waypoint(test_map_1, {1, 1}); // 7

  add_bidir_lane(4, 5);
  add_bidir_lane(5, 7);
  add_bidir_lane(7, 6);
  add_bidir_lane(6, 4);

  const std::string test_map_2 = "test_map_2";
  graph.add_waypoint(test_map_2, {0, 0}); // 8
  graph.add_waypoint(test_map_2, {1, 0}); // 9
  graph.add_waypoint(test_map_2, {0, 1}); // 10
  graph.add_waypoint(test_map_2, {1, 1}); // 11

  add_bidir_lane(8, 9);
  add_bidir_lane(9, 11);
  add_bidir_lane(11, 10);
  add_bidir_lane(10, 8);

  const auto lift_move_duration = std::chrono::seconds(10);
  auto lift_move = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::LiftMove(
          "does not matter", "does not matter", lift_move_duration));

  graph.add_lane({3, lift_move}, 7);
  graph.add_lane({7, lift_move}, 3);
  graph.add_lane({7, lift_move}, 11);
  graph.add_lane({11, lift_move}, 7);

  const double max_speed = 2.0;
  const rmf_traffic::agv::VehicleTraits traits(
    {max_speed, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::ShortestPathHeuristicFactory> cache_map(
        std::make_shared<
          rmf_traffic::agv::planning::ShortestPathHeuristicFactory>(
            supergraph));

  const std::size_t goal = 8;

  using CacheOpt = std::optional<rmf_traffic::agv::planning::Cache<
    rmf_traffic::agv::planning::ShortestPathHeuristic>>;
  CacheOpt cache = cache_map.get(goal)->get();

  const auto expected_cost = [&](const std::size_t steps)
  {
    return static_cast<double>(steps)/max_speed + 2/max_speed
        + 2 * rmf_traffic::time::to_seconds(lift_move_duration);
  };

  auto value = cache->get(0);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(2)).margin(1e-12));

  value = cache->get(1);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(1)).margin(1e-12));

  cache.reset();
  cache = cache_map.get(goal)->get();

  value = cache->get(2);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(3)).margin(1e-12));

  value = cache->get(3);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(0)).margin(1e-12));
}

//==============================================================================
SCENARIO("Shortest Path Heuristic -- Complex Multifloor")
{
  /// In this test, the planner is required to take a very roundabout path to
  /// reach the goal. It needs to go through floors 0 -> 1 -> 3 -> 2. There are
  /// multiple possible paths for moving through these, but we know the optimal
  /// path, and we expect the heuristic to find it.

  rmf_traffic::agv::Graph graph;

  const auto add_bidir_lane = [&](std::size_t i, std::size_t j)
  {
    graph.add_lane(i, j);
    graph.add_lane(j, i);
  };

  const std::string test_map_0 = "test_map_0";
  graph.add_waypoint(test_map_0, {0, 0}); // 0
  graph.add_waypoint(test_map_0, {1, 0}); // 1
  graph.add_waypoint(test_map_0, {0, 1}); // 2
  graph.add_waypoint(test_map_0, {1, 1}); // 3

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 3);
  add_bidir_lane(3, 2);
  add_bidir_lane(2, 0);

  const std::string test_map_1 = "test_map_1";
  graph.add_waypoint(test_map_1, {0, 0}); // 4
  graph.add_waypoint(test_map_1, {1, 0}); // 5
  graph.add_waypoint(test_map_1, {0, 1}); // 6
  graph.add_waypoint(test_map_1, {1, 1}); // 7

  add_bidir_lane(4, 5);
  add_bidir_lane(5, 7);
  add_bidir_lane(7, 6);
  add_bidir_lane(6, 4);

  const std::string test_map_2 = "test_map_2";
  graph.add_waypoint(test_map_2, {0, 0}); // 8
  graph.add_waypoint(test_map_2, {1, 0}); // 9
  graph.add_waypoint(test_map_2, {0, 1}); // 10
  graph.add_waypoint(test_map_2, {1, 1}); // 11

  add_bidir_lane(8, 9);
  add_bidir_lane(9, 11);
  add_bidir_lane(11, 10);
  add_bidir_lane(10, 8);

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

  add_bidir_lane(2, 12);
  add_bidir_lane(13, 14);
  add_bidir_lane(15, 16);
  add_bidir_lane(17, 10);

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

  const double max_speed = 2.0;
  const rmf_traffic::agv::VehicleTraits traits(
    {max_speed, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::ShortestPathHeuristicFactory> cache_map(
        std::make_shared<
          rmf_traffic::agv::planning::ShortestPathHeuristicFactory>(
            supergraph));

  const std::size_t goal = 8;

  using CacheOpt = std::optional<rmf_traffic::agv::planning::Cache<
    rmf_traffic::agv::planning::ShortestPathHeuristic>>;
  CacheOpt cache = cache_map.get(goal)->get();

  const auto expected_cost = [&](const std::size_t steps)
  {
    return (static_cast<double>(steps) + 29.0)/max_speed
        + 4 * rmf_traffic::time::to_seconds(lift_move_duration);
  };

  auto value = cache->get(3);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(1)).margin(1e-12));

  value = cache->get(0);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(1)).margin(1e-12));

  value = cache->get(1);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(2)).margin(1e-12));

  // Reset the cache so it reports its results upstream
  cache.reset();
  cache = cache_map.get(goal)->get();

  value = cache->get(2);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(0)).margin(1e-12));
}

//==============================================================================
SCENARIO("Shortest Path Heuristic -- No Connection")
{
  /// In this test, there is no sequence of map transitions that allow the
  /// planner to get from any of the start waypoints to the goal. The heuristic
  /// should identify this and return a nullopt.

  rmf_traffic::agv::Graph graph;

  const auto add_bidir_lane = [&](std::size_t i, std::size_t j)
  {
    graph.add_lane(i, j);
    graph.add_lane(j, i);
  };

  const std::string test_map_0 = "test_map_0";
  graph.add_waypoint(test_map_0, {0, 0}); // 0
  graph.add_waypoint(test_map_0, {1, 0}); // 1
  graph.add_waypoint(test_map_0, {0, 1}); // 2
  graph.add_waypoint(test_map_0, {1, 1}); // 3

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 3);
  add_bidir_lane(3, 2);
  add_bidir_lane(2, 0);

  const std::string test_map_1 = "test_map_1";
  graph.add_waypoint(test_map_1, {0, 0}); // 4
  graph.add_waypoint(test_map_1, {1, 0}); // 5
  graph.add_waypoint(test_map_1, {0, 1}); // 6
  graph.add_waypoint(test_map_1, {1, 1}); // 7

  add_bidir_lane(4, 5);
  add_bidir_lane(5, 7);
  add_bidir_lane(7, 6);
  add_bidir_lane(6, 4);

  const std::string test_map_2 = "test_map_2";
  graph.add_waypoint(test_map_2, {0, 0}); // 8
  graph.add_waypoint(test_map_2, {1, 0}); // 9
  graph.add_waypoint(test_map_2, {0, 1}); // 10
  graph.add_waypoint(test_map_2, {1, 1}); // 11

  add_bidir_lane(8, 9);
  add_bidir_lane(9, 11);
  // 10 intentionally does not connect to anything

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

  const double max_speed = 2.0;
  const rmf_traffic::agv::VehicleTraits traits(
    {max_speed, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));

  auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  auto cache_map =
    std::make_optional<rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::ShortestPathHeuristicFactory>>(
        std::make_shared<
          rmf_traffic::agv::planning::ShortestPathHeuristicFactory>(
            supergraph));

  const std::size_t goal = 8;

  const auto expected_cost = [&](
      const std::size_t steps,
      const std::size_t floors)
  {
    return static_cast<double>(steps)/max_speed
        + floors*rmf_traffic::time::to_seconds(lift_move_duration);
  };

  using CacheOpt = std::optional<rmf_traffic::agv::planning::Cache<
    rmf_traffic::agv::planning::ShortestPathHeuristic>>;
  CacheOpt cache = cache_map->get(goal)->get();

  // Waypoints on a different floor than the goal have no way to get there
  auto value = cache->get(0);
  CHECK(!value.has_value());

  value = cache->get(1);
  CHECK(!value.has_value());

  cache.reset();
  cache = cache_map->get(goal)->get();

  value = cache->get(2);
  CHECK(!value.has_value());

  value = cache->get(3);
  CHECK(!value.has_value());

  value = cache->get(4);
  CHECK(!value.has_value());

  value = cache->get(5);
  CHECK(!value.has_value());

  cache.reset();
  cache = cache_map->get(goal)->get();

  value = cache->get(6);
  CHECK(!value.has_value());

  value = cache->get(7);
  CHECK(!value.has_value());

  // Some waypoints on the same floor can connect
  value = cache->get(8);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(0.0).margin(1e-12));

  value = cache->get(9);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(1, 0)).margin(1e-12));

  value = cache->get(10);
  CHECK(!value.has_value());

  value = cache->get(11);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(2, 0)).margin(1e-12));

  // Add new lifts to the graph so the waypoints can reach floor 2. That will
  // make waypoint 8 reachable, but waypoint 10 will still be unreachable.
  graph.add_lane({5, lift_move}, 9);
  graph.add_lane({7, lift_move}, 11);

  supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  // We need to completely reconstruct the cache map, because the supergraph has
  // changed.
  cache_map.reset();
  cache_map.emplace(
    std::make_shared<
      rmf_traffic::agv::planning::ShortestPathHeuristicFactory>(supergraph));

  cache = cache_map->get(goal)->get();

  value = cache->get(0);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(2, 2)).margin(1e-12));

  value = cache->get(1);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(1, 2)).margin(1e-12));

  value = cache->get(2);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(3, 2)).margin(1e-12));

  value = cache->get(3);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(2, 2)).margin(1e-12));

  value = cache->get(4);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(2, 1)).margin(1e-12));

  value = cache->get(5);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(1, 1)).margin(1e-12));

  value = cache->get(6);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(3, 1)).margin(1e-12));

  value = cache->get(7);
  REQUIRE(value.has_value());
  CHECK(value.value() == Approx(expected_cost(2, 1)).margin(1e-12));

  // They still cannot reach waypoint 10
  cache = cache_map->get(10)->get();

  value = cache->get(0);
  CHECK(!value.has_value());

  value = cache->get(1);
  CHECK(!value.has_value());

  cache.reset();
  cache = cache_map->get(10)->get();

  value = cache->get(2);
  CHECK(!value.has_value());

  value = cache->get(3);
  CHECK(!value.has_value());

  value = cache->get(4);
  CHECK(!value.has_value());

  value = cache->get(5);
  CHECK(!value.has_value());

  cache.reset();
  cache = cache_map->get(10)->get();

  value = cache->get(6);
  CHECK(!value.has_value());

  value = cache->get(7);
  CHECK(!value.has_value());
}

