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

#include <src/rmf_traffic/agv/planning/Supergraph.hpp>

#include "../../utils_Trajectory.hpp"

#include <rmf_utils/catch.hpp>

// TODO(MXG): It would be good to add tests to see that the cache is behaving
// as intended, caching and using the values in the way that it should. This
// could be done by adding a proprocessor token into the cache manager header
// that enables some debug info which tells us when the cache has been used.

// TODO(MXG): We could add tests for the Traversal::best_time field

// TODO(MXG): We could add tests for multiple floors

//==============================================================================
std::size_t count_alternatives(
    const rmf_traffic::agv::planning::Traversals& traversals)
{
  std::size_t alternatives_counter = 0;
  for (const auto& traversal : traversals)
  {
    for (const auto& alt : traversal.alternatives)
    {
      if (!alt.has_value())
        continue;

      ++alternatives_counter;
    }
  }

  return alternatives_counter;
}

//==============================================================================
bool has_only_map(
    const std::string& reference_map,
    const rmf_traffic::agv::planning::Traversals& traversals)
{
  bool at_least_one = false;
  for (const auto& traversal : traversals)
  {
    for (const auto& alt : traversal.alternatives)
    {
      if (!alt.has_value())
        continue;

      for (const auto& route : alt->routes)
      {
        if (route.map() != reference_map)
          return false;

        at_least_one = true;
      }
    }
  }

  return at_least_one;
}

//==============================================================================
SCENARIO("Supergraph -- Single Floor, Reversible")
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map = "test_map";
  graph.add_waypoint(test_map, {0, 0}); // 0
  graph.add_waypoint(test_map, {1, 1}); // 1
  graph.add_waypoint(test_map, {2, 2}); // 2
  graph.add_waypoint(test_map, {3, 3}); // 3

  graph.add_lane(0, 1);
  graph.add_lane(1, 0);
  graph.add_lane(1, 2);
  graph.add_lane(2, 1);
  graph.add_lane(2, 3);
  graph.add_lane(3, 2);

  graph.add_waypoint(test_map, {0, 1}); // 4
  graph.add_waypoint(test_map, {0, 2}); // 5
  graph.add_waypoint(test_map, {0, 3}); // 6

  const auto forward_constraint =
      rmf_traffic::agv::Graph::OrientationConstraint::make(
        rmf_traffic::agv::Graph::OrientationConstraint::Direction::Forward,
        Eigen::Vector2d::UnitX());

  const auto backward_constraint =
      rmf_traffic::agv::Graph::OrientationConstraint::make(
        rmf_traffic::agv::Graph::OrientationConstraint::Direction::Backward,
        Eigen::Vector2d::UnitX());

  graph.add_lane(0, 4);
  graph.add_lane(4, 0);
  graph.add_lane({4, nullptr, forward_constraint}, 5);
  graph.add_lane({5, nullptr, backward_constraint}, 4);
  graph.add_lane(5, 6);

  graph.add_waypoint(test_map, {1, 0}); // 7
  graph.add_waypoint(test_map, {2, 0}); // 8
  graph.add_waypoint(test_map, {3, 0}); // 9

  graph.add_lane(0, 7);
  graph.add_lane(7, {8, nullptr, forward_constraint});
  graph.add_lane({8, nullptr, backward_constraint}, 9);

  // Add a significant turn
  graph.add_waypoint(test_map, {4, 3}); // 10
  graph.add_lane(3, 10);
  graph.add_lane(10, 3);

  // Add a standstill
  graph.add_waypoint(test_map, {4, 3}); // 11
  graph.add_lane(10, 11);
  graph.add_lane(11, 10);

  // Add a follow-on from the standstill
  graph.add_waypoint(test_map, {5, 3}); // 12
  graph.add_lane(11, 12);
  graph.add_lane(12, 11);

  const double max_speed = 2.0;
  const rmf_traffic::agv::VehicleTraits traits(
    {max_speed, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  auto traversals = supergraph->traversals_from(0);
  REQUIRE(traversals);
  CHECK(traversals->size() == 8);
  CHECK(count_alternatives(*traversals) == 13);
  CHECK(has_only_map(test_map, *traversals));

  traversals = supergraph->traversals_from(1);
  REQUIRE(traversals);
  CHECK(traversals->size() == 3);
  CHECK(count_alternatives(*traversals) == 6);
  CHECK(has_only_map(test_map, *traversals));

  traversals = supergraph->traversals_from(2);
  REQUIRE(traversals);
  CHECK(traversals->size() == 3);
  CHECK(count_alternatives(*traversals) == 6);
  CHECK(has_only_map(test_map, *traversals));

  traversals = supergraph->traversals_from(3);
  REQUIRE(traversals);
  CHECK(traversals->size() == 6);
  CHECK(count_alternatives(*traversals) == 12);
  CHECK(has_only_map(test_map, *traversals));

  traversals = supergraph->traversals_from(4);
  REQUIRE(traversals);
  CHECK(traversals->size() == 3);
  CHECK(count_alternatives(*traversals) == 4);
  CHECK(has_only_map(test_map, *traversals));

  traversals = supergraph->traversals_from(5);
  REQUIRE(traversals);
  CHECK(traversals->size() == 3);
  CHECK(count_alternatives(*traversals) == 4);
  CHECK(has_only_map(test_map, *traversals));

  traversals = supergraph->traversals_from(6);
  REQUIRE(traversals);
  CHECK(traversals->size() == 0);
  CHECK(count_alternatives(*traversals) == 0);
  // This traversal has no route in it, so it won't contain any map

  traversals = supergraph->traversals_from(7);
  REQUIRE(traversals);
  CHECK(traversals->size() == 1);
  CHECK(count_alternatives(*traversals) == 1);
  CHECK(has_only_map(test_map, *traversals));

  traversals = supergraph->traversals_from(8);
  REQUIRE(traversals);
  CHECK(traversals->size() == 1);
  CHECK(count_alternatives(*traversals) == 1);
  CHECK(has_only_map(test_map, *traversals));

  traversals = supergraph->traversals_from(9);
  REQUIRE(traversals);
  CHECK(traversals->size() == 0);
  CHECK(count_alternatives(*traversals) == 0);
  // This traversal has no route in it, so it won't contain any map

  traversals = supergraph->traversals_from(10);
  REQUIRE(traversals);
  CHECK(traversals->size() == 3);
  CHECK(count_alternatives(*traversals) == 5);
  CHECK(has_only_map(test_map, *traversals));
}

//==============================================================================
SCENARIO("Supergraph -- Single Floor, Events, Irreversible")
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map = "test_map";
  graph.add_waypoint(test_map, {0, 0}); // 0
  graph.add_waypoint(test_map, {-1, -1}); // 1
  graph.add_waypoint(test_map, {-2, -2}); // 2
  graph.add_waypoint(test_map, {-3, -3}); // 3
  graph.add_waypoint(test_map, {-4, -4}); // 4

  const auto wait = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::Wait(std::chrono::seconds(1)));

  graph.add_lane(0, 1);
  graph.add_lane(1, {2, wait});
  graph.add_lane(2, 3);
  graph.add_lane(3, 4);

  graph.add_waypoint(test_map, {1, 0}); // 5
  graph.add_waypoint(test_map, {2, 0}); // 6
  graph.add_waypoint(test_map, {3, 0}); // 7
  graph.add_waypoint(test_map, {4, 0}); // 8

  graph.add_lane(0, 5);
  graph.add_lane(5, 6);
  graph.add_lane({6, wait}, 7);
  graph.add_lane(7, 8);

  graph.add_waypoint(test_map, {0, 1}); // 9
  graph.add_waypoint(test_map, {0, 2}); // 10
  graph.add_waypoint(test_map, {0, 3}); // 11
  graph.add_waypoint(test_map, {0, 4}); // 12

  const auto forward_constraint =
      rmf_traffic::agv::Graph::OrientationConstraint::make(
        rmf_traffic::agv::Graph::OrientationConstraint::Direction::Forward,
        Eigen::Vector2d::UnitX());

  const auto backward_constraint =
      rmf_traffic::agv::Graph::OrientationConstraint::make(
        rmf_traffic::agv::Graph::OrientationConstraint::Direction::Backward,
        Eigen::Vector2d::UnitX());

  graph.add_lane(0, 9);
  graph.add_lane(9, {10, nullptr, forward_constraint});
  graph.add_lane(10, {11, nullptr, backward_constraint});
  graph.add_lane(11, 12);

  const double max_speed = 2.0;
  rmf_traffic::agv::VehicleTraits traits(
    {max_speed, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));
  traits.get_differential()->set_reversible(false);

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  auto traversals = supergraph->traversals_from(0);
  REQUIRE(traversals);
  CHECK(traversals->size() == 6);
  CHECK(count_alternatives(*traversals) == 6);
  CHECK(has_only_map(test_map, *traversals));
}
