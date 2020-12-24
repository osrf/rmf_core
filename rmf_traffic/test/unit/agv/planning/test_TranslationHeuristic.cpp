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

#include <src/rmf_traffic/agv/planning/TranslationHeuristic.hpp>
#include <src/rmf_traffic/agv/internal_Interpolate.hpp>

#include "../../utils_Trajectory.hpp"

#include <rmf_utils/catch.hpp>

//==============================================================================
double duration(const rmf_traffic::Trajectory& trajectory)
{
  return rmf_traffic::time::to_seconds(
        *trajectory.finish_time() - *trajectory.start_time());
}

//==============================================================================
SCENARIO("Translation Heuristic")
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map = "test_map";

  // This event does not actually take up any time, but it will force a pause
  // at every lane that contains it. As a result, the shorter path will
  // (intentionally) be forced to waste time accelerating and decelerating. That
  // slowdown will allow a longer path to require less time to reach the goal.
  const auto bogus_event = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::Wait(std::chrono::seconds(0)));

  const std::size_t N = 30;
  const std::size_t start_index = 0;
  const std::size_t goal_index = N;

  for (std::size_t i=0; i <= N; ++i)
  {
    graph.add_waypoint(test_map, {i, 0});
    if (i > 0)
    {
      graph.add_lane(i, {i-1, bogus_event});
      graph.add_lane({i-1, bogus_event}, i);
    }
  }

  const double peak = 3.0;
  for (std::size_t i=1; i < N; ++i)
  {
    double offset;
    if (i < N/2)
      offset = 2.0*static_cast<double>(i)/static_cast<double>(N) * peak;
    else
      offset = 2.0*static_cast<double>(N-i)/static_cast<double>(N) * peak;

    graph.add_waypoint(test_map, {i, offset});
    if (i > 1)
    {
      graph.add_lane(i+N, i+N-1);
      graph.add_lane(i+N-1, i+N);
    }
  }

  REQUIRE(graph.num_waypoints() == 2*N);

  // Connect the peak path to the start
  graph.add_lane(start_index, N+1);
  graph.add_lane(N+1, start_index);

  // Connect the peak path to the goal
  graph.add_lane(goal_index, 2*N-1);
  graph.add_lane(2*N-1, goal_index);

  const double v_nom = 2.0;
  const double a_nom = 0.3;
  rmf_traffic::agv::VehicleTraits traits(
    {v_nom, a_nom}, {1.0, 0.45}, create_test_profile(UnitCircle));
  traits.get_differential()->set_reversible(false);

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  using ShortestPathCache =
    rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::ShortestPathHeuristicFactory>;
  ShortestPathCache shortest_path_cache(
    std::make_shared<rmf_traffic::agv::planning::ShortestPathHeuristicFactory>(
      supergraph));

  auto shortest_path = shortest_path_cache.get(goal_index)->get();
  const auto shortest_path_result = shortest_path.get(start_index);
  REQUIRE(shortest_path_result.has_value());

  // The shortest path will use the direct line from the start to the goal
  CHECK(*shortest_path_result == Approx(30.0/v_nom).margin(1e-12));

  using TranslationCache =
    rmf_traffic::agv::planning::CacheManagerMap<
      rmf_traffic::agv::planning::TranslationHeuristicFactory>;
  TranslationCache translation_cache(
    std::make_shared<rmf_traffic::agv::planning::TranslationHeuristicFactory>(
      supergraph));

  // The translation heuristic will use the path that goes to the peak, to avoid
  // all the accelerations and decelerations needed along the shortest path.
  auto fastest_route = translation_cache.get(goal_index)->get();
  const auto fastest_route_result = fastest_route.get(start_index);
  REQUIRE(fastest_route_result.has_value());

  // We'll manually calculate the trajectory that we're expecting in order to
  // figure out what the expected duration is.
  const auto default_options = rmf_traffic::agv::Interpolate::Options();
  const Eigen::Vector3d p_start{0.0, 0.0, 0.0};
  const Eigen::Vector3d p_peak{N/2, peak, 0.0};
  const Eigen::Vector3d p_goal{N, 0.0, 0.0};

  const rmf_traffic::Trajectory trajectory =
    rmf_traffic::agv::Interpolate::positions(
      traits, std::chrono::steady_clock::now(), {p_start, p_peak, p_goal});

  const double expected_time = duration(trajectory);

  CHECK(*fastest_route_result == Approx(expected_time).margin(1e-12));
}
