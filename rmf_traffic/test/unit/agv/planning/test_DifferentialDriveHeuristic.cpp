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

#include <src/rmf_traffic/agv/planning/DifferentialDriveHeuristic.hpp>

#include "../../utils_Trajectory.hpp"

#include <rmf_utils/catch.hpp>

#include <variant>


#include <iostream>

//==============================================================================
// This is cruft needed for C++17. It should be removed when we migrate to C++20
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

//==============================================================================
struct Move
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<std::string> maps;
};

//==============================================================================
struct Wait
{
  rmf_traffic::Duration duration;
  std::vector<std::string> maps;
};

//==============================================================================
using Action = std::variant<Move, Wait>;

//==============================================================================
using SolutionNodePtr =
  rmf_traffic::agv::planning::DifferentialDriveMapTypes::SolutionNodePtr;

//==============================================================================
bool compare_routes(
  const rmf_traffic::Route& a,
  const rmf_traffic::Route& b)
{
  const auto initial_time =
      std::min(*a.trajectory().start_time(), *b.trajectory().start_time());

  std::cout << "[" << a.map() << "] | [" << b.map() << "]" << std::endl;

//  if (a.trajectory().size() != b.trajectory().size())
  {
    for (std::size_t i=0; i < std::max(a.trajectory().size(), b.trajectory().size()); ++i)
    {
      std::vector<const rmf_traffic::Trajectory::Waypoint*> wps;
      if (i < a.trajectory().size())
        wps.push_back(&a.trajectory()[i]);
      if (i < b.trajectory().size())
        wps.push_back(&b.trajectory()[i]);

      for (const auto& wp : wps)
      {
        std::cout << "(" << rmf_traffic::time::to_seconds(wp->time() - initial_time)
                  << "; " << wp->position().transpose() << "; "
                  << wp->velocity().transpose() << ")  |  ";
      }

      std::cout << std::endl;
    }
    std::cout << " -- " << std::endl;
  }

  REQUIRE(a.map() == b.map());
  REQUIRE(a.trajectory().size() == b.trajectory().size());

  bool all_correct = a.map() == b.map();
  for (std::size_t i=0; i < a.trajectory().size(); ++i)
  {
    const auto& wp_a = a.trajectory().at(i);
    const auto& wp_b = b.trajectory().at(i);

    const double time_diff =
        rmf_traffic::time::to_seconds(wp_a.time() - wp_b.time());
    const bool time_matches = time_diff == Approx(0.0).margin(1e-8);
    CHECK(time_matches);

    const Eigen::Vector3d p_a = wp_a.position();
    const Eigen::Vector3d p_b = wp_b.position();
    const bool positions_match = (p_a - p_b).norm() == Approx(0.0).margin(1e-8);
    CHECK(positions_match);

    const Eigen::Vector3d v_a = wp_a.velocity();
    const Eigen::Vector3d v_b = wp_b.velocity();
    const bool velocities_match =
        (v_a - v_b).norm() == Approx(0.0).margin(1e-8);

    CHECK(velocities_match);

    all_correct &= time_matches && positions_match && velocities_match;
  }

  return all_correct;
}

//==============================================================================
bool compare_plan(
    const rmf_traffic::agv::VehicleTraits& traits,
    const Eigen::Vector3d initial_position,
    const std::vector<Action>& actions,
    SolutionNodePtr solution)
{
  REQUIRE(solution);

  const rmf_traffic::Time start_time = std::chrono::steady_clock::now();
  rmf_traffic::Time time = start_time;
  double yaw = initial_position[2];
  std::vector<rmf_traffic::Route> routes;
  while (solution)
  {
    REQUIRE(solution->route_factory);

    auto new_route_info = solution->route_factory(time, yaw);
    routes.insert(
          routes.end(),
          new_route_info.routes.begin(),
          new_route_info.routes.end());
    time = new_route_info.finish_time;
    yaw = new_route_info.finish_yaw;

    solution = solution->child;
  }

  Eigen::Vector3d position = initial_position;
  time = start_time;
  std::size_t counter = 0;
  bool all_correct = true;
  for (const auto& action : actions)
  {
    std::visit(
      overloaded {
        [&](const Move& move)
        {
          auto positions = move.positions;
          positions.insert(positions.begin(), position);
          const auto trajectory = rmf_traffic::agv::Interpolate::positions(
              traits, time, positions);

          for (const auto& map : move.maps)
          {
            const bool routes_equal =
                compare_routes({map, trajectory}, routes.at(counter++));
            CHECK(routes_equal);
            all_correct &= routes_equal;
          }

          time = trajectory.back().time();
          position = trajectory.back().position();
        },
        [&](const Wait& wait)
        {
          rmf_traffic::Trajectory trajectory;
          trajectory.insert(time, position, Eigen::Vector3d::Zero());
          trajectory.insert(
              time + wait.duration, position, Eigen::Vector3d::Zero());

          for (const auto& map : wait.maps)
          {
            const bool routes_equal =
                compare_routes({map, trajectory}, routes.at(counter++));
            CHECK(routes_equal);
            all_correct &= routes_equal;
          }

          time = trajectory.back().time();
          position = trajectory.back().position();
        }
      }, action);
  }

  return all_correct;
}

//==============================================================================
SCENARIO("Differential Drive Heuristic -- Peak and Valley")
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map = "test_map";

  // This event does not actually take up any time, but it will force a pause
  // at every lane that contains it. As a result, the shorter path will
  // (intentionally) be forced to waste time accelerating and decelerating. That
  // slowdown will allow a longer path to require less time to reach the goal.
  const auto bogus_event = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::Wait(std::chrono::seconds(1)));

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

  const double forward_incline_up = std::atan2(peak, N/2);
  const double forward_incline_down = -forward_incline_up;
  const double backward_incline_up = forward_incline_up - 180._deg;
  const double backward_incline_down = 180._deg - forward_incline_up;

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
  traits.get_differential()->set_reversible(true);

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  auto diff_drive_cache =
      rmf_traffic::agv::planning
      ::DifferentialDriveHeuristic::make_manager(supergraph);

  using Ori = rmf_traffic::agv::planning::Orientation;
  using Side = rmf_traffic::agv::planning::Side;
  using Key = rmf_traffic::agv::planning::DifferentialDriveMapTypes::Key;

  const auto keys = supergraph->keys_for(0, goal_index, std::nullopt);
  CHECK(keys.size() == 16);

  CHECK(keys.count({116, Ori::Backward, Side::Start, 59, Ori::Backward}));
  CHECK(keys.count({116, Ori::Backward, Side::Start, 59, Ori::Forward}));
  CHECK(keys.count({116, Ori::Backward, Side::Start, 119, Ori::Backward}));
  CHECK(keys.count({116, Ori::Forward, Side::Start, 59, Ori::Backward}));
  CHECK(keys.count({116, Ori::Forward, Side::Start, 119, Ori::Backward}));
  CHECK(keys.count({116, Ori::Backward, Side::Start, 119, Ori::Forward}));
  CHECK(keys.count({116, Ori::Forward, Side::Start, 59, Ori::Forward}));
  CHECK(keys.count({116, Ori::Forward, Side::Start, 119, Ori::Forward}));
  CHECK(keys.count({1, Ori::Backward, Side::Start, 59, Ori::Backward}));
  CHECK(keys.count({1, Ori::Backward, Side::Start, 59, Ori::Forward}));
  CHECK(keys.count({1, Ori::Backward, Side::Start, 119, Ori::Backward}));
  CHECK(keys.count({1, Ori::Forward, Side::Start, 59, Ori::Backward}));
  CHECK(keys.count({1, Ori::Forward, Side::Start, 119, Ori::Backward}));
  CHECK(keys.count({1, Ori::Backward, Side::Start, 119, Ori::Forward}));
  CHECK(keys.count({1, Ori::Forward, Side::Start, 59, Ori::Forward}));
  CHECK(keys.count({1, Ori::Forward, Side::Start, 119, Ori::Forward}));


  const double initial_yaw = 0._deg;

  {
    const Key key{116, Ori::Backward, Side::Start, 59, Ori::Backward};
    const Eigen::Vector3d initial_position = {0.0, 0.0, initial_yaw};
    std::vector<Action> actions;

    actions.push_back(
      Move{
        {{0.0, 0.0, backward_incline_up}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{15.0, 3.0, backward_incline_up}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{15.0, 3.0, backward_incline_down}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{30.0, 0.0, backward_incline_down}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{30.0, 0.0, 180._deg}},
        {test_map}
      });

    const auto solution = diff_drive_cache->get().get(key);
    CHECK(compare_plan(traits, initial_position, actions, solution));
  }

  {
    const Key key{116, Ori::Forward, Side::Start, 119, Ori::Backward};
    const Eigen::Vector3d initial_position = {0.0, 0.0, initial_yaw};
    std::vector<Action> actions;

    actions.push_back(
      Move{
        {{0.0, 0.0, forward_incline_up}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{15.0, 3.0, forward_incline_up}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{15.0, 3.0, backward_incline_down}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{30.0, 0.0, backward_incline_down}},
        {test_map}
      });

    const auto solution = diff_drive_cache->get().get(key);
    CHECK(compare_plan(traits, initial_position, actions, solution));
  }

  std::cout << " vvvvvvvvvvvvvv BEGIN vvvvvvvvvvvvvvvvvvv" << std::endl;
  {
    const Key key{1, Ori::Backward, Side::Start, 119, Ori::Forward};
    const Eigen::Vector3d initial_position = {0.0, 0.0, initial_yaw};
    std::vector<Action> actions;

    actions.push_back(
      Move{
        {{0.0, 0.0, M_PI}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{0.0, 0.0, backward_incline_up}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{15.0, 3.0, backward_incline_up}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{15.0, 3.0, forward_incline_down}},
        {test_map}
      });

    actions.push_back(
      Move{
        {{30.0, 0.0, forward_incline_down}},
        {test_map}
      });

    const auto solution = diff_drive_cache->get().get(key);
    CHECK(compare_plan(traits, initial_position, actions, solution));
  }
}

//==============================================================================
SCENARIO("Differential Drive Heuristic -- Indeterminate Yaw Edge Case")
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map_0 = "test_map_0";
  const std::string test_map_1 = "test_map_1";
  const std::string test_map_2 = "test_map_2";

  graph.add_waypoint(test_map_0, {0, 0}); // 0
  graph.add_waypoint(test_map_1, {0, 0}); // 1
  graph.add_waypoint(test_map_2, {0, 0}); // 2
  graph.add_waypoint(test_map_2, {0, 1}); // 3

  const auto bogus_event_duration = std::chrono::seconds(1);
  const auto bogus_event = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::Wait(bogus_event_duration));

  graph.add_lane({0, bogus_event}, 1); // 0
  graph.add_lane({1, bogus_event}, 2); // 1
  graph.add_lane(2, 3); // 2

  const std::size_t goal_index = 3;

  rmf_traffic::agv::VehicleTraits traits(
    {2.0, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));
  traits.get_differential()->set_reversible(true);

  const auto supergraph = rmf_traffic::agv::planning::Supergraph::make(
        rmf_traffic::agv::Graph::Implementation::get(graph),
        traits, rmf_traffic::agv::Interpolate::Options());

  // TODO(MXG): Make a cleaner way to instantiate these caches
  using DifferentialDriveCache =
    rmf_traffic::agv::planning::CacheManager<
      rmf_traffic::agv::planning::Cache<
        rmf_traffic::agv::planning::DifferentialDriveHeuristic>>;
  auto diff_drive_cache = DifferentialDriveCache::make(
    std::make_shared<
      rmf_traffic::agv::planning::DifferentialDriveHeuristic>(
          supergraph), [N = supergraph->original().lanes.size()]()
  {
    return rmf_traffic::agv::planning::DifferentialDriveHeuristic::Storage(
      4093, rmf_traffic::agv::planning::DifferentialDriveMapTypes::KeyHash{N});
  });

  using Ori = rmf_traffic::agv::planning::Orientation;
  using Side = rmf_traffic::agv::planning::Side;
  using Key = rmf_traffic::agv::planning::DifferentialDriveMapTypes::Key;

  const auto keys = supergraph->keys_for(0, goal_index, std::nullopt);
  CHECK(keys.size() == 2);
  CHECK(keys.count({0, Ori::Any, Side::Start, 2, Ori::Forward}));
  CHECK(keys.count({0, Ori::Any, Side::Start, 2, Ori::Backward}));

  const double initial_yaw = 10._deg;

  {
    const Key key{0, Ori::Any, Side::Start, 2, Ori::Forward};
    const Eigen::Vector3d initial_position = {0.0, 0.0, initial_yaw};
    std::vector<Action> actions;

    // TODO(MXG): Figure out why there's a duplicate single-element route added
    // by the solution (represented below by having test_map_0 twice). If this
    // aspect of the test fails in the future due to any code changes, we can
    // remove this test expectation.
    actions.push_back(
      Move{
        {{0.0, 0.0, initial_yaw}},
        {test_map_0, test_map_0}
      });

    actions.push_back(
      Wait{
        bogus_event_duration,
        {test_map_0, test_map_1}
      });

    actions.push_back(
      Move{
        {{0.0, 0.0, 90._deg}},
        {test_map_0, test_map_1}
      });

    // I think this single-element route gets added by the node that triggers
    // the bogus_event to begin.
    actions.push_back(
      Move{
        {{0.0, 0.0, 90._deg}},
        {test_map_1}
      });

    actions.push_back(
      Wait{
        bogus_event_duration,
        {test_map_1, test_map_2}
      });

    actions.push_back(
      Move{
        {{0.0, 1.0, 90._deg}},
        {test_map_1, test_map_2}
      });

    const auto solution = diff_drive_cache->get().get(key);
    CHECK(compare_plan(traits, initial_position, actions, solution));
  }

  {
    const Key key{0, Ori::Any, Side::Start, 2, Ori::Backward};
    const Eigen::Vector3d initial_position = {0.0, 0.0, initial_yaw};
    std::vector<Action> actions;

    // TODO(MXG): Figure out why there's a duplicate single-element route added
    // by the solution.
    actions.push_back(
      Move{
        {initial_position},
        {test_map_0, test_map_0}
      });

    actions.push_back(
      Wait{
        bogus_event_duration,
        {test_map_0, test_map_1}
      });

    actions.push_back(
      Move{
        {{0.0, 0.0, -90._deg}},
        {test_map_0, test_map_1}
      });

    // I think this single-element route gets added by the node that triggers
    // the bogus_event to begin.
    actions.push_back(
      Move{
        {{0.0, 0.0, -90._deg}},
        {test_map_1}
      });

    actions.push_back(
      Wait{
        bogus_event_duration,
        {test_map_1, test_map_2}
      });

    actions.push_back(
      Move{
        {{0.0, 1.0, -90._deg}},
        {test_map_1, test_map_2}
      });

    const auto solution = diff_drive_cache->get().get(key);
    CHECK(compare_plan(traits, initial_position, actions, solution));
  }
}
