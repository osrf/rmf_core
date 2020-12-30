
#include <src/rmf_traffic/agv/planning/DifferentialDriveHeuristic.hpp>

#include "test/unit/utils_Trajectory.hpp"


#include <iostream>


//==============================================================================
inline rmf_traffic::Time print_start(const rmf_traffic::Route& route)
{
  assert(route.trajectory().size() > 0);
  auto p = route.trajectory().front().position();
  p[2] *= 180.0/M_PI;
  std::cout << "(start) --> ";
  std::cout << "(" << 0.0 << "; "
            << p.transpose()
            << ") --> ";

  return *route.trajectory().start_time();
}

//==============================================================================
inline void print_route(
    const rmf_traffic::Route& route,
    const rmf_traffic::Time start_time)
{
  assert(route.trajectory().size() > 0);
  std::cout << "[" << route.map() << "] ";
//  for (auto it = ++route.trajectory().begin(); it
  for (auto it = route.trajectory().begin(); it
       != route.trajectory().end(); ++it)
  {
    const auto& wp = *it;
    if (wp.velocity().norm() > 1e-3)
      continue;

    const auto rel_time = wp.time() - start_time;
    auto p = wp.position();
    p[2] *= 180.0/M_PI;
    std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
              << p.transpose() << ") --> ";
  }
}

//==============================================================================
inline void print_itinerary(const std::vector<rmf_traffic::Route>& itinerary)
{
  if (itinerary.empty())
  {
    std::cout << "No plan needed!" << std::endl;
  }
  else
  {
//    auto start_time = print_start(itinerary.front());
    const auto start_time = *itinerary.front().trajectory().start_time();
    for (const auto& r : itinerary)
    {
      print_route(r, start_time);
      std::cout  << " | ";
    }

    std::cout << "(end)" << std::endl;
  }
}

//==============================================================================
int main()
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

#define SHOW(X) std::cout << #X << ": " << X * 180.0/M_PI << std::endl
  SHOW(forward_incline_up);
  SHOW(forward_incline_down);
  SHOW(backward_incline_up);
  SHOW(backward_incline_down);

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

  const auto keys = supergraph->keys_for(0, goal_index, std::nullopt);

  std::cout << "Number of keys: " << keys.size() << std::endl;
  for (const auto& key : keys)
  {
//    std::cout << " ==================== " << std::endl;
    auto solution = diff_drive_cache->get().get(key);
//    std::cout << " --------------------\n"
    std::cout
              << "Solution for [" << key.start_lane << ", "
              << key.start_orientation << ", "
              << key.start_side << ", "
              << key.goal_lane << ", "
              << key.goal_orientation << "]:\n";

    rmf_traffic::Time time{rmf_traffic::Duration(0)};
    double yaw = 0.0;
    std::vector<rmf_traffic::Route> routes;
    while (solution)
    {
      if (solution->route_factory)
      {
        auto new_route_info = solution->route_factory(time, yaw);
        routes.insert(
              routes.end(),
              new_route_info.routes.begin(),
              new_route_info.routes.end());
        time = new_route_info.finish_time;
        yaw = new_route_info.finish_yaw;
      }

      solution = solution->child;
    }

    print_itinerary(routes);
    std::cout << "\n" << std::endl;
  }

//  const std::size_t second_goal_index = 45;
//  const auto second_keys = supergraph->keys_for(0, second_goal_index, std::nullopt);
//  std::cout << "Number of keys: " << second_keys.size() << std::endl;
//  for (const auto& key : second_keys)
//  {
//    std::cout << " ==================== " << std::endl;
//    auto solution = diff_drive_cache->get().get(key);
//    std::cout << " --------------------\n"
//              << "Solution for [" << key.start_lane << ", "
//              << key.start_orientation << ", "
//              << key.goal_lane << ", "
//              << key.goal_orientation << "]:";

//    double cost = 0.0;
//    while (solution)
//    {
//      cost += solution->info.cost_from_parent;
//      std::cout << " (" << cost << "; "
//                << solution->info.waypoint << ", "
//                << 180.0*solution->info.yaw.value_or(std::nan(""))/M_PI
//                << ") -->";
//      solution = solution->child;
//    }
//    std::cout << " (finished)" << std::endl;

//    const auto& lane = supergraph->original().lanes[key.goal_lane];
//    std::cout << "Lane " << key.goal_lane << ": " << lane.entry().waypoint_index()
//              << " -> " << lane.exit().waypoint_index() << std::endl;
//  }
}
