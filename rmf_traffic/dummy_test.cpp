
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


int main()
{
  rmf_traffic::agv::Graph graph;
  const std::string test_map_0 = "test_map_0";
  const std::string test_map_1 = "test_map_1";
  const std::string test_map_2 = "test_map_2";

  graph.add_waypoint(test_map_0, {0, 0}); // 0
  graph.add_waypoint(test_map_1, {0, 0}); // 1
  graph.add_waypoint(test_map_2, {0, 0}); // 2
  graph.add_waypoint(test_map_2, {0, 1}); // 3

  const auto bogus_event = rmf_traffic::agv::Graph::Lane::Event::make(
        rmf_traffic::agv::Graph::Lane::Wait(std::chrono::seconds(1)));

  graph.add_lane({0, bogus_event}, 1); // 0
  graph.add_lane({1, bogus_event}, 2); // 1
  graph.add_lane(2, 3); // 2

  const std::size_t goal_index = 3;

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
    std::cout << " ==================== " << std::endl;
    auto solution = diff_drive_cache->get().get(key);
    std::cout << " --------------------\n"
              << "Solution for [" << key.start_lane << ", "
              << key.start_orientation << ", "
              << key.goal_lane << ", "
              << key.goal_orientation << "]:";

    if (!solution)
    {
      std::cout << " No solution found!" << std::endl;
    }

    double cost = 0.0;
    std::vector<rmf_traffic::Route> routes;
    rmf_traffic::Time time(rmf_traffic::Duration(0));
    double yaw = 0.0;
    while (solution)
    {
      cost += solution->info.cost_from_parent;
      std::cout << " (" << cost << "; "
                << solution->info.waypoint << "; "
                << solution->info.position.transpose() << ", "
                << 180.0*solution->info.yaw.value_or(std::nan(""))/M_PI
                << ") -->";

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
      else
      {
        std::cout << "[No Route Factory]" << std::endl;
      }

      solution = solution->child;
    }
    std::cout << " (finished)" << std::endl;

    std::cout << "Itinerary:" << std::endl;
    print_itinerary(routes);

    const auto& lane = supergraph->original().lanes[key.goal_lane];
    std::cout << "Lane " << key.goal_lane << ": " << lane.entry().waypoint_index()
              << " -> " << lane.exit().waypoint_index() << std::endl;
  }
}
