
#include <src/rmf_traffic/agv/planning/DifferentialDriveHeuristic.hpp>

#include "test/unit/utils_Trajectory.hpp"


#include <iostream>


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

  graph.add_lane({0, bogus_event}, 1);
  graph.add_lane({1, bogus_event}, 2);
  graph.add_lane(2, 3);

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
    while (solution)
    {
      cost += solution->info.cost_from_parent;
      std::cout << " (" << cost << "; "
                << solution->info.waypoint << ", "
                << 180.0*solution->info.yaw.value_or(std::nan(""))/M_PI
                << ") -->";
      solution = solution->child;
    }
    std::cout << " (finished)" << std::endl;

    const auto& lane = supergraph->original().lanes[key.goal_lane];
    std::cout << "Lane " << key.goal_lane << ": " << lane.entry().waypoint_index()
              << " -> " << lane.exit().waypoint_index() << std::endl;
  }
}
