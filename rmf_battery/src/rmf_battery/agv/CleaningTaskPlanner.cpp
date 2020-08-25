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

#include <rmf_battery/agv/CleaningTaskPlanner.hpp>
#include <rmf_battery/agv/SimpleBatteryEstimator.hpp>

#include <rmf_traffic/Route.hpp>

#include <iostream>

namespace rmf_battery {
namespace agv {

class CleaningTaskPlanner::Implementation
{
public:
  SystemTraits system_traits;
  Planner planner;
  std::string cleaning_system;
  double battery_threshold;
};

//==============================================================================
CleaningTaskPlanner::CleaningTaskPlanner(
  SystemTraits& system_traits,
  Planner& planner,
  const std::string& cleaning_system,
  const double battery_threshold)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        system_traits,
        std::move(planner),
        std::move(cleaning_system),
        battery_threshold
      }))
{
  assert(battery_threshold >= 0.0 && battery_threshold <= 1.0);
}

//==============================================================================
const SystemTraits& CleaningTaskPlanner::system_traits() const
{
  return _pimpl->system_traits;
}

//==============================================================================
const rmf_traffic::agv::Planner& CleaningTaskPlanner::planner() const
{
  return _pimpl->planner;
}

//==============================================================================
const std::string& CleaningTaskPlanner::cleaning_system() const
{
  return _pimpl->cleaning_system;
}

//==============================================================================
double CleaningTaskPlanner::battery_threshold() const
{
  return _pimpl->battery_threshold;
}
//==============================================================================
namespace {
double get_battery_soc_after_itinerary(
  const SimpleBatteryEstimator& battery_estimator,
  const std::vector<rmf_traffic::Route>& itinerary,
  const SystemTraits& system_traits,
  const std::string& cleaning_system,
  const double initial_soc)
{
  double battery_soc = initial_soc;
  for (const auto& route : itinerary)
  {
    // TODO(YV): allow users to state which power systems are active during non-
    // cleaning routes
    rmf_utils::optional<SimpleBatteryEstimator::PowerMap> power_map =
      SimpleBatteryEstimator::PowerMap();
    for (const auto power_system : system_traits.power_systems())
    {
      if (power_system.first == cleaning_system)
        continue;
      
      power_map.value().insert({power_system.first, route.trajectory()});
    }
    battery_soc = battery_estimator.compute_state_of_charge(
      route.trajectory(), battery_soc, power_map);
  }
  
  return battery_soc;
}

} // anonymous

//==============================================================================
std::vector<rmf_traffic::Trajectory> CleaningTaskPlanner::plan(
  const Planner::Start& start,
  const double initial_soc,
  const std::size_t cleaning_start_waypoint,
  const rmf_traffic::Trajectory& cleaning_trajectory,
  const std::size_t cleaning_end_waypoint,
  const std::size_t charging_station_waypoint)
{
  // Ensure that the waypoint indices are valid
  const auto& graph = _pimpl->planner.get_configuration().graph();
  std::size_t num_waypoints = graph.num_waypoints();
  assert(start.waypoint() < num_waypoints);
  assert(cleaning_start_waypoint < num_waypoints);
  assert(cleaning_end_waypoint < num_waypoints);
  assert(charging_station_waypoint < num_waypoints);
  assert(_pimpl->system_traits.power_systems().find(_pimpl->cleaning_system)
    != _pimpl->system_traits.power_systems().end());

  std::vector<rmf_traffic::Trajectory> cleaning_plan;
  double battery_soc = initial_soc;
  SimpleBatteryEstimator battery_estimator(_pimpl->system_traits);

  Planner::Goal goal_1{cleaning_start_waypoint};
  const auto plan_to_cleaning_start = _pimpl->planner.plan(start, goal_1);
  // Stage 1: check if robot has charge to reach cleaning start waypoint
  if (!plan_to_cleaning_start.success())
  { 
    std::cout << "Failed to find plan from start to cleaning start waypoint"
              << std::endl;
    return cleaning_plan;
  }
  auto itinerary_1 = plan_to_cleaning_start->get_itinerary();
  // Compute battery drain for each route
  battery_soc = get_battery_soc_after_itinerary(battery_estimator,
    itinerary_1, _pimpl->system_traits, _pimpl->cleaning_system, battery_soc);

  std::cout << "Battery soc after traversing to cleaning start: " 
            << battery_soc << std::endl;
  if (battery_soc < _pimpl->battery_threshold)
    return cleaning_plan;

  // Stage 2: check if robot has charge to finish cleaning
  rmf_utils::optional<SimpleBatteryEstimator::PowerMap> power_map =
    SimpleBatteryEstimator::PowerMap();
  for (const auto& system : _pimpl->system_traits.power_systems())
    power_map.value().insert({system.first, cleaning_trajectory});
  battery_soc = battery_estimator.compute_state_of_charge(
    cleaning_trajectory, battery_soc, power_map);

  std::cout << "Battery soc after cleaning: " << battery_soc
            << std::endl;

  if (battery_soc < _pimpl->battery_threshold)
    return cleaning_plan;

  // Stage 3: check if robot has charge to theoretically travel to its charger
  auto end_it = cleaning_trajectory.end(); --end_it;
  Planner::Start start_2{end_it->time(),
    cleaning_end_waypoint, end_it->position()[2]};
  Planner::Goal goal_2{charging_station_waypoint};
  const auto plan_to_charger = _pimpl->planner.plan(start_2, goal_2);

  if (!plan_to_charger.success())
  {
    std::cout << "Failed to find plan from cleaning end to charger"
              << std::endl;
    return cleaning_plan;
  }

  auto itinerary_2 = plan_to_charger->get_itinerary();
  battery_soc = get_battery_soc_after_itinerary(battery_estimator,
    itinerary_2, _pimpl->system_traits, _pimpl->cleaning_system, battery_soc);

  std::cout << "Battery after returning to charger: " << battery_soc 
            <<std::endl;

  if (battery_soc < _pimpl->battery_threshold)
    return cleaning_plan;

  // A successful plan can be produced
  for (const auto& route : itinerary_1)
    cleaning_plan.push_back(route.trajectory());
  cleaning_plan.push_back(cleaning_trajectory);

  return cleaning_plan;
}

} // namespace agv
} // namespace rmf_battery