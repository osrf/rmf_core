/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "internal_Planner.hpp"
#include "internal_planning.hpp"

namespace rmf_traffic {
namespace agv {

//==============================================================================
// This line tells the linker to take care of defining the value of this field
// inside of this translation unit.
const Duration Planner::Options::DefaultMinHoldingTime;

//==============================================================================
class Planner::Configuration::Implementation
{
public:

  Graph graph;
  VehicleTraits traits;
  Interpolate::Options interpolation;

};

//==============================================================================
Planner::Configuration::Configuration(
    Graph graph,
    VehicleTraits traits,
    Interpolate::Options interpolation)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(graph),
               std::move(traits),
               std::move(interpolation)
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Configuration::graph(Graph graph) -> Configuration&
{
  _pimpl->graph = std::move(graph);
  return *this;
}

//==============================================================================
Graph& Planner::Configuration::graph()
{
  return _pimpl->graph;
}

//==============================================================================
const Graph& Planner::Configuration::graph() const
{
  return _pimpl->graph;
}

//==============================================================================
auto Planner::Configuration::vehicle_traits(VehicleTraits traits)
-> Configuration&
{
  _pimpl->traits = std::move(traits);
  return *this;
}

//==============================================================================
VehicleTraits& Planner::Configuration::vehicle_traits()
{
  return _pimpl->traits;
}

//==============================================================================
const VehicleTraits& Planner::Configuration::vehicle_traits() const
{
  return _pimpl->traits;
}

//==============================================================================
auto Planner::Configuration::interpolation(Interpolate::Options interpolate)
-> Configuration&
{
  _pimpl->interpolation = std::move(interpolate);
  return *this;
}

//==============================================================================
Interpolate::Options& Planner::Configuration::interpolation()
{
  return _pimpl->interpolation;
}

//==============================================================================
const Interpolate::Options& Planner::Configuration::interpolation() const
{
  return _pimpl->interpolation;
}

//==============================================================================
class Planner::Options::Implementation
{
public:

  rmf_utils::clone_ptr<RouteValidator> validator;
  Duration min_hold_time;
  const bool* interrupt_flag;

};

//==============================================================================
Planner::Options::Options(
    rmf_utils::clone_ptr<RouteValidator> validator,
    const Duration min_hold_time,
    const bool* interrupt_flag)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(validator),
               min_hold_time,
               interrupt_flag
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Options::validator(rmf_utils::clone_ptr<RouteValidator> v)
-> Options&
{
  _pimpl->validator = std::move(v);
  return *this;
}

//==============================================================================
const rmf_utils::clone_ptr<RouteValidator>& Planner::Options::validator() const
{
  return _pimpl->validator;
}

//==============================================================================
auto Planner::Options::minimum_holding_time(const Duration holding_time)
-> Options&
{
  _pimpl->min_hold_time = holding_time;
  return *this;
}

//==============================================================================
Duration Planner::Options::minimum_holding_time() const
{
  return _pimpl->min_hold_time;
}

//==============================================================================
auto Planner::Options::interrupt_flag(const bool* flag) -> Options&
{
  _pimpl->interrupt_flag = flag;
  return *this;
}

//==============================================================================
const bool* Planner::Options::interrupt_flag() const
{
  return _pimpl->interrupt_flag;
}

//==============================================================================
class Planner::Start::Implementation
{
public:

  Time time;
  std::size_t waypoint;
  double orientation;
  rmf_utils::optional<Eigen::Vector2d> location;
  rmf_utils::optional<std::size_t> lane;

};

//==============================================================================
Planner::Start::Start(
    const Time initial_time,
    const std::size_t initial_waypoint,
    const double initial_orientation,
    rmf_utils::optional<Eigen::Vector2d> initial_location,
    rmf_utils::optional<std::size_t> initial_lane)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               initial_time,
               initial_waypoint,
               initial_orientation,
               std::move(initial_location),
               std::move(initial_lane)
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Start::time(const Time initial_time) -> Start&
{
  _pimpl->time = initial_time;
  return *this;
}

//==============================================================================
Time Planner::Start::time() const
{
  return _pimpl->time;
}

//==============================================================================
auto Planner::Start::waypoint(const std::size_t initial_waypoint) -> Start&
{
  _pimpl->waypoint = initial_waypoint;
  return *this;
}

//==============================================================================
std::size_t Planner::Start::waypoint() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Planner::Start::orientation(const double initial_orientation) -> Start&
{
  _pimpl->orientation = initial_orientation;
  return *this;
}

//==============================================================================
double Planner::Start::orientation() const
{
  return _pimpl->orientation;
}

//==============================================================================
const rmf_utils::optional<Eigen::Vector2d>& Planner::Start::location() const
{
  return _pimpl->location;
}

//==============================================================================
auto Planner::Start::location(
    rmf_utils::optional<Eigen::Vector2d> initial_location) -> Start&
{
  _pimpl->location = std::move(initial_location);
  return *this;
}

//==============================================================================
const rmf_utils::optional<std::size_t>& Planner::Start::lane() const
{
  return _pimpl->lane;
}

//==============================================================================
auto Planner::Start::lane(
    rmf_utils::optional<std::size_t> initial_lane) -> Start&
{
  _pimpl->lane = initial_lane;
  return *this;
}

//==============================================================================
class Planner::Goal::Implementation
{
public:

  std::size_t waypoint;

  rmf_utils::optional<double> orientation;

};

//==============================================================================
Planner::Goal::Goal(const std::size_t waypoint)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               waypoint,
               rmf_utils::nullopt
             }))
{
  // Do nothing
}

//==============================================================================
Planner::Goal::Goal(
    const std::size_t waypoint,
    const double goal_orientation)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               waypoint,
               goal_orientation
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Goal::waypoint(const std::size_t goal_waypoint) -> Goal&
{
  _pimpl->waypoint = goal_waypoint;
  return *this;
}

//==============================================================================
std::size_t Planner::Goal::waypoint() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Planner::Goal::orientation(const double goal_orientation) -> Goal&
{
  _pimpl->orientation = goal_orientation;
  return *this;
}

//==============================================================================
auto Planner::Goal::any_orientation() -> Goal&
{
  _pimpl->orientation = rmf_utils::nullopt;
  return *this;
}

//==============================================================================
const double* Planner::Goal::orientation() const
{
  if(_pimpl->orientation)
    return &(*_pimpl->orientation);

  return nullptr;
}

//==============================================================================
class Planner::Implementation
{
public:

  internal::planning::CacheManager cache_mgr;

  Options default_options;

  Configuration configuration;

};

//==============================================================================
class Plan::Implementation
{
public:

  internal::planning::Result result;

  internal::planning::CacheManager cache_mgr;


  static rmf_utils::optional<Plan> generate(
      internal::planning::CacheManager cache_mgr,
      const std::vector<Planner::Start>& starts,
      Planner::Goal goal,
      Planner::Options options)
  {
    auto result = cache_mgr.get().plan(
        {starts}, std::move(goal), std::move(options));

    if (!result)
      return rmf_utils::nullopt;

    Plan plan;
    plan._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{std::move(*result), std::move(cache_mgr)});

    return std::move(plan);
  }

};

//==============================================================================
Planner::Planner(
    Configuration config,
    Options default_options)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               internal::planning::make_cache(config),
               std::move(default_options),
               config
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::get_configuration() const -> const Configuration&
{
  return _pimpl->configuration;
}

//==============================================================================
Planner& Planner::set_default_options(Options default_options)
{
  _pimpl->default_options = std::move(default_options);
  return *this;
}

//==============================================================================
auto Planner::get_default_options() -> Options&
{
  return _pimpl->default_options;
}

//==============================================================================
auto Planner::get_default_options() const -> const Options&
{
  return _pimpl->default_options;
}

//==============================================================================
rmf_utils::optional<Plan> Planner::plan(const Start& start, Goal goal) const
{
  return Plan::Implementation::generate(
        _pimpl->cache_mgr,
        {start},
        std::move(goal),
        _pimpl->default_options);
}

//==============================================================================
rmf_utils::optional<Plan> Planner::plan(
    const Start& start,
    Goal goal,
    Options options) const
{
  return Plan::Implementation::generate(
        _pimpl->cache_mgr,
        {start},
        std::move(goal),
        std::move(options));
}

//==============================================================================
rmf_utils::optional<Plan> Planner::plan(const StartSet& starts, Goal goal) const
{
  return Plan::Implementation::generate(
        _pimpl->cache_mgr,
        starts,
        std::move(goal),
        _pimpl->default_options);
}

//==============================================================================
rmf_utils::optional<Plan> Planner::plan(
    const StartSet& starts,
    Goal goal,
    Options options) const
{
  return Plan::Implementation::generate(
        _pimpl->cache_mgr,
        starts,
        std::move(goal),
        std::move(options));
}

//==============================================================================
const Eigen::Vector3d& Plan::Waypoint::position() const
{
  return _pimpl->position;
}

//==============================================================================
rmf_traffic::Time Plan::Waypoint::time() const
{
  return _pimpl->time;
}

//==============================================================================
rmf_utils::optional<std::size_t> Plan::Waypoint::graph_index() const
{
  return _pimpl->graph_index;
}

//==============================================================================
const Graph::Lane::Event* Plan::Waypoint::event() const
{
  return _pimpl->event.get();
}

//==============================================================================
Plan::Waypoint::Waypoint()
{
  // Do nothing
}

//==============================================================================
const std::vector<Route>& Plan::get_routes() const
{
  return _pimpl->result.routes;
}

//==============================================================================
const std::vector<Plan::Waypoint>& Plan::get_waypoints() const
{
  return _pimpl->result.waypoints;
}

//==============================================================================
rmf_utils::optional<Plan> Plan::replan(const Start& new_start) const
{
  return Plan::Implementation::generate(
        _pimpl->cache_mgr,
        {new_start},
        _pimpl->result.goal,
        _pimpl->result.options);
}

//==============================================================================
rmf_utils::optional<Plan> Plan::replan(
    const Planner::Start& new_start,
    Planner::Options new_options) const
{
  return Plan::Implementation::generate(
        _pimpl->cache_mgr,
        {new_start},
        _pimpl->result.goal,
        std::move(new_options));
}

//==============================================================================
rmf_utils::optional<Plan> Plan::replan(const StartSet& new_starts) const
{
  return Plan::Implementation::generate(
        _pimpl->cache_mgr,
        new_starts,
        _pimpl->result.goal,
        _pimpl->result.options);
}

//==============================================================================
rmf_utils::optional<Plan> Plan::replan(
    const StartSet& new_starts,
    Options new_options) const
{
  return Plan::Implementation::generate(
        _pimpl->cache_mgr,
        new_starts,
        _pimpl->result.goal,
        std::move(new_options));
}

//==============================================================================
const Planner::Start& Plan::get_start() const
{
  return _pimpl->result.start;
}

//==============================================================================
const Planner::Goal& Plan::get_goal() const
{
  return _pimpl->result.goal;
}

//==============================================================================
const Planner::Options& Plan::get_options() const
{
  return _pimpl->result.options;
}

//==============================================================================
const Planner::Configuration& Plan::get_configuration() const
{
  return _pimpl->cache_mgr.get_configuration();
}

//==============================================================================

std::vector<Plan::Start> compute_plan_starts(
    const rmf_traffic::agv::Graph& graph,
    const Eigen::Vector3d pose,
    const rmf_traffic::Time start_time,
    const double max_merge_waypoint_distance,
    const double max_merge_lane_distance,
    const double min_lane_length)
{
  const Eigen::Vector2d p_location = {pose[0], pose[1]};
  const double start_yaw = pose[2];

  // If there are waypoints which are very close, take that as the only Start
  for (std::size_t i=0; i < graph.num_waypoints() ; ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    const Eigen::Vector2d wp_location = wp.get_location();

    if ( (p_location - wp_location).norm() < max_merge_waypoint_distance)
    {
      return {Plan::Start(start_time, wp.index(), start_yaw)};
    }
  }

  // Iterate through the lanes and return the set of possible waypoints, i.e.
  // entries and exits of nearby lanes.
  std::vector<Plan::Start> starts;
  std::unordered_set<std::size_t> raw_starts;

  for (std::size_t i=0; i < graph.num_lanes(); ++i)
  {
    const auto& lane = graph.get_lane(i);
    const Eigen::Vector2d p0 = 
        graph.get_waypoint(lane.entry().waypoint_index()).get_location();
    const Eigen::Vector2d p1 =
        graph.get_waypoint(lane.exit().waypoint_index()).get_location();
    
    const double lane_length = (p1 - p0).norm();

    // This "lane" is effectively a single point, so we'll skip it
    if (lane_length < min_lane_length)
      continue;

    const Eigen::Vector2d pn = (p1 - p0) / lane_length;
    const Eigen::Vector2d p_l = p_location - p0;
    const double p_l_projection = p_l.dot(pn);

    // If it's negative then its closest point on the lane is the entry point
    if (p_l_projection < 0.0)
    {
      const double dist_to_entry = p_l.norm();
      const std::size_t entry_waypoint_index = lane.entry().waypoint_index();

      if (dist_to_entry < max_merge_lane_distance)
      {
        if (!raw_starts.insert(entry_waypoint_index).second)
          continue;

        starts.emplace_back(
            Plan::Start(
                start_time, entry_waypoint_index, start_yaw, p_location));
      }
    }
    // If it's larger than the lane length, then its closest point on the lane
    // is the exit point.
    else if (lane_length < p_l_projection)
    {
      const double dist_to_exit = (p_location - p1).norm();
      const std::size_t exit_waypoint_index = lane.exit().waypoint_index();

      if (dist_to_exit < max_merge_lane_distance)
      {
        if (!raw_starts.insert(exit_waypoint_index).second)
          continue;

        starts.emplace_back(
            Plan::Start(
                start_time, exit_waypoint_index, start_yaw, p_location));
      }
    }
    // If its between the entry and the exit waypoints, then we should
    // compute it's distance away from the lane line.
    else
    {
      const double lane_dist = (p_l - p_l_projection*pn).norm();
      const std::size_t exit_waypoint_index = lane.exit().waypoint_index();

      if (lane_dist < max_merge_lane_distance)
      {
        starts.emplace_back(
            Plan::Start(
                start_time, exit_waypoint_index, start_yaw, p_location, i));
      }
    }
  }

  return starts;
}

} // namespace agv
} // namespace rmf_traffic
