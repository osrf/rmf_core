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

#include "DifferentialDrivePlanner.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class ScheduledDifferentialDriveExpander
{
public:

  using Entry = DifferentialDriveMapTypes::Entry;

  struct SearchNode;
  using SearchNodePtr = std::shared_ptr<SearchNode>;
  using ConstSearchNodePtr = std::shared_ptr<const SearchNode>;

  struct SearchNode
  {
    // We use this Info structure to match the structure expected by the
    // DifferentialDriveCompare template.
    struct Info
    {
      std::optional<Entry> entry;

      std::optional<std::size_t> waypoint;
      Eigen::Vector2d position;
      double yaw;

      double remaining_cost_estimate;

      std::vector<Route> route_from_parent;

      // An event that should occur when this node is reached,
      // i.e. after route_from_parent has been traversed
      Graph::Lane::EventPtr event;
    };

    Info info;
    double current_cost;
    std::optional<Planner::Start> start;
    ConstSearchNodePtr parent;
  };

  using SearchQueue =
    std::priority_queue<
      SearchNodePtr,
      std::vector<SearchNodePtr>,
      DifferentialDriveCompare<SearchNodePtr>
    >;

  class InternalState : public State::Internal
  {
  public:

    std::optional<double> cost_estimate() const final
    {
      if (queue.empty())
        return std::nullopt;

      const auto& top = queue.top();
      return top->current_cost + top->info.remaining_cost_estimate;
    }

    std::size_t queue_size() const final
    {
      return queue.size();
    }

    SearchQueue queue;
  };


  ScheduledDifferentialDriveExpander(
    std::shared_ptr<const Supergraph> supergraph,
    DifferentialDriveHeuristicAdapter heuristic,
    const Planner::Goal& goal)
  : _supergraph(std::move(supergraph)),
    _heuristic(std::move(heuristic)),
    _goal_waypoint(goal.waypoint()),
    _goal_yaw(rmf_utils::pointer_to_opt(goal.orientation()))
  {
    // Do nothing
  }

  std::vector<Trajectory> make_start_approach_trajectories(
    const Planner::Start& start,
    const double hold_time)
  {
    const auto location_opt = start.location();
    if (!location_opt.has_value())
      return {};

    const auto* differential = _supergraph->traits().get_differential();
    DifferentialDriveConstraint constraint{
      differential->get_forward(),
      differential->is_reversible()
    };

    const std::size_t waypoint_index = start.waypoint();

    const Eigen::Vector2d p0 = *location_opt;
    const Eigen::Vector2d p1 =
        _supergraph->original().waypoints[waypoint_index].get_location();

    const double translation_thresh = _supergraph->options().translation_thresh;

    const double dist = (p1 - p0).norm();
    if (dist < translation_thresh)
      return {};

    const Eigen::Vector2d course_vector =
        (p1 - p0)/dist;
    const auto yaw_options = constraint.get_orientations(course_vector);

    const Graph::Lane* const lane = start.lane()?
          &_supergraph->original().lanes[*start.lane()] : nullptr;
    const Graph::OrientationConstraint* const entry_constraint =
        lane? lane->entry().orientation_constraint() : nullptr;
    const Graph::OrientationConstraint* const exit_constraint =
        lane? lane->exit().orientation_constraint() : nullptr;

    const double rotation_thresh = _supergraph->options().rotation_thresh;
    const auto start_time = start.time() + time::from_seconds(hold_time);
    const double start_yaw = start.orientation();
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

    const auto& traits = _supergraph->traits();
    const auto& linear = traits.linear();
    const auto& angular = traits.rotational();
    const double v_nom = linear.get_nominal_velocity();
    const double a_nom = linear.get_nominal_acceleration();
    const double w_nom = angular.get_nominal_velocity();
    const double alpha_nom = angular.get_nominal_acceleration();

    std::vector<Trajectory> trajectories;
    for (const auto& yaw_opt : yaw_options)
    {
      if (!yaw_opt.has_value())
        continue;

      const double yaw = *yaw_opt;

      if (!orientation_constraint_satisfied(
            p0, yaw, course_vector, entry_constraint, rotation_thresh))
        continue;

      if (!orientation_constraint_satisfied(
            p1, yaw, course_vector, exit_constraint, rotation_thresh))
        continue;

      Trajectory trajectory;
      const Eigen::Vector3d p_start = {p0.x(), p0.y(), start_yaw};
      trajectory.insert(start_time, p_start, zero);

      const Eigen::Vector3d p_oriented{p0.x(), p0.y(), yaw};
      internal::interpolate_rotation(
            trajectory, w_nom, alpha_nom, start_time, p_start, p_oriented,
            rotation_thresh);

      const Eigen::Vector3d p_arrived{p1.x(), p1.y(), yaw};
      internal::interpolate_translation(
            trajectory, v_nom, a_nom, *trajectory.finish_time(),
            p_oriented, p_arrived, translation_thresh);

      trajectories.emplace_back(std::move(trajectory));
    }

    return trajectories;
  }

  SearchNodePtr make_start_node(const Planner::Start& start)
  {
    const std::size_t initial_waypoint = start.waypoint();
    const auto initial_yaw = start.orientation();
    double remaining_cost_estimate = 0.0;
    std::optional<std::size_t> node_waypoint;

    const auto start_location = start.location();
    if (start_location.has_value())
    {
      const auto approaches = make_start_approach_trajectories(start, 0.0);
      std::optional<double> lowest_cost_estimate;
      for (const auto& approach : approaches)
      {
        const double yaw = approach.back().position()[2];
        double cost = time::to_seconds(approach.duration());
        const auto heuristic_cost_estimate =
            _heuristic.compute(initial_waypoint, yaw);

        if (!heuristic_cost_estimate.has_value())
          continue;

        cost += *heuristic_cost_estimate;
        if (!lowest_cost_estimate.has_value() || cost < *lowest_cost_estimate)
          lowest_cost_estimate = cost;
      }

      if (!lowest_cost_estimate.has_value())
      {
        // If this happens, the heuristic found that there is simply no path
        // from this start to the goal, so we return a nullptr.
        return nullptr;
      }

      remaining_cost_estimate = *lowest_cost_estimate;
    }
    else
    {
      node_waypoint = initial_waypoint;
      const auto heuristic_cost_estimate =
          _heuristic.compute(initial_waypoint, initial_yaw);

      if (!heuristic_cost_estimate.has_value())
      {
        // If this happens, the heuristic found that there is simply no path
        // from this start to the goal, so we return a nullptr.
        return nullptr;
      }

      remaining_cost_estimate = *heuristic_cost_estimate;
    }

    const Eigen::Vector2d waypoint_location =
        _supergraph->original().waypoints[initial_waypoint].get_location();

    return std::make_shared<SearchNode>(
          SearchNode{
            SearchNode::Info{
              std::nullopt,
              node_waypoint,
              start_location.value_or(waypoint_location),
              initial_yaw,
              remaining_cost_estimate,
              {},
              nullptr
            },
            0.0,
            start,
            nullptr
          });
  }

private:
  std::shared_ptr<const Supergraph> _supergraph;
  DifferentialDriveHeuristicAdapter _heuristic;
  std::size_t _goal_waypoint;
  std::optional<double> _goal_yaw;
};

//==============================================================================
DifferentialDrivePlanner::DifferentialDrivePlanner(
    Planner::Configuration config)
  : _config(std::move(config))
{
  _supergraph = Supergraph::make(
        Graph::Implementation::get(_config.graph()),
        _config.vehicle_traits(),
        _config.interpolation());

  _cache = DifferentialDriveHeuristic::make_manager(_supergraph);
}

//==============================================================================
State DifferentialDrivePlanner::initiate(
  const std::vector<Planner::Start>& starts,
  Planner::Goal goal,
  Planner::Options options) const
{
  using InternalState = ScheduledDifferentialDriveExpander::InternalState;

  State state{
    Conditions{
      starts,
      std::move(goal),
      std::move(options)
    },
    Issues{},
    std::nullopt,
    rmf_utils::make_derived_impl<State::Internal, InternalState>()
  };

  auto& internal = static_cast<InternalState&>(*state.internal);

  ScheduledDifferentialDriveExpander expander{
    _supergraph,
    DifferentialDriveHeuristicAdapter{
      _cache->get(),
      _supergraph,
      goal.waypoint(),
      rmf_utils::pointer_to_opt(goal.orientation())
    },
    goal
  };

  for (const auto& start : starts)
  {
    if (auto node = expander.make_start_node(start))
      internal.queue.push(node);
  }

  if (internal.queue.empty())
  {
    state.issues.disconnected = true;
  }
  else
  {
    const auto& top = internal.queue.top();
    state.ideal_cost = top->current_cost + top->info.remaining_cost_estimate;
  }

  return state;
}

//==============================================================================
std::optional<Plan> DifferentialDrivePlanner::plan(State& state) const
{

}

//==============================================================================
std::vector<schedule::Itinerary> DifferentialDrivePlanner::rollout(
  const Duration span,
  const Issues::BlockedNodes& nodes,
  const Planner::Goal& goal,
  const Planner::Options& options,
  std::optional<std::size_t> max_rollouts) const
{

}

//==============================================================================
const Planner::Configuration&
DifferentialDrivePlanner::get_configuration() const
{
  return _config;
}

//==============================================================================
auto DifferentialDrivePlanner::debug_begin(
  const std::vector<Planner::Start>& starts,
  Planner::Goal goal,
  Planner::Options options) const -> std::unique_ptr<Debugger>
{
  throw std::runtime_error(
    "[rmf_traffic::agv::planning::DifferentialDrivePlanner::debug_begin] "
    "Debugging is not yet implemented for this planner.");
}

//==============================================================================
std::optional<Plan> DifferentialDrivePlanner::debug_step(
  Debugger& debugger) const
{
  throw std::runtime_error(
    "[rmf_traffic::agv::planning::DifferentialDrivePlanner::debug_step] "
    "Debugging is not yet implemented for this planner.");
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
