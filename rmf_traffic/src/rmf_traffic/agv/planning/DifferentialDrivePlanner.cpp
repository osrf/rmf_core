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

#include <rmf_utils/math.hpp>

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
    // We use optional here because start nodes don't always have a waypoint.
    // If this is a nullopt, then SearchNode::start should have a value.
    std::optional<std::size_t> waypoint;
    Eigen::Vector2d position;
    double yaw;
    rmf_traffic::Time time;
    std::optional<Orientation> orientation;

    double remaining_cost_estimate;

    std::vector<Route> route_from_parent;

    // An event that should occur when this node is reached,
    // i.e. after route_from_parent has been traversed
    Graph::Lane::EventPtr event;

    double current_cost;
    std::optional<Planner::Start> start;
    ConstSearchNodePtr parent;

    double get_total_cost_estimate() const
    {
      return current_cost + remaining_cost_estimate;
    }

    double get_remaining_cost_estimate() const
    {
      return remaining_cost_estimate;
    }

    std::optional<Orientation> get_orientation() const
    {
      return orientation;
    }
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
      return top->current_cost + top->remaining_cost_estimate;
    }

    std::size_t queue_size() const final
    {
      return queue.size();
    }

    SearchQueue queue;
    std::size_t popped_count = 0;
  };

  bool quit(const SearchNodePtr& top, SearchQueue& queue) const
  {
    auto& internal = static_cast<InternalState&>(*_state->internal);
    ++internal.popped_count;

    if (_saturation_limit.has_value())
    {
      if (*_saturation_limit < internal.popped_count + queue.size())
        return true;
    }

    if (_maximum_cost_estimate.has_value())
    {
      const double cost_estimate = top->get_total_cost_estimate();

      if (*_maximum_cost_estimate < cost_estimate)
        return true;
    }

    return false;
  }

  bool is_finished(const SearchNodePtr& top) const
  {
    if (top->waypoint == _goal_waypoint)
    {
      if (!_goal_yaw.has_value())
        return true;

      const double angle_diff = rmf_utils::wrap_to_pi(top->yaw - *_goal_yaw);
      if (std::abs(angle_diff) <= _rotation_threshold)
        return true;
    }

    return false;
  }

  void expand_start(const SearchNodePtr& top, SearchQueue& queue)
  {
    const auto& start = top->start.value();
    const std::size_t target_waypoint_index = start.waypoint();
    const auto& wp = _supergraph->original().waypoints[target_waypoint_index];
    const Eigen::Vector2d wp_location = wp.get_location();

    // If this start node did not have a waypoint, then it must have a location
    assert(start.location().has_value());

    const auto approach_trajectories = make_start_approach_trajectories(
          top->start.value(), top->current_cost);

    if (approach_trajectories.empty())
    {
      // This means there are no valid ways to approach the start. We should
      // just give up on expanding from this node.
      return;
    }

    std::vector<std::string> map_names;
    Graph::Lane::EventPtr exit_event;
    double exit_event_cost = 0.0;
    if (const auto lane_index = start.lane())
    {
      const auto& lane = _supergraph->original().lanes[*lane_index];

      const std::size_t wp0_index = lane.entry().waypoint_index();
      const auto& wp0 = _supergraph->original().waypoints[wp0_index];
      const auto& wp0_map = wp0.get_map_name();

      assert(lane.exit().waypoint_index() == target_waypoint_index);
      const auto& wp1_map = wp.get_map_name();

      map_names.push_back(wp0_map);
      if (wp0_map != wp1_map)
        map_names.push_back(wp1_map);

      if (lane.exit().event())
      {
        exit_event = lane.exit().event()->clone();
        exit_event_cost = time::to_seconds(exit_event->duration());
      }
    }
    else
    {
      map_names.push_back(wp.get_map_name());
    }

    for (const auto& approach : approach_trajectories)
    {
      std::vector<Route> approach_routes;
      if (approach.size() >= 2)
      {
        bool all_valid = true;

        for (const auto& map : map_names)
        {
          Route route{map, approach};
          if (!is_valid(top, route))
          {
            all_valid = false;
            break;
          }

          approach_routes.emplace_back(std::move(route));
        }

        if (!all_valid)
          continue;
      }

      std::vector<Route> exit_event_routes;
      if (exit_event)
      {
        Trajectory hold;
        const auto& approached = approach.back();
        hold.insert(approached);
        hold.insert(approached.time(), approached.position(), {0, 0, 0});

        bool all_valid = true;
        for (const auto& map : map_names)
        {
          Route route{map, hold};
          if (!is_valid(top, route))
          {
            all_valid = false;
            break;
          }

          exit_event_routes.emplace_back(std::move(route));
        }

        if (!all_valid)
          continue;
      }

      const double approach_cost = time::to_seconds(approach.duration());
      const double approach_yaw = approach.back().position()[2];
      const auto approach_time = *approach.finish_time();

      // TODO(MXG): We can actually specify the orientation for this. We just
      // need to be smarter with make_start_approach_trajectories(). We should
      // really have it return a Traversal.
      auto node = std::make_shared<SearchNode>(
        SearchNode{
          target_waypoint_index,
          wp_location,
          approach_yaw,
          approach_time,
          std::nullopt,
          top->remaining_cost_estimate - approach_cost,
          std::move(approach_routes),
          exit_event,
          top->current_cost + approach_cost,
          std::nullopt,
          top
        });

      if (exit_event)
      {
        node = std::make_shared<SearchNode>(
          SearchNode{
            target_waypoint_index,
            wp_location,
            approach_yaw,
            approach_time,
            std::nullopt,
            node->remaining_cost_estimate - exit_event_cost,
            std::move(exit_event_routes),
            nullptr,
            node->current_cost + exit_event_cost,
            std::nullopt,
            node
          });
      }

      queue.push(node);
    }

    const Time initial_time = top->time;
    const Time hold_until = initial_time + _holding_time;
    const double hold_cost = time::to_seconds(_holding_time);
    const Eigen::Vector2d p = top->position;
    const double yaw = top->yaw;
    const Eigen::Vector3d position{p.x(), p.y(), yaw};
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    Trajectory hold;
    hold.insert(initial_time, position, zero);
    hold.insert(hold_until, position, zero);

    std::vector<Route> hold_routes;
    for (const auto& map : map_names)
    {
      Route route{map, hold};
      if (!is_valid(top, route))
        return;

      hold_routes.emplace_back(std::move(route));
    }

    queue.push(
    std::make_shared<SearchNode>(
          SearchNode{
            std::nullopt,
            p,
            yaw,
            hold_until,
            std::nullopt,
            top->remaining_cost_estimate,
            std::move(hold_routes),
            nullptr,
            top->current_cost + hold_cost,
            start,
            top
          }));
  }

  void expand_hold(const SearchNodePtr& top, SearchQueue& queue)
  {
    const std::size_t wp_index = top->waypoint.value();
    const std::string& map_name =
        _supergraph->original().waypoints[wp_index].get_map_name();

    const Eigen::Vector2d p = top->position;
    const double yaw = top->yaw;
    const Eigen::Vector3d position{p.x(), p.y(), yaw};
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    const auto start_time = top->time;
    const auto finish_time = start_time + _holding_time;
    const auto cost = time::to_seconds(_holding_time);

    Trajectory trajectory;
    trajectory.insert(start_time, position, zero);
    trajectory.insert(finish_time, position, zero);

    Route route{map_name, std::move(trajectory)};

    if (!is_valid(top, route))
      return;

    queue.push(std::make_shared<SearchNode>(
       SearchNode{
         wp_index,
         p,
         yaw,
         finish_time,
         top->orientation,
         top->remaining_cost_estimate,
         {std::move(route)},
         nullptr,
         cost,
         std::nullopt,
         top
       }));
  }

  SearchNodePtr rotate_to_goal(const SearchNodePtr& top)
  {
    assert(top->waypoint == _goal_waypoint);
    const std::string& map_name =
        _supergraph->original().waypoints[_goal_waypoint].get_map_name();

    const Eigen::Vector2d p = top->position;
    const double target_yaw = _goal_yaw.value();
    const Eigen::Vector3d start_position{p.x(), p.y(), top->yaw};
    const auto start_time = top->time;

    const Eigen::Vector3d finish_position{p.x(), p.y(), target_yaw};

    Trajectory trajectory;
    trajectory.insert(
          start_time, start_position, Eigen::Vector3d::Zero());
    internal::interpolate_rotation(
          trajectory, _w_nom, _alpha_nom, start_time,
          start_position, finish_position, _rotation_threshold);

    assert(trajectory.size() >= 2);

    const auto finish_time = *trajectory.finish_time();
    const double cost = time::to_seconds(trajectory.duration());
    Route route{map_name, std::move(trajectory)};
    if (!is_valid(top, route))
      return nullptr;

    return std::make_shared<SearchNode>(
      SearchNode{
        _goal_waypoint,
        p,
        target_yaw,
        finish_time,
        std::nullopt,
        0.0,
        {std::move(route)},
        nullptr,
        top->current_cost + cost,
        std::nullopt,
        top
      });
  }

  bool is_valid(const SearchNodePtr& parent, const Route& route)
  {
    if (route.trajectory().size() >= 2)
    {
      auto conflict = _validator->find_conflict(route);
      if (conflict)
      {
        auto time_it =
            _state->issues.blocked_nodes[conflict->participant]
            .insert({parent, conflict->time});

        if (!time_it.second)
        {
          time_it.first->second =
            std::max(time_it.first->second, conflict->time);
        }

        return false;
      }
    }

    return true;
  }

  void expand_traversal(
      const SearchNodePtr& top,
      const Traversal& traversal,
      SearchQueue& queue)
  {
    const auto initial_waypoint_index = top->waypoint.value();
    const auto& initial_waypoint =
        _supergraph->original().waypoints[initial_waypoint_index];
    const Eigen::Vector2d p0 = initial_waypoint.get_location();
    const double initial_yaw = top->yaw;
    const std::string& initial_map_name = initial_waypoint.get_map_name();

    const auto next_waypoint_index = traversal.finish_waypoint_index;
    const auto& next_waypoint =
        _supergraph->original().waypoints[next_waypoint_index];
    const Eigen::Vector2d next_position = next_waypoint.get_location();
    const std::string& next_map_name = next_waypoint.get_map_name();

    for (std::size_t i = 0; i < traversal.alternatives.size(); ++i)
    {
      const auto& alt = traversal.alternatives[i];
      if (!alt.has_value())
        continue;

      const Orientation orientation = Orientation(i);

      Time start_time = top->time;
      const auto traversal_yaw = alt->yaw;

      Trajectory approach_trajectory;
      const Eigen::Vector3d start{p0.x(), p0.y(), initial_yaw};
      approach_trajectory.insert(
            start_time, start, Eigen::Vector3d::Zero());

      // TODO(MXG): We could push the logic for creating this trajectory
      // upstream into the traversal alternative.
      if (traversal_yaw.has_value())
      {
        const Eigen::Vector3d finish{p0.x(), p0.y(), *traversal_yaw};
        internal::interpolate_rotation(
              approach_trajectory, _w_nom, _alpha_nom, start_time,
              start, finish, _rotation_threshold);
      }

      auto approach_route =
        Route{
          initial_map_name,
          std::move(approach_trajectory)
        };

      if (!is_valid(top, approach_route))
        continue;

      Trajectory entry_event_trajectory;
      const auto& approach_wp = approach_route.trajectory().back();
      entry_event_trajectory.insert(approach_wp);
      double entry_event_cost = 0.0;
      if (traversal.entry_event
          && traversal.entry_event->duration() > Duration(0))
      {
        const auto duration = traversal.entry_event->duration();
        entry_event_cost = time::to_seconds(duration);

        entry_event_trajectory.insert(
          approach_wp.time() + duration,
          approach_wp.position(), Eigen::Vector3d::Zero());
      }

      auto entry_event_route =
        Route{
          initial_map_name,
          std::move(entry_event_trajectory)
        };

      if (!is_valid(top, entry_event_route))
        continue;

      const auto& ready_wp = entry_event_route.trajectory().back();
      const auto ready_time = ready_wp.time();
      const double ready_yaw = ready_wp.position()[2];
      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);

      bool all_valid = true;
      for (const auto& r : traversal_result.routes)
      {
        if (!is_valid(top, r))
        {
          all_valid = false;
          break;
        }
      }

      if (!all_valid)
        continue;

      const auto remaining_cost_estimate = _heuristic.compute(
            next_waypoint_index, traversal_result.finish_yaw);

      if (!remaining_cost_estimate.has_value())
        continue;

      const auto& arrival_wp =
          traversal_result.routes.back().trajectory().back();

      Trajectory exit_event_trajectory;
      exit_event_trajectory.insert(arrival_wp);
      double exit_event_cost = 0.0;
      if (traversal.exit_event
          && traversal.exit_event->duration() > Duration(0))
      {
        const auto duration = traversal.exit_event->duration();
        exit_event_cost = time::to_seconds(duration);

        exit_event_trajectory.insert(
              arrival_wp.time() + duration,
              arrival_wp.position(), Eigen::Vector3d::Zero());
      }

      auto exit_event_route =
        Route{
          next_map_name,
          std::move(exit_event_trajectory)
        };

      if (!is_valid(top, exit_event_route))
        continue;

      auto node = top;
      if (approach_route.trajectory().size() >= 2)
      {
        const double cost =
            time::to_seconds(approach_route.trajectory().duration());
        const double yaw = approach_wp.position()[2];
        const auto time = approach_wp.time();

        node = std::make_shared<SearchNode>(
          SearchNode{
            initial_waypoint_index,
            p0,
            yaw,
            time,
            orientation,
            *remaining_cost_estimate
              + alt->time + entry_event_cost + exit_event_cost,
            {std::move(approach_route)},
            traversal.entry_event,
            node->current_cost + cost,
            std::nullopt,
            node
          });
      }

      if (entry_event_route.trajectory().size() >= 2)
      {
        node = std::make_shared<SearchNode>(
          SearchNode{
            initial_waypoint_index,
            p0,
            ready_yaw,
            ready_time,
            orientation,
            *remaining_cost_estimate + alt->time + exit_event_cost,
            {std::move(entry_event_route)},
            nullptr,
            node->current_cost + entry_event_cost,
            std::nullopt,
            node
          });
      }

      node = std::make_shared<SearchNode>(
        SearchNode{
          next_waypoint_index,
          next_position,
          traversal_result.finish_yaw,
          traversal_result.finish_time,
          orientation,
          *remaining_cost_estimate + exit_event_cost,
          std::move(traversal_result.routes),
          traversal.exit_event,
          node->current_cost + alt->time,
          std::nullopt,
          node
        });

      if (traversal.exit_event)
      {
        node = std::make_shared<SearchNode>(
          SearchNode{
            next_waypoint_index,
            next_position,
            traversal_result.finish_yaw,
            traversal_result.finish_time,
            orientation,
            *remaining_cost_estimate,
            {std::move(exit_event_route)},
            nullptr,
            node->current_cost + exit_event_cost,
            std::nullopt,
            node
          });
      }

      queue.push(node);
    }
  }

  void expand_freely(
      const SearchNodePtr& top,
      SearchQueue& queue)
  {
    // This function is used when there is no validator. We can just expand
    // freely to the goal without validating the results.
    const auto keys = _supergraph->keys_for(
      top->waypoint.value(), _goal_waypoint, _goal_yaw);

    for (const auto& key : keys)
    {
      const auto solution_root = _heuristic.cache().get(key);
      if (!solution_root)
        continue;

      auto solution_node = solution_root;
      auto search_node = top;
      while (solution_node)
      {
        auto route_from_parent = solution_node->route_factory(
              search_node->time, search_node->yaw);

        const auto entry = solution_node->info.entry;
        const auto orientation = entry.has_value()?
              std::make_optional(entry->orientation) : std::nullopt;

        search_node = std::make_shared<SearchNode>(
          SearchNode{
            solution_node->info.waypoint,
            solution_node->info.position,
            route_from_parent.finish_yaw,
            route_from_parent.finish_time,
            orientation,
            solution_node->info.remaining_cost_estimate,
            std::move(route_from_parent.routes),
            solution_node->info.event,
            search_node->current_cost + solution_node->info.cost_from_parent,
            std::nullopt,
            search_node
          });

        solution_node = solution_node->child;
      }

      queue.push(search_node);
    }
  }

  void expand(const SearchNodePtr& top, SearchQueue& queue)
  {
    if (!top->waypoint.has_value())
    {
      // If the node does not have a waypoint, then it must be a start node.
      assert(top->start.has_value());
      expand_start(top, queue);
      return;
    }

    expand_hold(top, queue);

    const auto current_wp_index = top->waypoint.value();
    if (current_wp_index == _goal_waypoint)
    {
      // If there is no goal yaw, then is_finished should have caught this node
      assert(_goal_yaw.has_value());

      if (auto node = rotate_to_goal(top))
        queue.push(std::move(node));

      return;
    }

    if (!_validator)
    {
      // If we don't have a validator, then we can jump straight to the solution
      expand_freely(top, queue);
      return;
    }

    const auto traversals = _supergraph->traversals_from(current_wp_index);
    for (const auto& traversal : *traversals)
      expand_traversal(top, traversal, queue);
  }

  std::vector<Trajectory> make_start_approach_trajectories(
    const Planner::Start& start,
    const double hold_time)
  {
    const auto location_opt = start.location();
    if (!location_opt.has_value())
      return {};

    // This will return all the different trajectories that can be used to
    // approach the start. If it returns empty, that means either you forgot to
    // check the location field, or there are no valid ways to approach the
    // start.

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

      if (const auto lane_index = start.lane())
      {
        const auto& lane = _supergraph->original().lanes[*lane_index];
        if (const auto* exit_event = lane.exit().event())
          remaining_cost_estimate += time::to_seconds(exit_event->duration());
      }
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
            node_waypoint,
            start_location.value_or(waypoint_location),
            initial_yaw,
            start.time(),
            std::nullopt,
            remaining_cost_estimate,
            {},
            nullptr,
            0.0,
            start,
            nullptr
          });
  }

  ScheduledDifferentialDriveExpander(
    State& state,
    std::shared_ptr<const Supergraph> supergraph,
    DifferentialDriveHeuristicAdapter heuristic,
    const Planner::Goal& goal,
    const Planner::Options& options)
  : _state(&state),
    _supergraph(std::move(supergraph)),
    _heuristic(std::move(heuristic)),
    _goal_waypoint(goal.waypoint()),
    _goal_yaw(rmf_utils::pointer_to_opt(goal.orientation())),
    _validator(options.validator().get()),
    _holding_time(options.minimum_holding_time()),
    _saturation_limit(options.saturation_limit()),
    _maximum_cost_estimate(options.maximum_cost_estimate())
  {
    const auto& angular = _supergraph->traits().rotational();
    _w_nom = angular.get_nominal_velocity();
    _alpha_nom = angular.get_nominal_acceleration();
    _rotation_threshold = _supergraph->options().rotation_thresh;
  }

private:
  State* _state;
  std::shared_ptr<const Supergraph> _supergraph;
  DifferentialDriveHeuristicAdapter _heuristic;
  std::size_t _goal_waypoint;
  std::optional<double> _goal_yaw;
  const RouteValidator* _validator;
  Duration _holding_time;
  std::optional<std::size_t> _saturation_limit;
  std::optional<double> _maximum_cost_estimate;
  double _w_nom;
  double _alpha_nom;
  double _rotation_threshold;
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
    state,
    _supergraph,
    DifferentialDriveHeuristicAdapter{
      _cache->get(),
      _supergraph,
      goal.waypoint(),
      rmf_utils::pointer_to_opt(goal.orientation())
    },
    state.conditions.goal,
    state.conditions.options
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
    state.ideal_cost = top->get_total_cost_estimate();
  }

  return state;
}

//==============================================================================
std::optional<Plan> DifferentialDrivePlanner::plan(State& state) const
{
  const auto& goal = state.conditions.goal;

  ScheduledDifferentialDriveExpander expander{
    state,
    _supergraph,
    DifferentialDriveHeuristicAdapter{
      _cache->get(),
      _supergraph,
      goal.waypoint(),
      rmf_utils::pointer_to_opt(goal.orientation())
    },
    state.conditions.goal,
    state.conditions.options
  };


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
