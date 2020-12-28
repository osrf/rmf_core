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

#include "DifferentialDriveHeuristic.hpp"
#include "a_star.hpp"

#include <rmf_utils/math.hpp>

#include <queue>

namespace rmf_traffic {
namespace agv {
namespace planning {

namespace {
//==============================================================================
Trajectory make_holding_trajectory(
    const Trajectory::Waypoint& waypoint,
    const rmf_traffic::Duration duration)
{
  Trajectory holding_trajectory;
  holding_trajectory.insert(waypoint);
  holding_trajectory.insert(
        waypoint.time() + duration,
        waypoint.position(),
        Eigen::Vector3d::Zero());
  return holding_trajectory;
}

} // anonymous namespace

//==============================================================================
template<typename NodePtrT>
struct DifferentialDriveCompare
{

  // Returning False implies that a is preferable to b
  // Returning True implies that b is preferable to a
  bool operator()(const NodePtrT& a, const NodePtrT& b)
  {
    // TODO(MXG): Micro-optimization: consider saving the sum of these values
    // in the Node instead of needing to re-add them for every comparison.
    const double a_value = a->info.remaining_cost_estimate + a->current_cost;
    const double b_value = b->info.remaining_cost_estimate + b->current_cost;

    // Note(MXG): The priority queue puts the greater value first, so we
    // reverse the arguments in this comparison.
    if (std::abs(a_value - b_value) > _threshold)
      return b_value < a_value;

    if (b->info.entry.value().orientation == Forward
        && a->info.entry.value().orientation != Forward)
      return true;
    else if (a->info.entry.value().orientation == Forward)
      return false;

    // If the cost estimates are within the threshold and there is no
    // orientation preference, then we'll prefer the one that seems to be closer
    // to the goal.
    return b->info.remaining_cost_estimate < a->info.remaining_cost_estimate;
  }

private:
  double _threshold;
};

//==============================================================================
class DifferentialDriveExpander
{
public:

  using NodeInfo = DifferentialDriveMapTypes::NodeInfo;

  struct SearchNode;
  using SearchNodePtr = std::shared_ptr<const SearchNode>;
  using NodePtr = SearchNodePtr;

  struct SearchNode
  {
    NodeInfo info;
    double current_cost;
    SearchNodePtr parent;
  };

  using Key = DifferentialDriveMapTypes::Key;
  using Entry = DifferentialDriveMapTypes::Entry;
  using EntryHash = DifferentialDriveMapTypes::EntryHash;

  using SearchQueue =
    std::priority_queue<
      SearchNodePtr,
      std::vector<SearchNodePtr>,
      DifferentialDriveCompare<SearchNodePtr>
    >;

  bool quit(const SearchNodePtr&, const SearchQueue&) const
  {
    return false;
  }

  bool is_finished(const SearchNodePtr& top) const
  {
    const auto& info = top->info;
    if (info.waypoint == _goal_waypoint)
    {
      if (!_goal_yaw.has_value())
        return true;

      const double angle_diff = rmf_utils::wrap_to_pi(info.yaw - *_goal_yaw);
      if (std::abs(angle_diff) <= _interpolate.rotation_thresh)
        return true;
    }

    return false;
  }

  void expand(const SearchNodePtr& top, SearchQueue& queue)
  {
    const auto& info = top->info;
    const auto current_entry = info.entry;
    assert(current_entry.has_value());
    if (!_visited.insert(current_entry.value()).second)
    {
      // This means we have already expanded from this entry before.
      // Expanding from here again is pointless because expanding from a more
      // costly parent cannot be better than expanding from a less costly one.
      return;
    }

    const auto current_wp_index = info.waypoint;
    if (current_wp_index == _goal_waypoint)
    {
      // If there is no goal yaw, then is_finished should have caught this node
      assert(_goal_yaw.has_value());
      queue.push(rotate_to_goal(top));
      return;
    }

    const auto traversals = _graph->traversals_from(current_wp_index);
    assert(traversals);
    for (const auto& traversal : *traversals)
    {
      const auto next_waypoint_index = traversal.finish_waypoint_index;
      const auto& next_waypoint =
          _graph->original().waypoints[next_waypoint_index];
      const Eigen::Vector2d next_position = next_waypoint.get_location();
      const std::string& next_map_name = next_waypoint.get_map_name();

      const double exit_event_duration = traversal.exit_event ?
            rmf_traffic::time::to_seconds(traversal.exit_event->duration())
          : 0.0;

      const auto next_lane_index = traversal.finish_lane_index;
      const auto remaining_cost_estimate = _heuristic.get(next_waypoint_index);
      if (!remaining_cost_estimate.has_value())
      {
        // If the heuristic for this waypoint is a nullopt, then there is no way
        // to reach the goal from this node. We should just skip this expansion.
        continue;
      }

      for (std::size_t i=0; i < traversal.alternatives.size(); ++i)
      {
        const auto& alt = traversal.alternatives[i];
        if (!alt.has_value())
          continue;

        const Orientation orientation = Orientation(i);

        const SearchNodePtr oriented_top = prepare_traversal(
              top, traversal.initial_lane_index, orientation, alt->yaw,
              alt->time + *remaining_cost_estimate + exit_event_duration,
              traversal.entry_event);

        Entry entry{next_lane_index, orientation};
        if (!_visited.count(entry))
        {
          // If we have already expanded from this waypoint, then there is no
          // point in re-expanding to it.
          continue;
        }

        // Check if a previous plan has found an optimal solution to get from
        // this entry
        // point to the solution.
        if (check_old_items(oriented_top, entry, queue))
          continue;

        // TODO(MXG): We should probably refactor the below into its own
        // function.
        Entry finishing_entry{
          traversal.finish_lane_index,
          orientation
        };

        std::optional<Entry> finishing_entry_opt = traversal.exit_event?
              std::nullopt : std::make_optional(finishing_entry);

        const double target_yaw = alt->yaw.value_or(
              oriented_top->info.yaw);

        auto new_node = std::make_shared<SearchNode>(
              SearchNode{
                NodeInfo{
                  finishing_entry_opt,
                  traversal.finish_waypoint_index,
                  next_position,
                  target_yaw,
                  *remaining_cost_estimate + exit_event_duration,
                  alt->time,
                  traversal.exit_event,
                  alt->routes
                },
                oriented_top->current_cost + alt->time,
                oriented_top
              });

        if (traversal.exit_event)
        {
          // The last node will trigger the exit event. This node will take over
          // when the exit event is finished.
          auto holding_trajectory = make_holding_trajectory(
                oriented_top->info.route_from_parent.back().trajectory().back(),
                traversal.exit_event->duration());

          new_node = std::make_shared<SearchNode>(
                SearchNode{
                  NodeInfo{
                    finishing_entry,
                    traversal.finish_waypoint_index,
                    next_position,
                    target_yaw,
                    *remaining_cost_estimate,
                    exit_event_duration,
                    nullptr,
                    {Route{next_map_name, std::move(holding_trajectory)}}
                  },
                  new_node->current_cost + exit_event_duration,
                  new_node
                });
        }

        queue.push(std::move(new_node));
      }
    }
  }

  SearchNodePtr rotate_to_goal(const SearchNodePtr& top)
  {
    const std::size_t waypoint_index = top->info.waypoint;
    const std::string& map_name =
        _graph->original().waypoints[waypoint_index].get_map_name();

    const Eigen::Vector2d p = top->info.position;
    const double target_yaw = _goal_yaw.value();
    const Eigen::Vector3d start_position = {p.x(), p.y(), top->info.yaw};
    const Eigen::Vector3d finish_position = {p.x(), p.y(), target_yaw};
    Trajectory trajectory_from_parent;
    trajectory_from_parent.insert(
          top->info.route_from_parent.back().trajectory().back().time(),
          start_position, Eigen::Vector3d::Zero());

    internal::interpolate_rotation(
          trajectory_from_parent,
          _w_nom, _alpha_nom,
          trajectory_from_parent.back().time(),
          start_position,
          finish_position,
          _interpolate.rotation_thresh);

    const double rotation_cost = rmf_traffic::time::to_seconds(
          trajectory_from_parent.duration());

    return std::make_shared<SearchNode>(
          SearchNode{
            NodeInfo{
              std::nullopt,
              top->info.waypoint,
              p,
              target_yaw,
              0.0,
              rotation_cost,
              nullptr,
              {Route{map_name, std::move(trajectory_from_parent)}}
            },
            top->current_cost + rotation_cost,
            top
          });
  }

  bool check_old_items(
      const SearchNodePtr& top,
      const Entry& entry,
      SearchQueue& queue)
  {
    const double base_cost = top->current_cost;

    Key key{
      entry.lane,
      entry.orientation,
      _goal_entry.lane,
      _goal_entry.orientation
    };

    const auto old_it = _old_items.find(key);
    if (old_it == _old_items.end())
      return false;

    auto solution = old_it->second;
    std::vector<NodeInfo> solution_info;
    while (solution)
    {
      solution_info.push_back(solution->info);
      solution = solution->child;
    }

    std::reverse(solution_info.begin(), solution_info.end());
    SearchNodePtr node = top;
    double current_cost = base_cost;
    for (auto&& info : solution_info)
    {
      current_cost += info.cost_from_parent;
      auto next_node = std::make_shared<SearchNode>(
            SearchNode{
              std::move(info),
              current_cost,
              node
            });
      node = next_node;
    }

    queue.push(node);

    return true;
  }

  SearchNodePtr prepare_traversal(
      SearchNodePtr top,
      const std::size_t target_lane_index,
      Orientation orientation,
      std::optional<double> yaw,
      const double remaining_cost_estimate,
      Graph::Lane::EventPtr entry_event)
  {
    const auto& original = _graph->original();
    const auto& lane = original.lanes[target_lane_index];
    const std::size_t target_waypoint_index = lane.entry().waypoint_index();
    const auto& target_waypoint = original.waypoints[target_waypoint_index];
    const std::string& map_name = target_waypoint.get_map_name();

    assert(!top->info.route_from_parent.empty());
    assert(!top->info.route_from_parent.back().trajectory().empty());

    const Eigen::Vector2d p = top->info.position;
    double target_yaw = top->info.yaw;
    const Eigen::Vector3d start_position = {p.x(), p.y(), target_yaw};
    Eigen::Vector3d finish_position = {p.x(), p.y(), *yaw};
    Trajectory trajectory_from_parent;
    trajectory_from_parent.insert(
          top->info.route_from_parent.back().trajectory().back().time(),
          start_position, Eigen::Vector3d::Zero());

    const double base_cost = top->current_cost;
    double rotation_cost = 0.0;
    if (yaw.has_value())
    {
      finish_position[2] = *yaw;
      internal::interpolate_rotation(
            trajectory_from_parent,
            _w_nom, _alpha_nom,
            trajectory_from_parent.back().time(),
            start_position,
            finish_position,
            _interpolate.rotation_thresh);

      target_yaw = *yaw;

      rotation_cost = rmf_traffic::time::to_seconds(
            trajectory_from_parent.duration());
    }

    double event_duration = 0.0;
    if (entry_event && entry_event->duration() > std::chrono::nanoseconds(0))
      event_duration = rmf_traffic::time::to_seconds(entry_event->duration());

    auto oriented_node = std::make_shared<SearchNode>(
          SearchNode{
            NodeInfo{
              Entry{
                target_lane_index,
                orientation
              },
              target_waypoint_index,
              p,
              target_yaw,
              remaining_cost_estimate + event_duration,
              rotation_cost,
              entry_event,
              {Route{map_name, std::move(trajectory_from_parent)}}
            },
            base_cost + rotation_cost,
            top
          });

    if (!entry_event || event_duration <= 0.0)
      return oriented_node;

    auto holding_trajectory = make_holding_trajectory(
          oriented_node->info.route_from_parent.back().trajectory().back(),
          entry_event->duration());

    return std::make_shared<SearchNode>(
          SearchNode{
            NodeInfo{
              std::nullopt,
              target_waypoint_index,
              p,
              target_yaw,
              remaining_cost_estimate,
              event_duration,
              nullptr,
              {Route{map_name, holding_trajectory}}
            },
            base_cost + rotation_cost + event_duration,
            oriented_node
          });
  }

  DifferentialDriveExpander(
    Entry goal_entry,
    const DifferentialDriveHeuristic::Storage& old_items,
    Cache<TranslationHeuristic> heuristic,
    std::shared_ptr<const Supergraph> graph)
  : _goal_entry(std::move(goal_entry)),
    _old_items(old_items),
    _heuristic(std::move(heuristic)),
    _graph(std::move(graph)),
    _interpolate(_graph->options()),

    // We choose to start with 4093 buckets because it's the largest prime
    // number below 2^12. This is mostly an arbitrary choice, except that
    // prime numbers make good bucket counts.
    //
    // TODO(MXG): We should experiment with this value to see how it impacts
    // performance
    _visited(4093, EntryHash(_graph->original().lanes.size()))
  {
    const auto& rotational = _graph->traits().rotational();
    _w_nom = rotational.get_nominal_velocity();
    _alpha_nom = rotational.get_nominal_acceleration();

    const auto& original = _graph->original();
    const auto& goal_lane = original.lanes[_goal_entry.lane];
    _goal_waypoint = goal_lane.exit().waypoint_index();
    _goal_yaw = _graph->yaw_of(_goal_entry);
  }

private:
  std::size_t _goal_waypoint;
  std::optional<double> _goal_yaw;
  Entry _goal_entry;
  const DifferentialDriveHeuristic::Storage& _old_items;
  Cache<TranslationHeuristic> _heuristic;
  std::shared_ptr<const Supergraph> _graph;
  double _w_nom;
  double _alpha_nom;
  Interpolate::Options::Implementation _interpolate;
  DifferentialDriveEntrySet _visited;
};

//==============================================================================
DifferentialDriveHeuristic::DifferentialDriveHeuristic(
  std::shared_ptr<const Supergraph> graph,
  CacheManagerPtr<TranslationHeuristic> heuristic)
: _graph(std::move(graph)),
  _heuristic(std::move(heuristic))
{
  // Do nothing
}

//==============================================================================
auto DifferentialDriveHeuristic::generate(
  const Key& key,
  const Storage& old_items,
  Storage& new_items) const -> SolutionNodePtr
{
  auto heuristic = _heuristic->get();
  const auto& original = _graph->original();
  const std::size_t start_lane_index = key.start_lane;
  const auto& start_lane = original.lanes[key.start_lane];
  const std::size_t start_waypoint_index = start_lane.entry().waypoint_index();

  auto start_heuristic = heuristic.get(start_waypoint_index);
  if (!start_heuristic.has_value())
  {
    // If the heuristic of this starting waypoint is a nullopt, then it is
    // impossible for the start to reach the goal.
    new_items.insert({key, nullptr});
    return nullptr;
  }

  const auto& start_wp = original.waypoints[start_waypoint_index];
  const Eigen::Vector2d start_p = start_wp.get_location();
  const std::string& start_map = start_wp.get_map_name();

  DifferentialDriveExpander::SearchQueue queue;

  std::optional<double> yaw = _graph->yaw_of(
    {key.start_lane, key.start_orientation});

  auto make_start_node = [&](const double yaw)
      -> DifferentialDriveExpander::SearchNodePtr
  {
    Trajectory trivial_trajectory;
    trivial_trajectory.insert(
          rmf_traffic::Time(rmf_traffic::Duration(0)),
          {start_p.x(), start_p.y(), yaw},
          Eigen::Vector3d::Zero());

    return std::make_shared<DifferentialDriveExpander::SearchNode>(
          DifferentialDriveExpander::SearchNode{
            DifferentialDriveExpander::NodeInfo{
              DifferentialDriveExpander::Entry{
                key.start_lane,
                key.start_orientation
              },
              start_waypoint_index,
              start_p,
              yaw,
              *start_heuristic,
              0.0,
              nullptr,
              {Route{start_map, std::move(trivial_trajectory)}}
            },
            0.0,
            nullptr
          });
  };

  if (yaw.has_value())
  {
    // If we can identify the starting yaw, we can just create one initial node.
    queue.push(make_start_node(*yaw));
  }
  else
  {
    // We cannot immediately identify the starting yaw, so let's look at every
    // traversal that comes out of the initial lane.
    const auto traversals = _graph->traversals_from(start_waypoint_index);
    assert(traversals);
    for (const auto& traversal : *traversals)
    {
      if (traversal.initial_lane_index == start_lane_index)
      {
        const auto& alt = traversal.alternatives[key.start_orientation];
        if (!alt.has_value())
          continue;

        const auto yaw = alt->yaw;
        if (!yaw.has_value())
          continue;

        queue.push(make_start_node(*yaw));
      }
    }

    if (queue.empty())
    {
      // We still have not found a suitable value for the yaw. This is a weird
      // edge case, but it is technically possible to construct. For now we'll
      // just use a yaw of 0.0.
      //
      // FIXME(MXG): Come up with a more robust (and guaranteed optimal) way of
      // handling this edge case.
      queue.push(make_start_node(0.0));
    }
  }

  DifferentialDriveExpander expander{
    DifferentialDriveExpander::Entry{key.goal_lane, key.goal_orientation},
    old_items,
    std::move(heuristic),
    _graph
  };

  const auto search = a_star_search(expander, queue);
  if (!search)
  {
    new_items.insert({key, nullptr});
    return nullptr;
  }

  std::vector<DifferentialDriveExpander::NodeInfo> node_info;
  auto node = search;
  SolutionNodePtr solution = nullptr;
  while (node)
  {
    solution = std::make_shared<SolutionNode>(
          SolutionNode{
            node->info,
            solution
          });

    if (const auto& entry = solution->info.entry)
    {
      Key new_key{
        entry->lane, entry->orientation,
        key.goal_lane, key.goal_orientation
      };

      new_items.insert({new_key, solution});
    }

    node = node->parent;
  }

  return solution;
}

//==============================================================================
DifferentialDriveHeuristicFactory::DifferentialDriveHeuristicFactory(
  std::shared_ptr<const Supergraph> graph)
: _graph(std::move(graph)),
  _heuristic_cache(std::make_shared<TranslationHeuristicFactory>(_graph))
{
  // Do nothing
}

//==============================================================================
ConstDifferentialDriveHeuristicPtr DifferentialDriveHeuristicFactory
::make(const std::size_t goal) const
{
  return std::make_shared<DifferentialDriveHeuristic>(
        _graph, _heuristic_cache.get(goal));
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
