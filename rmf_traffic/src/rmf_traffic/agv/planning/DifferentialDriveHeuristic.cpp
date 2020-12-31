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

#include <iostream>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class DifferentialDriveExpander
{
public:

  using NodeInfo = DifferentialDriveMapTypes::NodeInfo;

  struct SearchNode;
  using SearchNodePtr = std::shared_ptr<const SearchNode>;
  using NodePtr = SearchNodePtr;

  using RouteFactory = DifferentialDriveMapTypes::RouteFactory;
  using RouteFactoryFactory = DifferentialDriveMapTypes::RouteFactoryFactory;

  struct SearchNode
  {
    NodeInfo info;
    double current_cost;
    RouteFactoryFactory route_factory;
    SearchNodePtr parent;

    double get_total_cost_estimate() const
    {
      return current_cost + info.remaining_cost_estimate;
    }

    double get_remaining_cost_estimate() const
    {
      return info.remaining_cost_estimate;
    }

    std::optional<Orientation> get_orientation() const
    {
      if (info.entry.has_value())
        return info.entry->orientation;

      return std::nullopt;
    }
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
      if (!_goal_yaw.has_value() || !top->info.yaw.has_value())
      {
//        std::cout << " -- Found solution at (" << _goal_waypoint
//                  << ", nullopt)" << std::endl;
//        std::cout << " !! Total Expansions: " << _expansion_count << std::endl;
        return true;
      }

      const double angle_diff = rmf_utils::wrap_to_pi(*info.yaw - *_goal_yaw);
      if (std::abs(angle_diff) <= _interpolate.rotation_thresh)
      {
//        std::cout << " -- Found solution at (" << _goal_waypoint
//                  << ", " << info.yaw.value()*180.0/M_PI << ")" << std::endl;
//        std::cout << " !! Total Expansions: " << _expansion_count << std::endl;
        return true;
      }
    }

    return false;
  }

  void expand(const SearchNodePtr& top, SearchQueue& queue)
  {
//    std::cout << "Expanding from " << top->info.waypoint << ": ["
//              << top->info.entry->lane << ", "
//              << top->info.entry->orientation << ", "
//              << top->info.entry->side
//              << "]" << std::endl;

    const auto& info = top->info;
    const auto current_entry = info.entry;
    assert(current_entry.has_value());
    if (!_visited.insert(current_entry.value()).second)
    {
//      std::cout << " -- already visited" << std::endl;
      // This means we have already expanded from this entry before.
      // Expanding from here again is pointless because expanding from a more
      // costly parent cannot be better than expanding from a less costly one.
      return;
    }

    ++_expansion_count;

    const auto current_wp_index = info.waypoint;
    if (current_wp_index == _goal_waypoint)
    {
      // If there is no goal yaw, then is_finished should have caught this node
      assert(_goal_yaw.has_value());

      auto node = rotate_to_goal(top);
//      std::cout << __LINE__ << " pushing rotation to goal: ("
//                << node->info.waypoint << ", " << node->info.yaw.value_or(std::nan("")) << ")"
//                << std::endl;

      queue.push(std::move(node));
      return;
    }

    const auto traversals = _graph->traversals_from(current_wp_index);
    assert(traversals);
//    std::cout << " -- " << traversals->size() << " traversals" << std::endl;
    for (const auto& traversal : *traversals)
    {
      const auto next_waypoint_index = traversal.finish_waypoint_index;
      const auto& next_waypoint =
          _graph->original().waypoints[next_waypoint_index];
      const Eigen::Vector2d next_position = next_waypoint.get_location();

      const double exit_event_duration = traversal.exit_event ?
        rmf_traffic::time::to_seconds(traversal.exit_event->duration()) : 0.0;

//      std::cout << "    " << next_waypoint_index << ": ";

      const auto next_lane_index = traversal.finish_lane_index;
      const auto remaining_cost_estimate = _heuristic.get(next_waypoint_index);
      if (!remaining_cost_estimate.has_value())
      {
//        std::cout << "nullopt remaining_cost_estimate" << std::endl;
        // If the heuristic for this waypoint is a nullopt, then there is no way
        // to reach the goal from this node. We should just skip this expansion.
        continue;
      }

      for (std::size_t i=0; i < traversal.alternatives.size(); ++i)
      {
        const auto& alt = traversal.alternatives[i];
        if (!alt.has_value())
        {
//          std::cout << "skipping nullopt alt " << i << " | ";
          continue;
        }

        const Orientation orientation = Orientation(i);

        const SearchNodePtr oriented_top = prepare_traversal(
              top, traversal.initial_lane_index, orientation, alt->yaw,
              alt->time + *remaining_cost_estimate + exit_event_duration,
              traversal.entry_event, traversal.maps);

        Entry finishing_entry{next_lane_index, orientation, Side::Finish};
        if (_visited.count(finishing_entry))
        {
//          std::cout << "skipping pre-visited alt [" << next_lane_index
//                    << ", " << orientation << ", " << Side::Finish << "] | ";
          // If we have already expanded from this waypoint, then there is no
          // point in re-expanding to it.
          continue;
        }

        // Check if a previous plan has found an optimal solution to get from
        // this entry
        // point to the solution.
        if (check_old_items(oriented_top, finishing_entry, queue))
        {
//          std::cout << "skipping old alt " << i << " | ";
          continue;
        }

        // TODO(MXG): We should probably refactor the below into its own
        // function.

        std::optional<Entry> finishing_entry_opt = traversal.exit_event?
              std::nullopt : std::make_optional(finishing_entry);

        const std::optional<double> target_yaw = alt->yaw;

        auto new_node = std::make_shared<SearchNode>(
              SearchNode{
                NodeInfo{
                  finishing_entry_opt,
                  traversal.finish_waypoint_index,
                  next_position,
                  target_yaw,
                  *remaining_cost_estimate + exit_event_duration,
                  alt->time,
                  traversal.exit_event
                },
                oriented_top->current_cost + alt->time,
                alt->routes,
                oriented_top
              });
//        std::cout << " " << __LINE__ << " making[" << new_node << "] ";

        if (traversal.exit_event)
        {
          // The last node will trigger the exit event. This node will take over
          // when the exit event is finished.
          auto routes = make_hold_factory(
                next_position,
                target_yaw,
                traversal.exit_event->duration(),
                _limits,
                _interpolate.rotation_thresh,
                traversal.maps);

          new_node = std::make_shared<SearchNode>(
                SearchNode{
                  NodeInfo{
                    finishing_entry,
                    traversal.finish_waypoint_index,
                    next_position,
                    target_yaw,
                    *remaining_cost_estimate,
                    exit_event_duration,
                    nullptr
                  },
                  new_node->current_cost + exit_event_duration,
                  std::move(routes),
                  new_node
                });
//          std::cout << " " << __LINE__ << " making[" << new_node << "] ";
        }

//        std::cout << __LINE__ << " pushing traversal: ("
//                  << new_node->info.waypoint << ", " << new_node->info.yaw.value_or(std::nan("")) << ") | ";
        queue.push(std::move(new_node));
      }

//      std::cout << std::endl;
    }
  }

  SearchNodePtr rotate_to_goal(const SearchNodePtr& top)
  {
    const std::size_t waypoint_index = top->info.waypoint;
    const std::string& map_name =
        _graph->original().waypoints[waypoint_index].get_map_name();

    const Eigen::Vector2d p = top->info.position;
    const double target_yaw = _goal_yaw.value();
    const Eigen::Vector3d start_position =
      {p.x(), p.y(), top->info.yaw.value()};

    // We assume we will get back a valid factory, because if no rotation is
    // needed, then the planner should have accepted this node earlier.
    auto factory_info = make_rotate_factory(
          p, top->info.yaw, target_yaw, _limits,
          _interpolate.rotation_thresh, map_name).value();

    const double rotation_cost = factory_info.minimum_cost;

    auto new_node = std::make_shared<SearchNode>(
          SearchNode{
            NodeInfo{
              _goal_entry,
              top->info.waypoint,
              p,
              target_yaw,
              0.0,
              rotation_cost,
              nullptr
            },
            top->current_cost + rotation_cost,
            std::move(factory_info.factory),
            top
          });
//    std::cout << " " << __LINE__ << " making[" << new_node << "] ";
    return new_node;
  }

  bool check_old_items(
      const SearchNodePtr& top,
      const Entry& entry,
      SearchQueue& queue)
  {
    Key key{
      entry.lane,
      entry.orientation,
      entry.side,
      _goal_entry.lane,
      _goal_entry.orientation
    };

//    std::cout << " > looking up ["
//              << key.start_lane << ", "
//              << key.start_orientation << ", "
//              << key.start_side << ", "
//              << key.goal_lane << ", "
//              << key.goal_orientation << "] < ";

    const auto old_it = _old_items.find(key);
    if (old_it == _old_items.end())
      return false;

    auto solution = old_it->second;
    auto node = top;
    while (solution)
    {
      node = std::make_shared<SearchNode>(
            SearchNode{
              solution->info,
              node->current_cost + solution->info.cost_from_parent,
              make_recycling_factory(solution->route_factory),
              node
            });
//      std::cout << " " << __LINE__ << " making[" << node << "] ";

      solution = solution->child;
    }

//    std::cout << __LINE__ << " pushing from old archives: ("
//              << node->info.waypoint << ", " << node->info.yaw.value_or(std::nan("")) << ")"
//              << std::endl;
    queue.push(node);

    return true;
  }

  SearchNodePtr prepare_traversal(
      SearchNodePtr top,
      const std::size_t target_lane_index,
      Orientation orientation,
      std::optional<double> finish_yaw,
      const double remaining_cost_estimate,
      Graph::Lane::EventPtr entry_event,
      const std::vector<std::string>& maps)
  {
    const auto& original = _graph->original();
    const auto& lane = original.lanes[target_lane_index];
    const std::size_t target_waypoint_index = lane.entry().waypoint_index();
    const auto& target_waypoint = original.waypoints[target_waypoint_index];
    const std::string& initial_map_name = target_waypoint.get_map_name();
    const Eigen::Vector2d p = top->info.position;

    double event_duration = 0.0;
    if (entry_event && entry_event->duration() > std::chrono::nanoseconds(0))
      event_duration = rmf_traffic::time::to_seconds(entry_event->duration());

    auto rotation_factory = make_rotate_factory(
          p, top->info.yaw, finish_yaw, _limits,
          _interpolate.rotation_thresh, initial_map_name);

    const bool needs_rotation = rotation_factory.has_value();
    if (!needs_rotation && !entry_event)
    {
      // If no rotation nor entry event is needed to begin the traversal, then
      // we can just return the top as-is, because it is already suitable to
      // begin the traversal.
      return top;
    }

    SearchNodePtr oriented_node = top;
    if (needs_rotation)
    {
      const double rotation_cost = rotation_factory->minimum_cost;
      oriented_node =
        std::make_shared<SearchNode>(
          SearchNode{
            NodeInfo{
              Entry{
                target_lane_index,
                orientation,
                Side::Start
              },
              target_waypoint_index,
              p,
              finish_yaw,
              remaining_cost_estimate + event_duration,
              rotation_cost,
              entry_event
            },
            top->current_cost + rotation_cost,
            std::move(rotation_factory->factory),
            top
          });
    }

    if (!entry_event || entry_event->duration() <= std::chrono::nanoseconds(0))
    {
      // If there is no event (or the event has no duration), we can just return
      // the oriented node.
      return oriented_node;
    }

    auto hold_factory =
        make_hold_factory(
          p, finish_yaw, entry_event->duration(),
          _limits, _interpolate.rotation_thresh, maps);

    return std::make_shared<SearchNode>(
          SearchNode{
            NodeInfo{
              std::nullopt,
              target_waypoint_index,
              oriented_node->info.position,
              oriented_node->info.yaw,
              remaining_cost_estimate,
              event_duration,
              nullptr
            },
            oriented_node->current_cost + event_duration,
            std::move(hold_factory),
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
    _limits(VehicleTraits::Implementation::get_limits(_graph->traits())),
    _interpolate(_graph->options()),

    // We choose to start with 4093 buckets because it's the largest prime
    // number below 2^12. This is mostly an arbitrary choice, except that
    // prime numbers make good bucket counts.
    //
    // TODO(MXG): We should experiment with this value to see how it impacts
    // performance
    _visited(4093, EntryHash(_graph->original().lanes.size()))
  {
    const auto& original = _graph->original();
    const auto& goal_lane = original.lanes[_goal_entry.lane];
    _goal_waypoint = goal_lane.exit().waypoint_index();

//    std::cout << " ----- Getting goal yaw:" << std::endl;
    _goal_yaw = _graph->yaw_of(_goal_entry);
//    std::cout << " ----- Done: ";
//    if (_goal_yaw.has_value())
//      std::cout << _goal_yaw.value() << std::endl;
//    else
//      std::cout << "nullopt" << std::endl;
  }

private:
  std::size_t _goal_waypoint;
  std::optional<double> _goal_yaw;
  Entry _goal_entry;
  const DifferentialDriveHeuristic::Storage& _old_items;
  Cache<TranslationHeuristic> _heuristic;
  std::shared_ptr<const Supergraph> _graph;
  KinematicLimits _limits;
  Interpolate::Options::Implementation _interpolate;
  DifferentialDriveEntrySet _visited;
  std::size_t _expansion_count = 0;
};

//==============================================================================
DifferentialDriveHeuristic::DifferentialDriveHeuristic(
  std::shared_ptr<const Supergraph> graph)
: _graph(std::move(graph)),
  _heuristic_map(std::make_shared<TranslationHeuristicFactory>(_graph))
{
  // Do nothing
}

//==============================================================================
auto DifferentialDriveHeuristic::generate(
  const Key& key,
  const Storage& old_items,
  Storage& new_items) const -> SolutionNodePtr
{
  using SearchQueue = DifferentialDriveExpander::SearchQueue;
  using SearchNode = DifferentialDriveExpander::SearchNode;
  using NodeInfo = DifferentialDriveExpander::NodeInfo;
  using Entry = DifferentialDriveExpander::Entry;
  using RouteFactory = DifferentialDriveExpander::RouteFactory;

  const auto& original = _graph->original();
  const auto& start_lane = original.lanes[key.start_lane];
  const std::size_t start_waypoint_index = start_lane.entry().waypoint_index();
  const std::size_t goal_lane_index = key.goal_lane;
  const auto& goal_lane = original.lanes[goal_lane_index];
  const std::size_t goal_waypoint_index = goal_lane.exit().waypoint_index();

  auto heuristic = _heuristic_map.get(goal_waypoint_index)->get();

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

  SearchQueue queue;

  std::optional<double> yaw = _graph->yaw_of(
    {key.start_lane, key.start_orientation, key.start_side});

  queue.push(
    std::make_shared<SearchNode>(
      SearchNode{
        NodeInfo{
          Entry{
            key.start_lane,
            key.start_orientation,
            key.start_side
          },
          start_waypoint_index,
          start_p,
          yaw,
          *start_heuristic,
          0.0,
          nullptr
        },
        0.0,
        nullptr,
        nullptr
      }));

  DifferentialDriveExpander expander{
    DifferentialDriveExpander::Entry{
      key.goal_lane,
      key.goal_orientation,
      Side::Finish
    },
    old_items,
    std::move(heuristic),
    _graph
  };

//  std::cout << "Initial queue size: " << queue.size() << std::endl;
  const auto search = a_star_search(expander, queue);
  if (!search)
  {
    new_items.insert({key, nullptr});
    return nullptr;
  }

  std::vector<DifferentialDriveExpander::NodeInfo> node_info;
  auto goal_node = search;
  SolutionNodePtr output = nullptr;
  SolutionNodePtr solution = nullptr;


  while (goal_node)
  {
    while (goal_node && !goal_node->info.entry.has_value())
    {
      // Keep moving goal_node forward until it has entry info
      goal_node = goal_node->parent;
    }

    // TODO(MXG): We're stashing some unnecessary solutions here, like
    // [x y F x y]. We could add in some more logic here to prevent that
    // unnecessary stashing but we're leaving it as-is right now to minimize
    // code complexity.
    auto node = goal_node;
    auto previous_node = node;

    solution = nullptr;
    double remaining_cost = 0.0;
    while (node)
    {
      auto info = node->info;
      info.remaining_cost_estimate = remaining_cost;
      remaining_cost += info.cost_from_parent;

      RouteFactory routes;
      if (node->route_factory)
      {
        if (solution)
          routes = node->route_factory(solution->info.yaw);
        else
          routes = node->route_factory(node->info.yaw);
      }

      solution = std::make_shared<SolutionNode>(
            SolutionNode{
              std::move(info),
              std::move(routes),
              solution
            });

      if (const auto& entry = solution->info.entry)
      {
        const auto goal_entry = goal_node->info.entry.value();
        Key new_key{
          entry->lane, entry->orientation, entry->side,
          goal_entry.lane, goal_entry.orientation
        };

//        std::cout << " << stashing " << node << " -> " << goal_node << " ["
//                  << new_key.start_lane << ", "
//                  << new_key.start_orientation << ", "
//                  << new_key.start_side << ", "
//                  << new_key.goal_lane << ", "
//                  << new_key.goal_orientation << "]: ("
//                  << solution->info.waypoint << ", "
//                  << solution->info.yaw.value_or(std::nan(""))*180.0/M_PI << ") -> ("
//                  << goal_node->info.waypoint << ", "
//                  << goal_node->info.yaw.value_or(std::nan(""))*180.0/M_PI << ") => "
//                  << solution->info.remaining_cost_estimate << std::endl;

        // TODO(MXG): Add a bool field to SearchNode to indicate whether it is
        // newly generated or pulled from a previous solution. Then we can skip
        // inserting the old nodes here. The cost should be minor enough that
        // it won't matter.
        new_items.insert({new_key, solution});
      }

      previous_node = node;
      node = node->parent;
    }

    if (!output)
      output = solution;

    goal_node = goal_node->parent;
  }

  return output;
}

//==============================================================================
CacheManagerPtr<DifferentialDriveHeuristic>
DifferentialDriveHeuristic::make_manager(
  std::shared_ptr<const Supergraph> supergraph)
{
  const std::size_t N = supergraph->original().lanes.size();
  return CacheManager<Cache<DifferentialDriveHeuristic>>::make(
    std::make_shared<DifferentialDriveHeuristic>(std::move(supergraph)),
    [N](){ return Storage(4093, DifferentialDriveMapTypes::KeyHash{N}); });
}

//==============================================================================
DifferentialDriveHeuristicAdapter::DifferentialDriveHeuristicAdapter(
  Cache<DifferentialDriveHeuristic> cache,
  std::shared_ptr<const Supergraph> graph,
  std::size_t goal_waypoint,
  std::optional<double> goal_yaw)
: _cache(std::move(cache)),
  _graph(std::move(graph)),
  _goal_waypoint(goal_waypoint),
  _goal_yaw(goal_yaw),
  _w_nom(_graph->traits().rotational().get_nominal_velocity()),
  _alpha_nom(_graph->traits().rotational().get_nominal_acceleration()),
  _rotation_threshold(_graph->options().rotation_thresh)
{
  // Do nothing
}

//==============================================================================
std::optional<double> DifferentialDriveHeuristicAdapter::compute(
  std::size_t start_waypoint,
  double yaw) const
{
  const auto keys = _graph->keys_for(start_waypoint, _goal_waypoint, _goal_yaw);

  std::optional<double> best_cost;
  for (const auto& key : keys)
  {
    const auto solution = _cache.get(key);
    if (!solution)
      continue;

    const auto target_yaw = _graph->yaw_of(
      {key.start_lane, key.start_orientation, key.start_side});

    double cost = solution->info.remaining_cost_estimate;
    if (target_yaw.has_value())
    {
      cost += time::to_seconds(internal::estimate_rotation_time(
            _w_nom, _alpha_nom, yaw, *target_yaw, _rotation_threshold));
    }

    if (!best_cost.has_value() || cost < *best_cost)
      best_cost = cost;
  }

  return best_cost;
}

//==============================================================================
auto DifferentialDriveHeuristicAdapter::compute(Entry start) const
-> SolutionNodePtr
{
  const auto goal_entries = _graph->entries_into(_goal_waypoint)
      ->relevant_entries(_goal_yaw);

  SolutionNodePtr best_solution;
  for (const auto& goal_entry : goal_entries)
  {
    const Key key{
      start.lane,
      start.orientation,
      start.side,
      goal_entry.lane,
      goal_entry.orientation
    };

    const auto solution = _cache.get(key);
    if (!solution)
      continue;

    const double cost = solution->info.remaining_cost_estimate;
    if (!best_solution || cost < best_solution->info.remaining_cost_estimate)
      best_solution = solution;
  }

  return best_solution;
}

//==============================================================================
const Cache<DifferentialDriveHeuristic>&
DifferentialDriveHeuristicAdapter::cache() const
{
  return _cache;
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
