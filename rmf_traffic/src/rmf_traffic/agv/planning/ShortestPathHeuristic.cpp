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

#include "ShortestPathHeuristic.hpp"
#include "a_star.hpp"

#include <queue>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class ShortestPathExpander
{
public:

  struct Node;
  using NodePtr = std::shared_ptr<const Node>;

  struct Node
  {
    std::size_t waypoint;

    // For the remaining_cost_estimate we'll use the Euclidean Heuristic, which
    // takes floor changes into account.
    double remaining_cost_estimate;
    double current_cost;
    NodePtr parent;
  };

  using SearchQueue =
    std::priority_queue<
      NodePtr, std::vector<NodePtr>, SimpleCompare<NodePtr>>;

  bool quit(const NodePtr&, const SearchQueue&) const
  {
    return false;
  }

  bool is_finished(const NodePtr& top) const
  {
    if (top->waypoint == _goal)
      return true;

    return false;
  }

  void expand(const NodePtr& top, SearchQueue& queue)
  {
    const auto current_wp_index = top->waypoint;
    if (!_visited.insert(current_wp_index).second)
    {
      // This means we have already expanded from this waypoint before.
      // Expanding from here again is pointless because expanding from a more
      // costly parent cannot be better than expanding from a less costly one.
      return;
    }

    const auto current_cost = top->current_cost;
    const auto old_it = _old_items.find(current_wp_index);
    if (old_it != _old_items.end())
    {
      // If the current waypoint already has an entry in the old items, then we
      // can immediately create a node that brings it the rest of the way to the
      // goal with the best possible cost.
      const auto remaining_cost = old_it->second;
      if (!remaining_cost.has_value())
      {
        // If the old value is a nullopt, then this waypoint has no way to reach
        // the goal, so we should just discard it.
        return;
      }

      auto new_node = std::make_shared<Node>(
            Node{
              _goal,
              0.0,
              current_cost + remaining_cost.value(),
              top
            });

      queue.push(std::move(new_node));
      return;
    }

    for (const auto l : _graph->original().lanes_from[current_wp_index])
    {
      const auto& lane = _graph->original().lanes[l];
      const auto& exit = lane.exit();
      const auto next_waypoint_index = exit.waypoint_index();

      if (_visited.count(next_waypoint_index))
      {
        // If we have already expanded from this waypoint, then there is no
        // point in re-expanding to it.
        continue;
      }

      const std::optional<double> remaining_cost_estimate =
          _heuristic.get(next_waypoint_index);

      if (!remaining_cost_estimate.has_value())
      {
        // If the heuristic for this waypoint comes up as a nullopt, then there
        // is definitely no path from this waypoint to the goal. Therefore we
        // should not bother expanding to it.
        continue;
      }

      const auto& entry = lane.entry();
      assert(entry.waypoint_index() == current_wp_index);
      const auto& current_wp = _graph->original().waypoints[current_wp_index];
      const Eigen::Vector2d p0 = current_wp.get_location();

      const auto& next_wp = _graph->original().waypoints[next_waypoint_index];
      const Eigen::Vector2d p1 = next_wp.get_location();

      double local_cost = (p1 - p0).norm()/_max_speed;
      if (const auto entry_event = entry.event())
        local_cost += rmf_traffic::time::to_seconds(entry_event->duration());

      if (const auto exit_event = exit.event())
        local_cost += rmf_traffic::time::to_seconds(exit_event->duration());

      auto new_node = std::make_shared<Node>(
            Node{
              next_waypoint_index,
              remaining_cost_estimate.value(),
              current_cost + local_cost,
              top
            });

      queue.push(std::move(new_node));
    }
  }

  ShortestPathExpander(
      std::size_t goal,
      Eigen::Vector2d goal_p,
      double max_speed,
      const ShortestPathHeuristic::Storage& old_items,
      Cache<EuclideanHeuristic> heuristic,
      std::shared_ptr<const Supergraph> graph)
    : _goal(goal),
      _goal_p(goal_p),
      _max_speed(max_speed),
      _old_items(old_items),
      _heuristic(std::move(heuristic)),
      _graph(std::move(graph))
  {
    // Do nothing
  }

private:
  std::size_t _goal;
  Eigen::Vector2d _goal_p;
  double _max_speed;
  const ShortestPathHeuristic::Storage& _old_items;
  Cache<EuclideanHeuristic> _heuristic;
  std::shared_ptr<const Supergraph> _graph;
  std::unordered_set<std::size_t> _visited;
};

//==============================================================================
ShortestPathHeuristic::ShortestPathHeuristic(
    std::size_t goal,
    double max_speed,
    std::shared_ptr<const Supergraph> graph,
    CacheManagerPtr<EuclideanHeuristic> heuristic)
  : _goal(goal),
    _max_speed(max_speed),
    _graph(std::move(graph)),
    _heuristic(std::move(heuristic))
{
  const auto& goal_wp = _graph->original().waypoints.at(goal);
  _goal_p = goal_wp.get_location();
  _goal_map = &goal_wp.get_map_name();
}

//==============================================================================
std::optional<double> ShortestPathHeuristic::generate(
    const std::size_t& key,
    const Storage& old_items,
    Storage& new_items) const
{
  auto heuristic = _heuristic->get();
  auto start_heuristic = heuristic.get(key);
  if (!start_heuristic.has_value())
  {
    // If the heuristic of this starting waypoint is a nullopt, then it is
    // impossible for the start to reach the goal.
    new_items.insert({key, std::nullopt});
    return std::nullopt;
  }

  ShortestPathExpander::SearchQueue queue;
  queue.push(
    std::make_shared<ShortestPathExpander::Node>(
      ShortestPathExpander::Node{
        key,
        start_heuristic.value(),
        0.0,
        nullptr
      }));

  ShortestPathExpander expander{
    _goal,
    _goal_p,
    _max_speed,
    old_items,
    std::move(heuristic),
    _graph
  };

  const ShortestPathExpander::NodePtr solution = a_star_search(expander, queue);
  if (!solution)
  {
    // This means there is no way to move to the goal from the start waypoint
    new_items.insert({key, std::nullopt});
    return std::nullopt;
  }

  const double final_cost = solution->current_cost;
  auto node = solution;
  while (node)
  {
    // We can save the results for every waypoint that was used in this solution
    // because every segment of an optimal solution is an optimal solution
    // itself.
    new_items.insert({node->waypoint, final_cost - node->current_cost});
    node = node->parent;
  }

  return final_cost;
}

//==============================================================================
ShortestPathHeuristicFactory::ShortestPathHeuristicFactory(
    std::shared_ptr<const Supergraph> graph,
    double max_speed)
  : _graph(std::move(graph)),
    _max_speed(max_speed),
    _heuristic_cache(
      std::make_shared<EuclideanHeuristicFactory>(_graph, max_speed))
{
  // Do nothing
}

//==============================================================================
ConstShortestPathHeuristicPtr ShortestPathHeuristicFactory::make(
    const std::size_t goal) const
{
  return std::make_shared<ShortestPathHeuristic>(
        goal, _max_speed, _graph, _heuristic_cache.get(goal));
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
