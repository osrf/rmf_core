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

#include "EuclideanHeuristic.hpp"
#include "a_star.hpp"

#include <queue>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class EuclideanExpander
{
public:

  struct Node;
  using NodePtr = std::shared_ptr<const Node>;

  struct Node
  {
    std::size_t waypoint;

    // For the remaining_cost_estimate we'll use the Euclidean distance from the
    // current waypoint to the goal, regardless of differences in the floors of
    // each.
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

    // If the current waypoint does not have an entry in the old items, then we
    // need to keep expanding, step by step.
    const auto& current_wp = _graph->original().waypoints.at(current_wp_index);
    const auto& current_map = current_wp.get_map_name();
    const Eigen::Vector2d current_p = current_wp.get_location();

    if (current_map == _goal_map)
    {
      // We're on the same map as the goal, so we can reach it directly now.
      const auto cost = (_goal_p - current_p).norm()/_max_speed;

      auto new_node = std::make_shared<Node>(
            Node{
              _goal,
              0.0,
              current_cost + cost,
              top
            });

      queue.push(std::move(new_node));
      return;
    }

    const auto current_floor_it = _graph->floor_change().find(current_map);
    if (current_floor_it == _graph->floor_change().end())
    {
      // This means vehicles on the current floor have no way to move to any
      // other floor at all. Therefore we cannot expand from this node.
      return;
    }

    const auto& destinations = current_floor_it->second;
    for (const auto& [floor_name, routes] : destinations)
    {
      for (const auto& change : routes)
      {
        const auto change_start_wp_index =
            _graph->original().lanes[change.lane]
            .entry().waypoint_index();

        const Eigen::Vector2d first_p =
            _graph->original().waypoints[change_start_wp_index].get_location();

        double floor_change_cost = (first_p - current_p).norm()/_max_speed;

        const auto& lane = _graph->original().lanes.at(change.lane);
        if (const auto& entry_event = lane.entry().event())
        {
          floor_change_cost +=
              rmf_traffic::time::to_seconds(entry_event->duration());
        }

        if (const auto& exit_event = lane.exit().event())
        {
          floor_change_cost +=
              rmf_traffic::time::to_seconds(exit_event->duration());
        }

        const auto change_finish_wp_index =
            _graph->original().lanes[change.lane]
            .exit().waypoint_index();

        const Eigen::Vector2d last_p =
            _graph->original().waypoints[change_finish_wp_index].get_location();

        const double remaining_cost_estimate =
            (_goal_p - last_p).norm()/_max_speed;

        auto new_node = std::make_shared<Node>(
              Node{
                change_finish_wp_index,
                remaining_cost_estimate,
                current_cost + floor_change_cost,
                top
              });

        queue.push(std::move(new_node));
      }
    }
  }

  EuclideanExpander(
    std::size_t goal,
    Eigen::Vector2d goal_p,
    const std::string& goal_map,
    double max_speed,
    const EuclideanHeuristic::Storage& old_items,
    std::shared_ptr<const Supergraph> graph)
  : _goal(goal),
    _goal_p(goal_p),
    _goal_map(goal_map),
    _max_speed(max_speed),
    _old_items(old_items),
    _graph(std::move(graph))
  {
    // Do nothing
  }

private:
  std::size_t _goal;
  Eigen::Vector2d _goal_p;
  const std::string& _goal_map;
  double _max_speed;
  const EuclideanHeuristic::Storage& _old_items;
  std::shared_ptr<const Supergraph> _graph;
  std::unordered_set<std::size_t> _visited;
};

//==============================================================================
EuclideanHeuristic::EuclideanHeuristic(
  std::size_t goal,
  double max_speed,
  std::shared_ptr<const Supergraph> graph)
: _goal(goal),
  _max_speed(max_speed),
  _graph(std::move(graph))
{
  const auto& goal_wp = _graph->original().waypoints.at(goal);
  _goal_p = goal_wp.get_location();
  _goal_map = &goal_wp.get_map_name();
}

//==============================================================================
std::optional<double> EuclideanHeuristic::generate(
    const std::size_t& key,
    const Storage& old_items,
    Storage& new_items) const
{
  const auto& start_wp = _graph->original().waypoints.at(key);
  const auto& start_map = start_wp.get_map_name();
  const Eigen::Vector2d start_p = start_wp.get_location();
  const auto minimum_cost = (_goal_p - start_p).norm()/_max_speed;

  if (start_map == *_goal_map)
  {
    const auto it = new_items.insert({key, minimum_cost});
    return it.first->second;
  }

  EuclideanExpander expander{
    _goal,
    _goal_p,
    *_goal_map,
    _max_speed,
    old_items,
    _graph
  };

  EuclideanExpander::SearchQueue queue;
  queue.push(
    std::make_shared<EuclideanExpander::Node>(
      EuclideanExpander::Node{
        key,
        minimum_cost,
        0.0,
        nullptr
      }));

  const EuclideanExpander::NodePtr solution = a_star_search(expander, queue);
  if (!solution)
  {
    // This means there is no way to move to the goal from the start waypoint
    new_items.insert({key, std::nullopt});
    return std::nullopt;
  }

  const double final_cost = solution->current_cost;
  auto node = solution;
  do
  {
    new_items.insert({node->waypoint, final_cost - node->current_cost});

    // We can save the results for every waypoint that was used in this solution
    // because every segment of an optimal solution is an optimal solution
    // itself
    node = node->parent;
  } while (node);

  return final_cost;
}

//==============================================================================
EuclideanHeuristicFactory::EuclideanHeuristicFactory(
  std::shared_ptr<const Supergraph> graph,
  double max_speed)
: _graph(std::move(graph)),
  _max_speed(max_speed)
{
  // Do nothing
}

//==============================================================================
ConstEuclideanHeuristicPtr EuclideanHeuristicFactory::make(
    const std::size_t goal) const
{
  return std::make_shared<EuclideanHeuristic>(goal, _max_speed, _graph);
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
