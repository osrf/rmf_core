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

#include "TranslationHeuristic.hpp"
#include "a_star.hpp"

#include <queue>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class TranslationExpander
{
public:

  struct Node;
  using NodePtr = std::shared_ptr<const Node>;

  struct Node
  {
    std::size_t waypoint;

    // For the remaining_cost_estimate we'll use the ShortestPathExpander
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
    // TODO(MXG): We could probably refactor this so that
    // Euclidean, ShortestPath, and Translation Heuristics
    // all share most of this implementation.

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


  }

private:
  std::size_t _goal;
  double _max_speed;
  const TranslationHeuristic::Storage& _old_items;
  Cache<ShortestPathHeuristic> _heuristic;
  std::shared_ptr<const Supergraph> _graph;
  std::unordered_set<std::size_t> _visited;
};

//==============================================================================
TranslationHeuristic::TranslationHeuristic(
  std::size_t goal,
  double max_speed,
  double acceleration,
  std::shared_ptr<const Supergraph> graph,
  CacheManagerPtr<ShortestPathHeuristic> heuristic)
: _goal(goal),
  _max_speed(max_speed),
  _acceleration(acceleration),
  _graph(std::move(graph)),
  _heuristic(std::move(heuristic))
{
  // Do nothing
}

//==============================================================================
std::optional<double> TranslationHeuristic::generate(
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
}

//==============================================================================
TranslationHeuristicFactory::TranslationHeuristicFactory(
  std::shared_ptr<const Supergraph> graph,
  double max_speed,
  double acceleration)
  : _graph(std::move(graph)),
    _max_speed(max_speed),
    _acceleration(acceleration),
    _heuristic_cache(
      std::make_shared<ShortestPathHeuristicFactory>(_graph, max_speed))
{
  // Do nothing
}

//==============================================================================
ConstTranslationHeuristicPtr TranslationHeuristicFactory::make(
    const std::size_t goal) const
{
  return std::make_shared<TranslationHeuristic>(
        goal, _max_speed, _acceleration, _graph, _heuristic_cache.get(goal));
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
