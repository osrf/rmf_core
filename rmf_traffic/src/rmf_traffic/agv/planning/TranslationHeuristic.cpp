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

    const auto traversals = _graph->traversals_from(current_wp_index);
    assert(traversals);
    for (const auto& traversal : *traversals)
    {
      const auto next_waypoint_index = traversal.finish_waypoint_index;
      if (_visited.count(next_waypoint_index))
      {
        // If we have already expanded from this waypoint, then there is no
        // point in re-expanding to it.
        continue;
      }

      const auto remaining_cost_estimate = _heuristic.get(next_waypoint_index);
      if (!remaining_cost_estimate.has_value())
      {
        // If the heuristic for this waypoint is a nullopt, then there is no way
        // to reach the goal from this node. We should just skip this expansion.
        continue;
      }

      auto new_node = std::make_shared<Node>(
            Node{
              next_waypoint_index,
              remaining_cost_estimate.value(),
              current_cost + traversal.best_time,
              top
            });

      queue.push(std::move(new_node));
    }
  }

  TranslationExpander(
      std::size_t goal,
      const TranslationHeuristic::Storage& old_items,
      Cache<ShortestPathHeuristic> heuristic,
      std::shared_ptr<const Supergraph> graph)
    : _goal(goal),
      _old_items(old_items),
      _heuristic(heuristic),
      _graph(std::move(graph))
  {
    // Do nothing
  }

private:
  std::size_t _goal;
  const TranslationHeuristic::Storage& _old_items;
  Cache<ShortestPathHeuristic> _heuristic;
  std::shared_ptr<const Supergraph> _graph;
  std::unordered_set<std::size_t> _visited;
};

//==============================================================================
TranslationHeuristic::TranslationHeuristic(
  std::size_t goal,
  std::shared_ptr<const Supergraph> graph,
  CacheManagerPtr<ShortestPathHeuristic> heuristic)
: _goal(goal),
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

  TranslationExpander::SearchQueue queue;
  queue.push(
    std::make_shared<TranslationExpander::Node>(
      TranslationExpander::Node{
        key,
        start_heuristic.value(),
        0.0,
        nullptr
      }));

  TranslationExpander expander{
    _goal,
    old_items,
    std::move(heuristic),
    _graph
  };

  const TranslationExpander::NodePtr solution = a_star_search(expander, queue);
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
TranslationHeuristicFactory::TranslationHeuristicFactory(
    std::shared_ptr<const Supergraph> graph)
  : _graph(std::move(graph)),
    _heuristic_cache(std::make_shared<ShortestPathHeuristicFactory>(_graph))
{
  // Do nothing
}

//==============================================================================
ConstTranslationHeuristicPtr TranslationHeuristicFactory::make(
    const std::size_t goal) const
{
  return std::make_shared<TranslationHeuristic>(
        goal, _graph, _heuristic_cache.get(goal));
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
