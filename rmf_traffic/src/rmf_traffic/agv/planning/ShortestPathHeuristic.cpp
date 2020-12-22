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

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
ShortestPathHeuristic::ShortestPathHeuristic(
    std::size_t goal,
    double max_speed,
    std::shared_ptr<const Supergraph> graph,
    ConstEuclideanHeuristicPtr heuristic)
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
  static_assert(false, "Implement this");
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
