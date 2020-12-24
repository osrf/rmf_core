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

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
DifferentialDriveHeuristic::DifferentialDriveHeuristic(
  std::size_t goal,
  std::shared_ptr<const Supergraph> graph,
  CacheManagerPtr<TranslationHeuristic> heuristic)
: _goal(goal),
  _graph(std::move(graph)),
  _heuristic(std::move(heuristic))
{
  // Do nothing
}

//==============================================================================
auto DifferentialDriveHeuristic::generate(
  const Key& key,
  const Storage& old_items,
  Storage& new_items) const -> NodePtr
{

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
        goal, _graph, _heuristic_cache.get(goal));
}

//==============================================================================
auto DifferentialDriveHeuristicFactory::keys_for(
  std::size_t start_waypoint_index,
  std::size_t goal_waypoint_index,
  std::optional<double> goal_orientation) const -> std::vector<Generator::Key>
{
  using Key = Generator::Key;
  std::vector<Key> keys;

  const auto relevant_entries = _graph->entries_into(goal_waypoint_index)
      .relevant_entries(goal_orientation);

  const auto traversals = _graph->traversals().get(start_waypoint_index);
  assert(traversals);

  for (const auto& traversal : *traversals)
  {
    const std::size_t lane_index = traversal.initial_lane_index;
    for (std::size_t orientation = 0; orientation < 3; ++orientation)
    {
      const auto& alt = traversal.alternatives[orientation];
      if (!alt.has_value())
        continue;

      for (const auto& entry : relevant_entries)
      {
        keys.push_back(
          Key{
            lane_index, Orientation(orientation),
            entry.lane, entry.orientation
          });
      }
    }
  }

  return keys;
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
