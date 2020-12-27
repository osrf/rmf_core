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

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
