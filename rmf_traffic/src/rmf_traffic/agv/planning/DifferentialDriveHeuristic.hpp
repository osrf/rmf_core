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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEHEURISTIC_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEHEURISTIC_HPP

#include "CacheManager.hpp"
#include "Supergraph.hpp"
#include "DifferentialDriveMap.hpp"

#include "TranslationHeuristic.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class DifferentialDriveHeuristic : public Generator<DifferentialDriveMap>
{
public:

  DifferentialDriveHeuristic(std::shared_ptr<const Supergraph> graph);

  using SolutionNode = DifferentialDriveMapTypes::SolutionNode;
  using SolutionNodePtr = DifferentialDriveMapTypes::SolutionNodePtr;
  using Entry = DifferentialDriveMapTypes::Entry;

  SolutionNodePtr generate(
    const Key& key,
    const Storage& old_items,
    Storage& new_items) const final;

private:
  std::shared_ptr<const Supergraph> _graph;
  CacheManagerMap<TranslationHeuristicFactory> _heuristic_map;
};

//==============================================================================
using ConstDifferentialDriveHeuristicPtr =
  std::shared_ptr<const DifferentialDriveHeuristic>;

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEHEURISTIC_HPP
