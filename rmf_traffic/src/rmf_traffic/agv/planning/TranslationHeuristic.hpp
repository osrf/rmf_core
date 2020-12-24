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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__TRANSLATIONHEURISTIC_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__TRANSLATIONHEURISTIC_HPP

#include "CacheManager.hpp"
#include "Supergraph.hpp"

#include "ShortestPathHeuristic.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// The TranslationHeuristic finds the fastest path between two waypoints when
/// rotation is ignored. It accounts for the translational acceleration
/// and deceleration of the robot.
class TranslationHeuristic
    : public Generator<std::unordered_map<std::size_t, std::optional<double>>>
{
public:

  TranslationHeuristic(
    std::size_t goal,
    std::shared_ptr<const Supergraph> graph,
    CacheManagerPtr<ShortestPathHeuristic> heuristic);

  std::optional<double> generate(
    const std::size_t& key,
    const Storage& old_items,
    Storage& new_items) const final;

private:
  std::size_t _goal;
  std::shared_ptr<const Supergraph> _graph;
  CacheManagerPtr<ShortestPathHeuristic> _heuristic;
};

//==============================================================================
using ConstTranslationHeuristicPtr =
  std::shared_ptr<const TranslationHeuristic>;

//==============================================================================
class TranslationHeuristicFactory : public Factory<TranslationHeuristic>
{
public:

  using Generator = TranslationHeuristic;

  TranslationHeuristicFactory(std::shared_ptr<const Supergraph> graph);

  ConstTranslationHeuristicPtr make(const std::size_t goal) const final;

private:
  std::shared_ptr<const Supergraph> _graph;
  ShortestPathHeuristicCacheMap _heuristic_cache;
};

//==============================================================================
using TranslationHeuristicCacheMap =
  CacheManagerMap<TranslationHeuristicFactory>;

}
}
}

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__TRANSLATIONHEURISTIC_HPP
