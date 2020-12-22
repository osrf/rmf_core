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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__SHORTESTPATHHEURISTIC_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__SHORTESTPATHHEURISTIC_HPP

#include "CacheManager.hpp"
#include "../Supergraph.hpp"

#include "EuclideanHeuristic.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// The ShortestPathHeuristic finds the shortest (in terms of time required)
/// path between two waypoints, not accounting for acceleration, deceleration,
/// turning, or orientation constraints.
class ShortestPathHeuristic
    : public Generator<std::unordered_map<std::size_t, std::optional<double>>>
{
public:

  ShortestPathHeuristic(
      std::size_t goal,
      double max_speed,
      std::shared_ptr<const Supergraph> graph,
      CacheManagerPtr<EuclideanHeuristic> heuristic);

  std::optional<double> generate(
      const std::size_t& key,
      const Storage& old_items,
      Storage& new_items) const final;

private:
  std::size_t _goal;
  Eigen::Vector2d _goal_p;
  const std::string* _goal_map;
  double _max_speed;
  std::shared_ptr<const Supergraph> _graph;
  CacheManagerPtr<EuclideanHeuristic> _heuristic;
};

//==============================================================================
using ConstShortestPathHeuristicPtr =
  std::shared_ptr<const ShortestPathHeuristic>;

//==============================================================================
class ShortestPathHeuristicFactory : public Factory<ShortestPathHeuristic>
{
public:

  using Generator = ShortestPathHeuristic;

  ShortestPathHeuristicFactory(
      std::shared_ptr<const Supergraph> graph,
      double max_speed);

  ConstShortestPathHeuristicPtr make(const std::size_t goal) const final;

private:
  std::shared_ptr<const Supergraph> _graph;
  double _max_speed;
  EuclideanHeuristicCacheMap _heuristic_cache;
};


} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__SHORTESTPATHHEURISTIC_HPP
