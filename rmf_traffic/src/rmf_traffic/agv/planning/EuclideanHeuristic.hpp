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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__EUCLIDEANHEURISTIC_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__EUCLIDEANHEURISTIC_HPP

#include "CacheManager.hpp"
#include "../Supergraph.hpp"
#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// The EuclideanHeuristic helps with computing a Euclidean distance heuristic
/// for the nav graph search, but with a twist: When calculating the heuristic
/// for a goal waypoint that is on a different map, it will perform a search to
/// figure out which choice of floor-changing lane will allow for the shortest
/// Euclidean distance traveling.
class EuclideanHeuristic
    : public Generator<std::unordered_map<std::size_t, std::optional<double>>>
{
public:

  EuclideanHeuristic(
      std::size_t goal,
      double max_speed,
      std::shared_ptr<const Supergraph> graph);

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
};

//==============================================================================
using ConstEuclideanHeuristicPtr = std::shared_ptr<const EuclideanHeuristic>;

//==============================================================================
class EuclideanHeuristicFactory : public Factory<EuclideanHeuristic>
{
public:

  using Generator = EuclideanHeuristic;

  EuclideanHeuristicFactory(
    std::shared_ptr<const Supergraph> graph,
    double max_speed);

  ConstEuclideanHeuristicPtr make(const std::size_t goal) const final;

private:
  std::shared_ptr<const Supergraph> _graph;
  double _max_speed;
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__EUCLIDEANHEURISTIC_HPP
