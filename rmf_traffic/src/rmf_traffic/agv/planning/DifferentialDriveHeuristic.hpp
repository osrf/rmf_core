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
  using Key = DifferentialDriveMapTypes::Key;

  SolutionNodePtr generate(
    const Key& key,
    const Storage& old_items,
    Storage& new_items) const final;

  static CacheManagerPtr<DifferentialDriveHeuristic> make_manager(
      std::shared_ptr<const Supergraph> graph);

private:
  std::shared_ptr<const Supergraph> _graph;
  CacheManagerMap<TranslationHeuristicFactory> _heuristic_map;
};

//==============================================================================
using ConstDifferentialDriveHeuristicPtr =
  std::shared_ptr<const DifferentialDriveHeuristic>;

//==============================================================================
class DifferentialDriveHeuristicAdapter
{
public:

  DifferentialDriveHeuristicAdapter(
      Cache<DifferentialDriveHeuristic> cache,
      std::shared_ptr<const Supergraph> graph,
      std::size_t goal_waypoint,
      std::optional<double> goal_yaw);

  std::optional<double> compute(
      std::size_t start_waypoint,
      double yaw) const;

  using SolutionNodePtr = DifferentialDriveHeuristic::SolutionNodePtr;
  using Entry = DifferentialDriveHeuristic::Entry;
  using Key = DifferentialDriveHeuristic::Key;

  SolutionNodePtr compute(Entry start) const;

  const Cache<DifferentialDriveHeuristic>& cache() const;

private:
  Cache<DifferentialDriveHeuristic> _cache;
  std::shared_ptr<const Supergraph> _graph;
  std::size_t _goal_waypoint;
  std::optional<double> _goal_yaw;
  double _w_nom;
  double _alpha_nom;
  double _rotation_threshold;
};

//==============================================================================
template<typename NodePtrT>
struct DifferentialDriveCompare
{
  // Returning False implies that a is preferable to b
  // Returning True implies that b is preferable to a
  bool operator()(const NodePtrT& a, const NodePtrT& b)
  {
    // TODO(MXG): Micro-optimization: consider saving the sum of these values
    // in the Node instead of needing to re-add them for every comparison.
    const double a_value = a->get_total_cost_estimate();
    const double b_value = b->get_total_cost_estimate();

    // Note(MXG): The priority queue puts the greater value first, so we
    // reverse the arguments in this comparison.
    if (std::abs(a_value - b_value) > _threshold)
      return b_value < a_value;

    const std::optional<Orientation> a_orientation = a->get_orientation();
    const std::optional<Orientation> b_orientation = b->get_orientation();

    if (b_orientation == Orientation::Forward
        && a_orientation != Orientation::Forward)
    {
      return true;
    }
    else if (a_orientation == Orientation::Forward)
    {
      return false;
    }

    // If the cost estimates are within the threshold and there is no
    // orientation preference, then we'll prefer the one that seems to be closer
    // to the goal.
    return b->get_remaining_cost_estimate() < a->get_remaining_cost_estimate();
  }

private:
  double _threshold;
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEHEURISTIC_HPP
