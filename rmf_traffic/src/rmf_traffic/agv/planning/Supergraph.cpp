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

#include "Supergraph.hpp"

#include <rmf_utils/math.hpp>

#include <unordered_set>

namespace rmf_traffic {
namespace agv {
namespace planning {

namespace {
//==============================================================================
Supergraph::FloorChangeMap find_floor_changes(
    const Graph::Implementation& original)
{
  Supergraph::FloorChangeMap all_floor_changes;

  for (std::size_t i = 0; i < original.waypoints.size(); ++i)
  {
    const auto& initial_map_name = original.waypoints[i].get_map_name();
    auto& floor_changes = all_floor_changes[initial_map_name];

    for (const auto l : original.lanes_from[i])
    {
      const auto& lane = original.lanes[l];
      const auto& exit = original.waypoints[lane.exit().waypoint_index()];
      const auto& final_map_name = exit.get_map_name();
      if (initial_map_name != final_map_name)
        floor_changes[final_map_name].push_back(Supergraph::FloorChange{l});
    }
  }

  return all_floor_changes;
}

} // anonymous namespace

//==============================================================================
Eigen::Rotation2Dd DifferentialDriveConstraint::compute_forward_offset(
    const Eigen::Vector2d& forward)
{
  return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
}

//==============================================================================
const Eigen::Rotation2Dd DifferentialDriveConstraint::R_pi =
  Eigen::Rotation2Dd(M_PI);

//==============================================================================
DifferentialDriveConstraint::DifferentialDriveConstraint(
    const Eigen::Vector2d& forward,
    const bool reversible)
: R_f_inv(compute_forward_offset(forward).inverse()),
  reversible(reversible)
{
  // Do nothing
}

//==============================================================================
std::vector<double> DifferentialDriveConstraint::get_orientations(
    const Eigen::Vector2d& course_vector)
{
  std::vector<double> orientations;
  orientations.reserve(2);

  const Eigen::Rotation2Dd R_c(
    std::atan2(course_vector[1], course_vector[0]));
  const Eigen::Rotation2Dd R_h = R_c * R_f_inv;

  orientations.push_back(rmf_utils::wrap_to_pi(R_h.angle()));

  if (reversible)
    orientations.push_back(rmf_utils::wrap_to_pi((R_pi * R_h).angle()));

  return orientations;
}

//==============================================================================
TraversalGenerator::TraversalGenerator(
  std::shared_ptr<const Supergraph> graph,
  std::optional<DifferentialDriveConstraint> constraint)
: _graph(std::move(graph)),
  _constraint(std::move(constraint))
{
  // Do nothing
}

//==============================================================================
ConstTraversalsPtr TraversalGenerator::generate(
    const std::size_t& key,
    const Storage& old_items,
    Storage& new_items) const
{
  const auto graph = _graph.lock();
  if (!graph)
  {
    // This means the supergraph that's being traversed has destructed while
    // this cache is still alive. That's really weird and shouldn't happen.
    // The only reason we keep the graph as a nullptr is
    // 1) to avoid a circular dependency
    // 2) we cannot technically guarantee that the cache's lifecycle will fit
    //    within the supergraph's lifecycle, and throwing an exception is
    //    preferable to Undefined Behavior.
    throw std::runtime_error(
          "[rmf_traffic::agv::planning::TraversalGenerator::generate] "
          "Supergraph died while a TraversalCache was still being used. "
          "Please report this critical bug to the maintainers of rmf_traffic.");
  }

  const std::size_t waypoint_index = key;
  const auto& initial_lanes = graph->original().lanes_from[waypoint_index];
}

//==============================================================================
std::shared_ptr<const Supergraph> Supergraph::make(
    Graph::Implementation original,
    const rmf_traffic::agv::VehicleTraits* traits)
{
  auto supergraph = std::shared_ptr<Supergraph>(
        new Supergraph(std::move(original)));

  std::optional<DifferentialDriveConstraint> constraint;
  if (traits)
  {
    if (const auto diff = traits->get_differential())
    {
      constraint = DifferentialDriveConstraint(
            diff->get_forward(),
            diff->is_reversible());
    }
  }

  supergraph->_traversals =
      std::make_shared<CacheManager<TraversalCache>>(
        std::make_shared<TraversalGenerator>(
          supergraph, constraint));

  return supergraph;
}

//==============================================================================
const Graph::Implementation& Supergraph::original() const
{
  return _original;
}

//==============================================================================
auto Supergraph::floor_change() const -> const FloorChangeMap&
{
  return _floor_changes;
}

//==============================================================================
TraversalCache Supergraph::traversals() const
{
  return _traversals->get();
}

//==============================================================================
Supergraph::Supergraph(Graph::Implementation original)
: _original(std::move(original)),
  _floor_changes(find_floor_changes(_original))
{
  // Do nothing
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
