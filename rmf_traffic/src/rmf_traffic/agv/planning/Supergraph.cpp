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

//==============================================================================
Eigen::Rotation2Dd compute_forward_offset(
    const Eigen::Vector2d& forward)
{
  return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
}

//==============================================================================
struct TraversalNode
{
  std::size_t finish_waypoint_index;

  Eigen::Vector2d initial_p;
  Eigen::Vector2d finish_p;

  Graph::Lane::EventPtr entry_event;
  Graph::Lane::EventPtr exit_event;

  // TODO(MXG): Can std::string_view be used to make this more memory efficient?
  std::unordered_set<std::string> map_names;

  std::array<std::optional<double>, 2> orientations;
  bool standstill = false;
};

//==============================================================================
bool valid_traversal(const TraversalNode& node)
{
  if (node.standstill)
    return true;

  for (const auto orientation : node.orientations)
  {
    if (orientation.has_value())
      return true;
  }

  return false;
}

//==============================================================================
bool satisfied(
    const Eigen::Vector2d p,
    const double orientation,
    const Eigen::Vector2d course_vector,
    const rmf_traffic::agv::Graph::OrientationConstraint* constraint,
    const TraversalGenerator::Kinematics& kin)
{
  if (!constraint)
    return true;

  Eigen::Vector3d position{p.x(), p.y(), orientation};
  constraint->apply(position, course_vector);
  const double diff = rmf_utils::wrap_to_pi(position[2] - orientation);
  if (std::abs(diff) > kin.interpolate.rotation_thresh)
    return false;

  return true;
}

//==============================================================================
void node_to_traversals(
  const TraversalNode& node,
  const TraversalGenerator::Kinematics& kin,
  std::vector<Traversal>& output)
{
  assert(valid_traversal(node));
  Traversal traversal;
  traversal.finish_waypoint_index = node.finish_waypoint_index;

  if (node.entry_event)
    traversal.entry_event = node.entry_event->clone();

  if (node.exit_event)
    traversal.exit_event = node.exit_event->clone();

  if (node.standstill)
  {
    // If the node is a standstill, just add this empty Alternative
    traversal.alternatives[Any] = Traversal::Alternative{};

    // If the node is a standstill, it should't have any orientation
    // requirements
    assert(!node.orientations[Forward].has_value()
           && !node.orientations[Backward].has_value());
  }

  for (std::size_t i=0; i < 2; ++i)
  {
    const auto orientation = node.orientations[i];

    if (!orientation.has_value())
      continue;

    const Eigen::Vector3d start{
      node.initial_p.x(),
      node.initial_p.y(),
      *orientation
    };

    const Eigen::Vector3d finish{
      node.finish_p.x(),
      node.finish_p.y(),
      *orientation
    };

    const auto start_time = rmf_traffic::Time(std::chrono::seconds(0));
    Trajectory trajectory;
    trajectory.insert(start_time, start, Eigen::Vector3d::Zero());

    internal::interpolate_translation(
          trajectory,
          kin.v_nom,
          kin.a_nom,
          start_time,
          start,
          finish,
          kin.interpolate.translation_thresh);

    Traversal::Alternative alternative;
    for (const auto& map : node.map_names)
      alternative.routes.push_back({map, trajectory});

    traversal.alternatives[i] = std::move(alternative);
  }

  output.emplace_back(std::move(traversal));
}

//==============================================================================
void perform_traversal(
  const TraversalNode* parent,
  const std::size_t lane_index,
  const Graph::Implementation& graph,
  const TraversalGenerator::Kinematics& kin,
  std::vector<TraversalNode>& queue,
  std::vector<Traversal>& output,
  std::unordered_set<std::size_t>& visited)
{
  const auto& lane = graph.lanes[lane_index];
  const auto& entry = lane.entry();
  const auto& exit = lane.exit();
  const std::size_t wp_index_0 = entry.waypoint_index();
  const std::size_t wp_index_1 = exit.waypoint_index();

  if (!visited.insert(wp_index_1).second)
  {
    // If we have already added the finish waypoint to the queue, then there is
    // no need to add it again.
    return;
  }

  const auto& wp0 = graph.waypoints[wp_index_0];
  const auto& wp1 = graph.waypoints[wp_index_1];
  const Eigen::Vector2d p0 = wp0.get_location();
  const Eigen::Vector2d p1 = wp1.get_location();

  TraversalNode node;
  node.finish_waypoint_index = wp_index_1;

  if (parent)
  {
    node.initial_p = parent->initial_p;
    node.map_names = parent->map_names;

    if (entry.event())
    {
      // If this lane has an entry event, then we cannot continue the traversal.
      // A new traversal will have to begin from this waypoint.
      return;
    }
  }
  else
  {
    node.initial_p = p0;

    if (const auto* entry_event = entry.event())
      node.entry_event = entry_event->clone();
  }

  node.finish_p = p1;

  node.map_names.insert(wp0.get_map_name());
  node.map_names.insert(wp1.get_map_name());

  const double dist = (p1 - p0).norm();
  if (!kin.constraint.has_value() || dist < kin.interpolate.translation_thresh)
  {
    if (!parent)
    {
      // These waypoints are effectively on top of each other, and we haven't
      // moved anywhere to arrive here. We will call this a standstill.
      node.standstill = true;
    }
    else
    {
      // These waypoints are effectively on top of each other. We will carry
      // over the orientations of the parent.
      node.standstill = parent->standstill;
      node.orientations = parent->orientations;
    }
  }
  else
  {
    const Eigen::Vector2d course_vector = (p1 - p0)/dist;
    const auto orientations =
        kin.constraint->get_orientations(course_vector);

    for (std::size_t i = 0; i < 2; ++i)
    {
      const auto orientation = orientations[i];
      if (!orientation.has_value())
        continue;

      const auto* entry_constraint = entry.orientation_constraint();
      if (!satisfied(p0, *orientation, course_vector, entry_constraint, kin))
        continue;

      const auto* exit_constraint = exit.orientation_constraint();
      if (!satisfied(p1, *orientation, course_vector, exit_constraint, kin))
        continue;

      if (parent && !parent->standstill)
      {
        const auto parent_orientation = parent->orientations[i];
        if (!parent_orientation.has_value())
          continue;

        const double R_diff = rmf_utils::wrap_to_pi(
              *orientation - *parent_orientation);

        if (std::abs(R_diff) > kin.interpolate.rotation_thresh)
          continue;
      }

      node.orientations[i] = *orientation;
    }
  }

  if (!valid_traversal(node))
  {
    // If this lane has no valid orientations and also does not stand still,
    // then it's not a real traversal, and we should not output it or queue it.
    return;
  }

  const auto* exit_event = exit.event();
  if (exit_event)
    node.exit_event = exit_event->clone();

  // Convert this node into a set of traversals
  node_to_traversals(node, kin, output);

  if (exit_event)
  {
    // If this lane has an exit event, then we need to stop the traversal here,
    // so it does not get added to the queue.
    return;
  }

  queue.push_back(std::move(node));
}

//==============================================================================
void expand_traversal(
  const TraversalNode& parent,
  const std::size_t lane_index,
  const Graph::Implementation& graph,
  const TraversalGenerator::Kinematics& kin,
  std::vector<TraversalNode>& queue,
  std::vector<Traversal>& output,
  std::unordered_set<std::size_t>& visited)
{
  perform_traversal(&parent, lane_index, graph, kin, queue, output, visited);
}

//==============================================================================
void initiate_traversal(
  const std::size_t lane_index,
  const Graph::Implementation& graph,
  const TraversalGenerator::Kinematics& kin,
  std::vector<TraversalNode>& queue,
  std::vector<Traversal>& output,
  std::unordered_set<std::size_t>& visited)
{
  perform_traversal(nullptr, lane_index, graph, kin, queue, output, visited);
}

} // anonymous namespace

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
std::array<std::optional<double>, 2>
DifferentialDriveConstraint::get_orientations(
    const Eigen::Vector2d& course_vector) const
{
  std::array<std::optional<double>, 2> orientations;

  const Eigen::Rotation2Dd R_c(
    std::atan2(course_vector[1], course_vector[0]));
  const Eigen::Rotation2Dd R_h = R_c * R_f_inv;

  orientations[Forward] = rmf_utils::wrap_to_pi(R_h.angle());

  if (reversible)
    orientations[Backward] = rmf_utils::wrap_to_pi((R_pi * R_h).angle());

  return orientations;
}

//==============================================================================
TraversalGenerator::Kinematics::Kinematics(
  const VehicleTraits& traits,
  const Interpolate::Options::Implementation& interpolate_)
: interpolate(interpolate_)
{
  const auto& linear = traits.linear();
  v_nom = linear.get_nominal_velocity();
  a_nom = linear.get_nominal_acceleration();

  if (const auto* diff_drive = traits.get_differential())
  {
    constraint = DifferentialDriveConstraint(
          diff_drive->get_forward(), diff_drive->is_reversible());
  }
}

//==============================================================================
TraversalGenerator::TraversalGenerator(
  std::shared_ptr<const Supergraph> graph,
  const VehicleTraits& traits,
  const Interpolate::Options::Implementation& interpolate)
: _graph(std::move(graph)),
  _kinematics(traits, interpolate)
{
  // Do nothing
}

//==============================================================================
ConstTraversalsPtr TraversalGenerator::generate(
    const std::size_t& key,
    const Storage&,
    Storage& new_items) const
{
  const auto supergraph = _graph.lock();
  if (!supergraph)
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
  const auto& graph = supergraph->original();
  const auto& initial_lanes = graph.lanes_from[waypoint_index];
  std::vector<TraversalNode> queue;
  std::vector<Traversal> output;
  std::unordered_set<std::size_t> visited;
  visited.insert(waypoint_index);

  for (const auto l : initial_lanes)
    initiate_traversal(l, graph, _kinematics, queue, output, visited);

  while (!queue.empty())
  {
    auto top = std::move(queue.back());
    queue.pop_back();

    const auto& lanes = graph.lanes_from[top.finish_waypoint_index];
    for (const auto l : lanes)
      expand_traversal(top, l, graph, _kinematics, queue, output, visited);
  }

  auto new_traversals = std::make_shared<Traversals>(std::move(output));
  new_items.insert({waypoint_index, new_traversals});

  return new_traversals;
}

//==============================================================================
std::shared_ptr<const Supergraph> Supergraph::make(
    Graph::Implementation original,
    const VehicleTraits& traits,
    const Interpolate::Options::Implementation& interpolate)
{
  auto supergraph = std::shared_ptr<Supergraph>(
        new Supergraph(std::move(original)));

  supergraph->_traversals =
      std::make_shared<CacheManager<TraversalCache>>(
        std::make_shared<TraversalGenerator>(supergraph, traits, interpolate));

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
