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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__SUPERGRAPH_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__SUPERGRAPH_HPP

#include "../internal_Graph.hpp"
#include "../internal_Interpolate.hpp"
#include "CacheManager.hpp"

#include <rmf_traffic/Route.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class Supergraph;

//==============================================================================
enum Orientation
{
  Forward = 0,
  Backward,
  Any
};

//==============================================================================
class DifferentialDriveConstraint
{
public:

  static const Eigen::Rotation2Dd R_pi;

  DifferentialDriveConstraint(
    const Eigen::Vector2d& forward,
    const bool reversible);

  std::array<std::optional<double>, 2> get_orientations(
    const Eigen::Vector2d& course_vector) const;

private:
  Eigen::Rotation2Dd R_f_inv;
  bool reversible;
};

//==============================================================================
struct Traversal
{
  std::size_t finish_waypoint_index;
  Graph::Lane::EventPtr entry_event;
  Graph::Lane::EventPtr exit_event;
  double best_time;

  struct Alternative
  {
    std::vector<Route> routes;
  };

  std::array<std::optional<Alternative>, 3> alternatives;
};
using Traversals = std::vector<Traversal>;
using ConstTraversalsPtr = std::shared_ptr<const Traversals>;

//==============================================================================
class TraversalGenerator
    : public Generator<std::unordered_map<std::size_t, ConstTraversalsPtr>>
{
public:

  TraversalGenerator(
    std::shared_ptr<const Supergraph> graph,
    const VehicleTraits& traits,
    const Interpolate::Options::Implementation& interpolate);

  ConstTraversalsPtr generate(
      const std::size_t& key,
      const Storage& old_items,
      Storage& new_items) const final;

  struct Kinematics
  {
    Kinematics(
      const VehicleTraits& traits,
      const Interpolate::Options::Implementation& interpolate);

    double v_nom;
    double a_nom;
    std::optional<DifferentialDriveConstraint> constraint;
    Interpolate::Options::Implementation interpolate;
  };

private:
  std::weak_ptr<const Supergraph> _graph;
  Kinematics _kinematics;
};

//==============================================================================
using TraversalCache = Cache<TraversalGenerator>;

//==============================================================================
/// A Supergraph is derived from a regular Graph. It analyzes the vertices and
/// edges of a regular Graph and adds in new edges and extra information that
/// may be useful for planners. The underlying graph cannot be changed without
/// reconstructing the Supergraph instance.
class Supergraph : public std::enable_shared_from_this<Supergraph>
{
public:

  static std::shared_ptr<const Supergraph> make(
    Graph::Implementation original,
    const VehicleTraits& traits,
    const Interpolate::Options::Implementation& interpolate);

  const Graph::Implementation& original() const;

  struct FloorChange
  {
    std::size_t lane;
  };
  using FloorChanges = std::vector<FloorChange>;
  using DestinationFloorMap = std::unordered_map<std::string, FloorChanges>;
  using FloorChangeMap = std::unordered_map<std::string, DestinationFloorMap>;
  /// Get the lanes that allow the floor to change. This is useful for
  /// identifying the bottlenecks for moving between different maps.
  const FloorChangeMap& floor_change() const;

  /// Get the continuous traversals that can be done from the given waypoint.
  /// This means traversals during which the robot does not need to stop or
  /// rotate.
  TraversalCache traversals() const;

  // TODO(MXG): Consider having a reachability map in this class

private:
  Supergraph(Graph::Implementation original);
  Graph::Implementation _original;
  FloorChangeMap _floor_changes;
  std::shared_ptr<const CacheManager<TraversalCache>> _traversals;
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__SUPERGRAPH_HPP
