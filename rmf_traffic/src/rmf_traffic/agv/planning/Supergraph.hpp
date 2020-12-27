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
#include "DifferentialDriveMap.hpp"

#include <rmf_traffic/Route.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <map>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class Supergraph;

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
  std::size_t initial_lane_index;
  std::size_t finish_lane_index;
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
    const std::shared_ptr<const Supergraph>& graph,
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

  // TODO(MXG): We could consider moving some of this class's nested classes out
  // into the global scope to clean up the API here.

  static std::shared_ptr<const Supergraph> make(
    Graph::Implementation original,
    VehicleTraits traits,
    const Interpolate::Options::Implementation& interpolate);

  const Graph::Implementation& original() const;
  const VehicleTraits& traits() const;

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

  struct Entry
  {
    std::size_t lane;
    Orientation orientation;
  };

  class Entries
  {
  public:

    std::vector<Entry> relevant_entries(
        std::optional<double> orientation) const;

    Entries(
        std::map<double, Entry> angled_entries,
        std::vector<Entry> agnostic_entries);

  private:
    // These are entries that require a specific entry angle
    std::map<double, Entry> _angled_entries;

    // These are entries that do not require any specific entry angle
    std::vector<Entry> _agnostic_entries;

    std::size_t _total_entries;
  };
  using ConstEntriesPtr = std::shared_ptr<const Entries>;

  ConstEntriesPtr entries_into(std::size_t waypoint_index) const;

  /// Get the keys for the DifferentialDriveHeuristic cache entries that are
  /// relevant for a given combination of start and goal conditions.
  std::vector<DifferentialDriveMapTypes::Key> keys_for(
      std::size_t start_waypoint_index,
      std::size_t goal_waypoint_index,
      std::optional<double> goal_orientation) const;

private:
  Supergraph(Graph::Implementation original, VehicleTraits traits);
  Graph::Implementation _original;
  VehicleTraits _traits;
  FloorChangeMap _floor_changes;
  std::shared_ptr<const CacheManager<TraversalCache>> _traversals;

  class EntriesGenerator
      : public Generator<std::unordered_map<std::size_t, ConstEntriesPtr>>
  {
  public:
    EntriesGenerator(
      const std::shared_ptr<const Supergraph>& graph,
      const Interpolate::Options::Implementation& interpolate);

    ConstEntriesPtr generate(
      const std::size_t& key,
      const Storage& old_items,
      Storage& new_items) const final;

  private:
    std::weak_ptr<const Supergraph> _graph;
    Interpolate::Options::Implementation _interpolate;
    std::optional<DifferentialDriveConstraint> _constraint;
  };
  using EntriesCache = Cache<EntriesGenerator>;
  std::shared_ptr<const CacheManager<EntriesCache>> _entries_cache;
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__SUPERGRAPH_HPP
