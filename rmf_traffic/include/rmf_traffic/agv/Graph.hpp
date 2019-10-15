/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef RMF_TRAFFIC__AGV__GRAPH_HPP
#define RMF_TRAFFIC__AGV__GRAPH_HPP

#include <rmf_traffic/Time.hpp>

#include <Eigen/Geometry>

#include <rmf_utils/impl_ptr.hpp>

#include <vector>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Graph
{
public:

  class Waypoint
  {
  public:

    /// Get the name of the map that this Waypoint exists on.
    const std::string& get_map_name() const;
    // TODO(MXG): What should be done for waypoints that "exist" on multiple
    // maps? For example, a waypoint that is in a lift shaft exists on both the
    // map of the floor that it's level with, and the map of the lift that it's
    // inside of.
    //
    // For now, we will solve this by testing trajectories for conflicts on both
    // maps that waypoints claim to belong to, if there is ever a difference
    // between the maps of two connected waypoints.

    /// Set the name of the map that this Waypoint exists on.
    Waypoint& set_map_name(std::string map);

    /// Get the position of this Waypoint
    const Eigen::Vector2d& get_location() const;

    /// Set the position of this Waypoint
    Waypoint& set_location(Eigen::Vector2d location);

    /// Returns true if this Waypoint can be used as a holding point for the
    /// vehicle, otherwise returns false.
    bool is_holding_point() const;

    /// Set whether this waypoint can be used as a holding point for the
    /// vehicle.
    Waypoint& set_holding_point(bool _is_holding_point);

    /// The index of this waypoint within the Graph. This cannot be changed
    /// after the waypoint is created.
    std::size_t index() const;

    class Implementation;
  private:
    Waypoint();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A door in the graph which needs to be opened before a robot can enter any
  /// lane which crosses through it.
  class Door
  {
  public:

    /// Get the unique name (ID) of this Door
    const std::string& get_name() const;

    /// Set the unique name (ID) of this Door
    Door& set_name(std::string name);

    /// Get the delay incurred by waiting for this door to open or close.
    Duration get_delay() const;

    /// Set the delay.
    Door& set_delay(Duration duration);

    /// Get this Door's index within the Graph.
    std::size_t index() const;

    class Implementation;
  private:
    Door();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A lift in the graph which the robot can use to move between floors.
  //
  // TODO(MXG): Design and implement the lift class, and incorporate it into
  // the planner.
  class Lift
  {
  public:

    class Implementation;
  private:
    Lift();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A class that implicitly specifies a constraint on the robot's orientation.
  class OrientationConstraint
  {
  public:

    /// Make an orientation constraint that requires a specific value for the
    /// orientation.
    static std::unique_ptr<OrientationConstraint>
    make(std::vector<double> acceptable_orientations);

    /// Apply the constraint to the given homogeneous position.
    ///
    /// \param[in,out] position
    ///   The position which needs to be constrained. The function should modify
    ///   this position such that it satisfies the constraint, if possible.
    ///
    /// \param[in] course_vector
    ///   The direction that the robot is travelling in. Given for informational
    ///   purposes.
    ///
    /// \return True if the constraint is satisfied with the new value of
    /// position. False if the constraint could not be satisfied.
    virtual bool apply(
        Eigen::Vector3d& position,
        const Eigen::Vector2d& course_vector) const = 0;

    /// Clone this OrientationConstraint.
    virtual std::unique_ptr<OrientationConstraint> clone() const = 0;

    // Default destructor.
    virtual ~OrientationConstraint() = default;
  };

  /// A class that implicitly specifies a constraint on the robot's velocity.
  class VelocityConstraint
  {
  public:

    /// Appl the constraint to the given homogeneous velocity.
    ///
    /// \param[in,out] velocity
    ///   The velocity which needs to be constrained. The function should modify
    ///   this velocity such that it satisfies the constraint, if possible.
    ///
    /// \param[in] course_vector
    ///   The direction that the robot is travelling in. Given for informational
    ///   purposes.
    ///
    /// \return True if the constraint is satisfied with the new value of
    /// velocity. False if the constraint could not be satisfied.
    virtual bool apply(
        Eigen::Vector3d& velocity,
        const Eigen::Vector2d& course_vector) const = 0;

    /// Clone this VelocityConstraint.
    virtual std::unique_ptr<VelocityConstraint> clone() const = 0;

    // Default destructor
    virtual ~VelocityConstraint() = default;
  };

  /// Add a lane to connect two waypoints
  class Lane
  {
  public:

    /// A Lane Node wraps up a Waypoint with constraints. The constraints
    /// stipulate the conditions for entering or exiting the lane to reach this
    /// waypoint.
    class Node
    {
    public:

      // Constructor
      Node(std::size_t waypoint_index,
           std::unique_ptr<OrientationConstraint> orientation = nullptr,
           std::unique_ptr<VelocityConstraint> velocity = nullptr);

      /// Get the index of the waypoint that this Node is wrapped around.
      std::size_t waypoint_index() const;

      /// Get the constraint on orientation that is tied to this Node.
      const OrientationConstraint* orientation_constraint() const;

      /// Get the constraint on velocity that is tied to this Node.
      const VelocityConstraint* velocity_constraint() const;

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// Get the entry node of this Lane. The lane represents an edge in the
    /// graph that goes away from this node.
    const Node& entry() const;

    /// Get the exit node of this Lane. The lane represents an edge in the graph
    /// that goes into this node.
    const Node& exit() const;

    /// Get the index of this Lane within the Graph.
    std::size_t index() const;

    /// Get a pointer to the index of the door that blocks this lane, if one
    /// exists.
    ///
    /// If there is no door along this lane, then return a nullptr.
    // TODO(MXG): Consider replacing this with std::optional<std::size_t> when
    // we can rely on C++17 support.
    const std::size_t* door_index() const;

    class Implementation;
  private:
    Lane();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Make a new waypoint for this graph. It will not be connected to any other
  /// waypoints until you use make_lane() to connect it.
  ///
  /// \note Waypoints cannot be erased from a Graph after they are created. That
  /// feature could be introduced in a later version of this library, but as of
  /// this version, it is not supported. This is to avoid confusion for Lanes if
  /// a Waypoint that they depend on is erased.
  Waypoint& add_waypoint(
      std::string map_name,
      Eigen::Vector2d location,
      bool is_holding_point = false);

  /// Get a waypoint based on its index.
  Waypoint& get_waypoint(std::size_t index);

  /// const-qualified get_waypoint()
  const Waypoint& get_waypoint(std::size_t index) const;

  /// Get the number of waypoints in this Graph
  std::size_t num_waypoints() const;

  /// Make a new door for this Graph.
  Door& add_door(
      std::string name,
      Duration delay = std::chrono::seconds(3));

  /// Get the door at the specified index
  Door& get_door(std::size_t index);

  /// const-qualified get_door()
  const Door& get_door(std::size_t index) const;

  /// Get the number of doors in this Graph
  std::size_t num_doors() const;

  /// Make a lane for this graph. Lanes connect waypoints together, allowing the
  /// graph to know how the robot is allowed to traverse between waypoints.
  Lane& add_lane(
      Lane::Node entry,
      Lane::Node exit);

  /// Make a lane with a door.
  Lane& add_lane(
      Lane::Node entry,
      Lane::Node exit,
      const Door& door);

  // TODO(MXG): Make a lane with a lift. Or perhaps have lifts be a completely
  // different type of graph edge than Lane.
//  Lane& add_lane(
//      Lane::Node entry,
//      Lane::Node exit,
//      const Lift& lift);

  /// Get the number of Lanes in this Graph.
  std::size_t num_lanes() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__GRAPH_HPP
