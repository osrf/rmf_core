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

    /// Constructor
    ///
    /// \param[in] name
    ///   Unique name of the door.
    ///
    /// \param[in] delay
    ///   How long the door takes to open. This will also be used for how long
    ///   the door takes to close, so the total delay incurred by opening and
    ///   closing will be 2*delay.
    Door(std::string name, Duration delay);

    /// Constructor
    ///
    /// \param[in] name
    ///   Unique name of the door.
    ///
    /// \param[in] open_delay
    ///   How long the door takes to open.
    ///
    /// \param[in] close_delay
    ///   How long the door takes to close.
    Door(std::string name, Duration open_delay, Duration close_delay);

    /// Get the unique name (ID) of this Door
    const std::string& name() const;

    /// Set the unique name (ID) of this Door
    Door& name(std::string name);

    /// Get the delay incurred by waiting for this door to open.
    Duration open_delay() const;

    /// Set the delay incurred by waiting for this door to open.
    Door& open_delay(Duration duration);

    /// Get the delay incurred by waiting for this door to close.
    Duration close_delay() const;

    /// Get the delay incurred by waiting for this door to close.
    Door& close_delay(Duration duration);

    /// Get the total delay incurred by waiting for this door to open and then
    /// close.
    Duration total_delay() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A door in the graph which belongs to a lift. The lift needs to be summoned
  /// to this floor before the robot can proceed.
  ///
  /// \note The planner will naively assume that the lift is already on the
  /// desired floor and ready to be opened right away.
  ///
  // TODO(MXG): If we can have a centralized lift schedule we can make better
  // predictions and more accurate plans.
  class LiftDoor
  {
  public:

    /// Constructor
    ///
    /// \param[in] floor_name
    ///   Name of the floor
    ///
    /// \param[in] door_info
    ///   Information about the door
    LiftDoor(std::string floor_name, Door door_info);

    /// Get the door information of this LiftDoor
    const Door& door() const;

    /// Set the door information of this LiftDoor
    LiftDoor& door(Door door_info);

    /// Get the name of the floor that this LiftDoor is on
    const std::string& floor_name() const;

    /// Set the name of the floor that this LiftDoor is on
    LiftDoor& floor_name(std::string name);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The lane traverses up or down a lift.
  class LiftShaft
  {
  public:

    /// Constructor
    ///
    /// \param[in] destination_floor_name
    ///   Name of the floor that the lane will take the robot to.
    ///
    /// \param[in] delay
    ///   How long the robot will take to get to the destination floor.
    LiftShaft(std::string destination_floor_name, Duration delay);

    /// Get the name of the destination floor.
    const std::string& destination_floor_name() const;

    /// Set the name of the destination floor.
    LiftShaft& destination_floor_name(std::string name);

    /// Get how long the lift will take to move between the floors.
    Duration delay() const;

    /// Set how long the lift will take to move between the floors.
    LiftShaft& delay(Duration duration);

    class Implementation;
  private:
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

    enum class Direction
    {
      Forward,
      Backward,
    };

    static std::unique_ptr<OrientationConstraint>
    make(Direction direction, const Eigen::Vector2d& forward_vector);

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
  //
  // TODO(MXG): This class is not currently being used while planning. Remember
  // to add this feature later, or else delete this API.
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

      /// Constructor
      ///
      /// \param waypoint_index
      ///   The index of the waypoint for this Node
      ///
      /// \param orientation
      ///   Any orientation constraints for moving to/from this Node (depending
      ///   on whether it's an entry Node or an exit Node).
      ///
      /// \param velocity
      ///   Any velocity constraints for moving to/from this Node (depending on
      ///   whether it's an entry Node or an exit Node).
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

    class Feature
    {
    public:



      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// Get the index of this Lane within the Graph.
    std::size_t index() const;

    /// This exception gets thrown when a user tries to change a Lane in a way
    /// that does not make sense. For example if a lane is given both a Door
    /// and a LiftDoor. Right now we only support having one Lane feature at a
    /// time.
    ///
    /// To avoid getting this error, be sure to remove any feature
    class bad_feature_change : std::logic_error
    {

    };

    /// Get a reference to the door that blocks this lane, if one exists.
    ///
    /// If there is no door along this lane, then return a nullptr.
    const Door* get_door() const;

    /// Set the door for this lane
    Lane& set_door(Door door);

    /// Remove the door for this lane
    Lane& remove_door();


    Lane& clear_feature();

    class Implementation;
  private:
    Lane();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Default constructor
  Graph();

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

  // TODO(MXG): Allow waypoints to have keynames so that they can be gotten by
  // a string value instead of an index value.

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
      Door door);

  /// Get the lane at the specified index
  Lane& get_lane(std::size_t index);

  /// const-qualified get_lane()
  const Lane& get_lane(std::size_t index) const;

  /// Get the number of Lanes in this Graph.
  std::size_t num_lanes() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__GRAPH_HPP
