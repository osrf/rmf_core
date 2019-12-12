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
#include <rmf_utils/clone_ptr.hpp>

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

  /// A class that implicitly specifies a constraint on the robot's orientation.
  class OrientationConstraint
  {
  public:

    /// Make an orientation constraint that requires a specific value for the
    /// orientation.
    static rmf_utils::clone_ptr<OrientationConstraint>
    make(std::vector<double> acceptable_orientations);

    enum class Direction
    {
      Forward,
      Backward,
    };

    /// Make an orientation constraint that requires the vehicle to face forward
    /// or backward.
    static rmf_utils::clone_ptr<OrientationConstraint>
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
    virtual rmf_utils::clone_ptr<OrientationConstraint> clone() const = 0;

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

    /// Apply the constraint to the given homogeneous velocity.
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

    /// A door in the graph which needs to be opened before a robot can enter a
    /// certain lane or closed before the robot can exit the lane.
    class Door
    {
    public:

      /// Constructor
      ///
      /// \param[in] name
      ///   Unique name of the door.
      ///
      /// \param[in] duration
      ///   How long the door takes to open or close.
      Door(std::string name, Duration duration);

      /// Get the unique name (ID) of this Door
      const std::string& name() const;

      /// Set the unique name (ID) of this Door
      Door& name(std::string name);

      /// Get the duration incurred by waiting for this door to open or close.
      Duration duration() const;

      /// Set the duration incurred by waiting for this door to open or close.
      Door& duration(Duration duration);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    class DoorOpen : public Door { public: using Door::Door; };
    class DoorClose : public Door { public: using Door::Door; };

    /// A lift door in the graph which needs to be opened before a robot can
    /// enter a certain lane or closed before the robot can exit the lane.
    class LiftDoor
    {
    public:

      /// Constructor
      ///
      /// \param[in] lift_name
      ///   Name of the lift that this door belongs to.
      ///
      /// \param[in] floor_name
      ///   Name of the floor that this door belongs to.
      ///
      /// \param[in] duration
      ///   How long the door takes to open or close.
      LiftDoor(
          std::string lift_name,
          std::string floor_name,
          Duration duration);

      /// Get the name of the lift that the door belongs to
      const std::string& lift_name() const;

      /// Set the name of the lift that the door belongs to
      LiftDoor& lift_name(std::string name);

      /// Get the name of the floor that this door is on
      const std::string& floor_name() const;

      /// Set the name of the floor that this door is on
      LiftDoor& floor_name(std::string name);

      /// Get an estimate of how long it will take the door to open or close
      Duration duration() const;

      /// Set an estimate of how long it will take the door to open or close
      LiftDoor& duration(Duration duration);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    class LiftDoorOpen : public LiftDoor { public: using LiftDoor::LiftDoor; };
    class LiftDoorClose : public LiftDoor { public: using LiftDoor::LiftDoor; };

    class LiftMove
    {
    public:

      /// Constructor
      ///
      /// \param[in] lift_name
      ///   Name of the lift that will move.
      ///
      /// \param[in] destination_floor_name
      ///   Name of the floor that will be moved to.
      ///
      /// \param[in] duration
      ///   How long the lift will take to move.
      LiftMove(
          std::string lift_name,
          std::string destination_floor_name,
          Duration duration);

      /// Get the name of the lift that needs to move
      const std::string& lift_name() const;

      /// Set the name of the lift that needs to move
      LiftMove& lift_name(std::string name);

      /// Get the name of the destination floor
      const std::string& destination_floor() const;

      /// Set the name of the destination floor
      LiftMove& destination_floor(std::string name);

      /// Get an estimate of how long the move will take
      Duration duration() const;

      /// Set an estimate for how long the move will take
      LiftMove& duration(Duration d);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    class Dock
    {
    public:

      /// Constructor
      ///
      /// \param[in]
      ///   Name of the dock that will be approached
      ///
      /// \param[in]
      ///   How long the robot will take to dock
      Dock(std::string dock_name,
           Duration duration);

      /// Get the name of the dock
      const std::string& dock_name() const;

      /// Set the name of the dock
      Dock& dock_name(std::string name);

      /// Get an estimate for how long the docking will take
      Duration duration() const;

      /// Set an estimate for how long the docking will take
      Dock& duration(Duration d);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// A customizable Executor that can carry out actions based on which Event
    /// type is present.
    class Executor
    {
    public:

      virtual void execute(const DoorOpen& open) = 0;
      virtual void execute(const DoorClose& close) = 0;
      virtual void execute(const LiftDoorOpen& open) = 0;
      virtual void execute(const LiftDoorClose& close) = 0;
      virtual void execute(const LiftMove& move) = 0;
      virtual void execute(const Dock& dock) = 0;

      virtual ~Executor() = default;
    };

    class Event;
    using EventPtr = rmf_utils::clone_ptr<Event>;

    /// An abstraction for the different kinds of Lane events
    class Event
    {
    public:

      /// An estimate of how long the event will take
      virtual Duration duration() const = 0;

      template<typename DerivedExecutor>
      DerivedExecutor& execute(DerivedExecutor& executor) const
      {
        return static_cast<DerivedExecutor&>(execute(
              static_cast<Executor&>(executor)));
      }

      /// Execute this event
      virtual Executor& execute(Executor& executor) const = 0;

      /// Clone this event
      virtual EventPtr clone() const = 0;

      virtual ~Event() = default;

      static EventPtr make(DoorOpen open);
      static EventPtr make(DoorClose close);
      static EventPtr make(LiftDoorOpen open);
      static EventPtr make(LiftDoorClose close);
      static EventPtr make(LiftMove move);
      static EventPtr make(Dock dock);
    };


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
      /// \param event
      ///   An event that must happen before/after this Node is approached
      ///   (before if it's an entry Node or after if it's an exit Node).
      ///
      /// \param orientation
      ///   Any orientation constraints for moving to/from this Node (depending
      ///   on whether it's an entry Node or an exit Node).
      ///
      /// \param velocity
      ///   Any velocity constraints for moving to/from this Node (depending on
      ///   whether it's an entry Node or an exit Node).
      Node(std::size_t waypoint_index,
           rmf_utils::clone_ptr<Event> event = nullptr,
           rmf_utils::clone_ptr<OrientationConstraint> orientation = nullptr,
           rmf_utils::clone_ptr<VelocityConstraint> velocity = nullptr);

      /// Constructor, event and velocity_constraint parameters will be nullptr
      ///
      /// \param waypoint_index
      ///   The index of the waypoint for this Node
      ///
      /// \param orientation
      ///   Any orientation constraints for moving to/from this Node (depending
      ///   on whether it's an entry Node or an exit Node).
      Node(std::size_t waypoint_index,
           rmf_utils::clone_ptr<OrientationConstraint> orientation);

      /// Get the index of the waypoint that this Node is wrapped around.
      std::size_t waypoint_index() const;

      /// Get a reference to an event that must occur before or after this Node
      /// is visited.
      ///
      /// \note Before if this is an entry node or after if this is an exit node
      const Event* event() const;

      /// Get the constraint on orientation that is tied to this Node.
      const OrientationConstraint* orientation_constraint() const;

      /// Get the constraint on velocity that is tied to this Node.
      ///
      /// \warning This is currently not being used anyway
      //
      // TODO(MXG): Decide if this should be used or thrown away.
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

  /// Make a lane for this graph. Lanes connect waypoints together, allowing the
  /// graph to know how the robot is allowed to traverse between waypoints.
  Lane& add_lane(
      Lane::Node entry,
      Lane::Node exit);

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
