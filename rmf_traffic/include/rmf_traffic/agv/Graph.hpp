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

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Graph
{
public:

  class Waypoint
  {
  public:

    const std::string& get_map_name() const;

    Waypoint& set_map_name(std::string map);

    /// Get the position of this Waypoint
    const Eigen::Vector2d& get_location() const;

    /// Set the position of this Waypoint
    Waypoint& set_location(const Eigen::Vector2d& location);

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

    static std::unique_ptr<OrientationConstraint>
    make(double required_orientation);

    virtual bool apply(
        Eigen::Vector3d& position,
        const Eigen::Vector2d& course_vector) const = 0;

    virtual std::unique_ptr<OrientationConstraint> clone() const = 0;

    virtual ~OrientationConstraint() = default;
  };

  /// A door in the graph which needs to be opened before a robot can enter any
  /// lane which crosses through it.
  class Door
  {
  public:

    const std::string& get_name() const;

    Door& set_name(std::string name);

    Duration get_opening_speed() const;

    Door& set_opening_speed(Duration duration);

    std::size_t index() const;

    class Implementation;
  private:
    Door();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A lift in the graph which the robot can use to move between floors.
  class Lift
  {
  public:

    // TODO(MXG): Design and implement the lift class, and incorporate it into
    // the planner.

    class Implementation;
  private:
    Lift();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A class that implicitly specifies a constraint on the robot's velocity.
  class VelocityConstraint
  {
  public:

    virtual bool apply(
        Eigen::Vector3d& velocity,
        const Eigen::Vector2d& course_vector) const = 0;

    virtual std::unique_ptr<VelocityConstraint> clone() const = 0;

    virtual ~VelocityConstraint() = default;
  };

  ///
  class Lane
  {
  public:

    class Node
    {
    public:

      Node(std::size_t waypoint_index,
           std::unique_ptr<OrientationConstraint> orientation = nullptr,
           std::unique_ptr<VelocityConstraint> velocity = nullptr);

      std::size_t waypoint_index() const;

      const OrientationConstraint* orientation_constraint() const;

      const VelocityConstraint* velocity_constraint() const;

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    const Node& entry_node() const;

    const Node& exit_node() const;

    const std::size_t* door_index() const;

    class Implementation;
  private:
    Lane();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Make a new waypoint for this graph. It will not be connected to any other
  /// waypoints until you use make_lane() to connect it.
  Waypoint& make_waypoint(
      std::string map_name,
      Eigen::Vector2d location,
      bool is_holding_point = false);

  /// Get a waypoint based on its index.
  Waypoint& get_waypoint(std::size_t index);

  /// const-qualified get_waypoint()
  const Waypoint& get_waypoint(std::size_t index) const;

  /// Make a new door for this graph.
  Door& make_door(
      std::string name,
      Duration opening_speed = std::chrono::seconds(3));

  /// Get the door at the specified index
  Door& get_door(std::size_t index);

  /// const-qualified get_door()
  const Door& get_door(std::size_t index) const;

  /// Make a lane for this graph. Lanes connect waypoints together, allowing the
  /// graph to know how the robot is allowed to traverse between waypoints.
  Lane& make_lane(
      Lane::Node entry,
      Lane::Node exit);

  /// Make a lane with a door.
  Lane& make_lane(
      Lane::Node entry,
      Lane::Node exit,
      const Door& door);

  // TODO(MXG): make a lane with a lift.
//  Lane& make_lane(
//      Lane::Node entry,
//      Lane::Node exit,
//      const Lift& lift);

};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__GRAPH_HPP
