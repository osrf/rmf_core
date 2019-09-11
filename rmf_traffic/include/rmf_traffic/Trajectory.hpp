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

#ifndef RMF_TRAFFIC__TRAJECTORY_HPP
#define RMF_TRAFFIC__TRAJECTORY_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <memory>
#include <vector>

namespace rmf_traffic {

class Trajectory
{
public:

  class Profile;
  using ProfilePtr = std::shared_ptr<Profile>;
  using ConstProfilePtr = std::shared_ptr<const Profile>;

  /// The Trajectory::Profile class describes how the trajectory will take up
  /// space at a single instant in time. A profile description is assigned to
  /// each segment of a trajectory, so that the profile of a trajectory may
  /// change over time, for example as the robot changes its mode of movement.
  ///
  /// There are two factors to describe the profile:
  ///  * shape - Describes the occupied space
  ///  * movement - Describes how the robot is moving
  ///
  /// For Queued movement, the profile has two additional parameters:
  ///  * queue_id
  ///  * queue_number
  ///
  /// The queue_id indicates which queue the robot will be waiting in. The
  /// queue_number indicates the robot's intended place within that queue. The
  /// value of the queue_number should decrease with each step of the trajectory
  /// until it reaches zero, at which point the robot will be using the resource
  /// that is being queued on.
  class Profile
  {
  public:

    /// The Trajectory::Profile::Movement enum describes how the robot intends
    /// to move while following its trajectory.
    enum Movement {

      /// This movement type is illegal and will always be rejected by the
      /// schedule verifier. Having this movement type implies a major bug in
      /// the code and should be reported immediately.
      Unspecified = 0,

      /// The robot will follow the specified trajectory exactly.
      Strict,

      /// The robot will autonomously navigate within the specified space.
      Autonomous,

      /// The robot is waiting in a queue, and will wait to traverse the
      /// trajectory segment until the rmf_traffic_monitor tells it to proceed.
      Queued,
    };

    // Collision table:
    // |=============================================|
    // | Movement   | Strict  | Autonomous | Queued  |
    // |------------+---------+------------+---------|
    // | Strict     | COLLIDE |   okay     | COLLIDE |
    // |------------+---------+------------+---------|
    // | Autonomous |  okay   |  COLLIDE   |  okay   |
    // |------------+---------+------------+---------|
    // | Queued     | COLLIDE |   okay     |  okay   |
    // |=============================================|


    /// Create a profile with Strict movement
    static ProfilePtr make_strict(geometry::ConstConvexShapePtr shape);

    /// Create a profile with Autonomous movement
    static ProfilePtr make_autonomous(geometry::ConstConvexShapePtr shape);

    /// Create a profile for a Queued segment
    static ProfilePtr make_queued(
        geometry::ConstConvexShapePtr shape,
        const std::string& queue_id,
        uint32_t queue_number);

    /// Get the shape being used for this profile
    geometry::ConstConvexShapePtr get_shape() const;

    /// Set the shape that will be used by this profile
    void set_shape(geometry::ConstConvexShapePtr new_shape);

    /// Get the movement being used for this profile
    Movement get_movement() const;

    /// Set the movement of this profile to Strict
    void set_to_strict();

    /// Set the movement of this profile to Autonomous
    void set_to_autonomous();

    /// Set the movement of this profile to queued
    void set_to_queued(const std::string& queue_id, uint32_t queue_number);

    class QueueInfo
    {
    public:

      /// Get the id of the queue that this profile is waiting in
      std::string get_queue_id() const;

      /// Get the number that this profile has within the queue
      uint32_t get_queue_number() const;

    private:
      QueueInfo(void* pimpl);
      friend class Profile;
      const void* const _pimpl;
    };

    /// If this Profile is queued, this will return a pointer to its queue
    /// information. If it is not in a queue, this will return a nullptr.
    ///
    /// This pointer is invalidated any time a modification is made to the
    /// Profile object that provided it.
    //
    // TODO(MXG): Change this to a std::optional when we can have C++17 support
    const QueueInfo* get_queue_info() const;

  private:

    Profile(geometry::ConstConvexShapePtr shape);

    class Implementation;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  struct Segment
  {
    ConstProfilePtr profile;
    // TODO(MXG): Add waypoints
  };




private:

  class Implementation;
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__TRAJECTORY_HPP
