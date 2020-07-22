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

#ifndef RMF_FLEET_ADAPTER__AGV__TRAFFICLIGHT_HPP
#define RMF_FLEET_ADAPTER__AGV__TRAFFICLIGHT_HPP

#include <memory>
#include <Eigen/Geometry>

#include <rmf_traffic/Time.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TrafficLight
{
public:

  /// A class for updating a traffic light manager about the intentions and
  /// state of its robot.
  class UpdateHandle
  {
  public:

    struct Waypoint
    {
      /// The name of the reference map that the position is on.
      std::string map_name;

      /// A position along the robot's path where it will (or can) have zero
      /// instantaneous velocity. This will usually be a point where the robot
      /// needs to turn, or a point that comes before an intersection, so the
      /// robot can come to a stop and allow other vehicles to pass.
      Eigen::Vector3d position;

      /// A delay that the robot is expected to experience once it arrives at
      /// this waypoint. Usually this will be caused by the robots needing to
      /// wait for doors to open or lifts to arrive.
      rmf_traffic::Duration mandatory_delay = std::chrono::nanoseconds(0);

      /// Whether or not the robot can wait at this point. If passthrough is
      /// true, then the planner will assume that it cannot ask the robot to
      /// wait here. If passthrough is false, the planner may request that the
      /// robot wait here to avoid a conflict with other traffic participants.
      bool passthrough = false;
    };

    /// Update the traffic light with a new path for your robot.
    ///
    /// The traffic light manager is an async system, which means responses for
    /// this update will come at a later time. Due to race conditions, it's
    /// possible to get responses for old path updates that you no longer care
    /// about because you are operating off of a new path now. Save the return
    /// value of this function, and ignore all calls to
    /// CommandHandle::receive_path_timing(~) whose version number is different
    /// from the return value of your latest call to update_path(~).
    ///
    /// This function should be called any time the robot changes its path, and
    /// it may also be a good idea to call it if significant delays or
    /// interruptions have accumulated.
    ///
    /// \param[in] new_path
    ///   Submit a new path that the robot intends to follow.
    std::size_t update_path(
        const std::vector<Waypoint>& new_path);

    /// Call this function if the robot has been delayed along its path.
    ///
    /// \param[in] version
    ///   The version number of the path that has been delayed. This must be
    ///   equal to the return value provided by the update_path(~) function.
    ///
    /// \param[in] delay
    ///   The severity of the delay. This should generally not be negative,
    ///   because you are obligated to have the robot pause any time it is
    ///   getting ahead of its scheduled itinerary.
    void add_delay(
        std::size_t version,
        rmf_traffic::Duration delay);
  };

  using UpdateHandlePtr = std::shared_ptr<UpdateHandle>;

  /// Implement this class to receive traffic light commands from RMF.
  class CommandHandle
  {
  public:

    /// Receive the required timing for a path that has been submitted.
    ///
    /// \param[in] version
    ///   The version number of the path whose timing is being provided. If this
    ///   version number does not match the latest path that you submitted, then
    ///   simply ignore and discard the timing information.
    ///
    /// \param[in] timing
    ///   This is a vector of the earliest times that the vehicle is allowed to
    ///   leave a given waypoint. The entries of this vector have a 1:1 mapping
    ///   with the entries of the path vector that was submitted earlier. Each
    ///   entry represents the earliest time that a robot is allowed to move
    ///   past its corresponding waypoint. If the robot arrives at the waypoint
    ///   before the given time, then the robot is obligated to pause until the
    ///   given time arrives. The robot is allowed to arrive at a waypoint late,
    ///   but UpdateHandle::add_delay(~) should be called whenever that happens.
    virtual void receive_path_timing(
        std::size_t version,
        const std::vector<rmf_traffic::Time>& timing);

  };

  using CommandHandlePtr = std::shared_ptr<CommandHandle>;

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__TRAFFICLIGHT_HPP
