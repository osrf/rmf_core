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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP

#include <rclcpp/node.hpp>

#include <rmf_traffic/schedule/Negotiator.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
/// A ROS2 interface for negotiating solutions to schedule conflicts
class Negotiation
{
public:

  /// Constructor
  Negotiation(
      rclcpp::Node& node,
      const rmf_traffic::schedule::Viewer& viewer);

  /// Register a negotiator with this Negotiation manager.
  ///
  /// \param[in] for_participant
  ///   The ID of the participant that this negotiator will work for
  ///
  /// \param[in] negotiator
  ///   The negotiator interface to use for this participant
  ///
  /// \return a handle that should be kept by the caller. When this handle
  /// expires, this negotiator will be automatically unregistered.
  std::shared_ptr<void> register_negotiator(
      rmf_traffic::schedule::ParticipantId for_participant,
      std::unique_ptr<rmf_traffic::schedule::Negotiator> negotiator);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__NEGOTIATION_HPP
