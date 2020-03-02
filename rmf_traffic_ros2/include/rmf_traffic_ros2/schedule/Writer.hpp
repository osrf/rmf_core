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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__WRITER_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__WRITER_HPP

#include <rmf_traffic/schedule/Writer.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic_msgs/msg/schedule_writer_item.hpp>

#include <rclcpp/node.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
/// The Writer class provides an API that allows a Node to create schedule
/// Participants.
class Writer : std::enable_shared_from_this<Writer>
{
public:

  /// Create an instance of a writer.
  ///
  /// \param[in] node
  ///   The node that will manage the subscriptions of this writer
  static std::shared_ptr<Writer> make(rclcpp::Node& node);

  /// Begin creation of a schedule participant.
  ///
  /// The node of this Writer needs to be spun in order for the Participant to
  /// finish being created.
  ///
  /// \param[in] description.
  ///   The description of the participant.
  std::future<rmf_traffic::schedule::Participant> make_participant(
      rmf_traffic::schedule::ParticipantDescription description);

  class Implementation;
private:
  Writer(rclcpp::Node& node);
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule

//==============================================================================
rmf_traffic::schedule::Writer::Input convert(
    const std::vector<rmf_traffic_msgs::msg::ScheduleWriterItem>& from);

//==============================================================================
std::vector<rmf_traffic_msgs::msg::ScheduleWriterItem> convert(
    const rmf_traffic::schedule::Writer::Input& from);

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__SCHEDULE__WRITER_HPP
