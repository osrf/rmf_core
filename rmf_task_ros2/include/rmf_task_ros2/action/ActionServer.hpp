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

#ifndef SRC__RMF_TASK_ROS2__ACTION_SERVER_HPP
#define SRC__RMF_TASK_ROS2__ACTION_SERVER_HPP

#include <rclcpp/node.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/TaskStatus.hpp>
#include <rmf_traffic/Time.hpp>

using TaskProfile = rmf_task_msgs::msg::TaskProfile;

namespace rmf_task_ros2 {
namespace action {

// ==============================================================================
// Task Action Server - responsible for listening to request, and execute
// incoming task which is being requested by the client. "fleet_name" is key
// to differentiate between multiple action servers.

template<typename RequestMsg, typename StatusMsg>
class TaskActionServer
{
public:
  /// initialize action server
  ///
  /// \param[in] node
  ///   ros2 node instance
  ///
  /// \param[in] fleet_name
  ///   action server id (e.g. fleet adapter name)
  static std::shared_ptr<TaskActionServer> make(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name);
  using AddTaskCallback =
    std::function<bool(const TaskProfile& task_profile)>;
  using CancelTaskCallback =
    std::function<bool(const TaskProfile& task_profile)>;

  /// Add event callback functions. These functions will be triggered when a
  /// requests msg is received
  ///
  /// \param[in] add_task_cb
  ///   When an add task request is called
  ///
  /// \param[in] cancel_task_cb
  ///   when a cancel task request is called
  void register_callbacks(
    AddTaskCallback add_task_cb,
    CancelTaskCallback cancel_task_cb);

  /// Use this to send a status update to action client
  /// A On Change update is recommended to inform the task progress
  ///
  /// \param[in] task_status
  ///   latest status of the task
  void update_status(
    const TaskStatus& task_status);

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::string _fleet_name;
  AddTaskCallback _add_task_cb_fn;
  CancelTaskCallback _cancel_task_cb_fn;

  typename rclcpp::Subscription<RequestMsg>::SharedPtr _request_msg_sub;
  typename rclcpp::Publisher<StatusMsg>::SharedPtr _status_msg_pub;

  // Private Constructor
  TaskActionServer(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name);
};

} // namespace action
} // namespace rmf_task_ros2

#include "internal_ActionServer.tpp"

#endif // SRC__RMF_TASK_ROS2__ACTION_SERVER_HPP
