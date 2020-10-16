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

#ifndef SRC__RMF_TASK_ROS2__ACTION_INTERFACE_HPP
#define SRC__RMF_TASK_ROS2__ACTION_INTERFACE_HPP

#include <rclcpp/node.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/TaskProfile.hpp>
#include <rmf_traffic/Time.hpp>


namespace rmf_task_ros2 {
namespace action {

//==============================================================================
// todo: use template?
// template<typename RequestMsg, typename StatusMsg>

//==============================================================================
// Task Action Client -- responsible of initiating a rmf task to the server. A
// status update will be provided to the client when the task progresses.
// Termination will be triggered when the task ends.

class TaskActionClient
{
public:

  /// Initialize action client
  ///
  /// \param[in] ros2 node instance
  static std::shared_ptr<TaskActionClient> make(
    std::shared_ptr<rclcpp::Node> node);

  /// Add a task to a targetted server
  ///
  /// \param[in] fleet_name
  ///   Target server which will execute this task
  ///
  /// \param[in] task_profile
  ///   Task Description which will be executed
  ///
  /// \param[out] status_ptr
  ///   Will update the status of the task here
  ///
  /// \return bool which indicate if add task is success
  std::future<bool> add_task(
    const std::string& fleet_name,
    const TaskProfile& task_profile,
    TaskStatusPtr status_ptr);

  /// Cancel an added task
  ///
  /// \param[in] task_profile
  ///   Task which to cancel
  /// \return bool which indicate if cancel task is success
  std::future<bool> cancel_task(
    const TaskProfile& task_profile);

  /// Get the number of active task being track by client
  ///
  /// \return number of active task
  int size();

  /// Callback Function which will trigger during an event
  ///
  /// \param[in] status
  ///   Status of a task which the event is triggered
  using StatusCallback = std::function<void(const TaskStatusPtr status)>;

  /// Callback when a task status has changed
  ///
  /// \param[in] status_cb_fn
  ///   Status callback function
  void on_change(StatusCallback status_cb_fn);

  /// Callback when a task is terminated
  ///
  /// \param[in] status_cb_fn
  ///   Status callback function
  void on_terminate(StatusCallback status_cb_fn);

private:
  std::shared_ptr<rclcpp::Node> _node;

  rclcpp::Publisher<RequestMsg>::SharedPtr _request_msg_pub;
  rclcpp::Subscription<StatusMsg>::SharedPtr _status_msg_sub;

  StatusCallback _on_change_callback;
  StatusCallback _on_terminate_callback;

  // Task Tracker
  using TaskStatusWeakPtr = std::weak_ptr<TaskStatus>;
  std::map<TaskProfile, std::promise<bool>> _task_request_fut_ack;
  std::map<TaskProfile, TaskStatusWeakPtr> _active_task_status;

  // Private Constructor
  TaskActionClient(
    std::shared_ptr<rclcpp::Node> node);

};

// ==============================================================================
// Task Action Server - responsible for listening to request, and execute
// incoming task which is being requested by the client. "fleet_name" is key
// to differentiate between multiple action servers.

class TaskActionServer
{
public:

  /// initialize action server
  ///
  /// \param[in] ros2 node instance
  /// \param[in] action server id (e.g. fleet adapter name)
  static std::shared_ptr<TaskActionServer> make(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name);

  using AddTaskCallback =
    std::function<bool(const TaskProfile& task_profile)>;
  using CancelTaskCallback =
    std::function<bool(const TaskProfile& task_profile)>;

  /// Add event callback fns
  ///
  /// \param[in] When a add task is called
  /// \param[in] when a cancel task is called
  void register_callbacks(
    AddTaskCallback add_task_cb_fn,
    CancelTaskCallback cancel_task_cb_fn);

  /// Use this to send a status update to action client
  /// A periodic update is recommended to inform the task progress
  ///
  /// \param[in] task status
  void update_status(
    const TaskStatus& task_status);

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::string _fleet_name;
  AddTaskCallback _add_task_cb_fn;
  CancelTaskCallback _cancel_task_cb_fn;

  rclcpp::Subscription<RequestMsg>::SharedPtr _request_msg_sub;
  rclcpp::Publisher<StatusMsg>::SharedPtr _status_msg_pub;

  // Private Constructor
  TaskActionServer(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name);

  void add_task_impl(const TaskProfileMsg& task_profile);

  void cancel_task_impl(const TaskProfileMsg& task_profile);
};

} // namespace action
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__ACTION_INTERFACE_HPP
