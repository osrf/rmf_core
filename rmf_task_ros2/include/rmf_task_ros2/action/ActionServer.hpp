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
#include <rmf_task_ros2/TaskProfile.hpp>
#include <rmf_traffic/Time.hpp>


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
  /// A On Change update is recommended to inform the task progress
  ///
  /// \param[in] task status
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

  void add_task_impl(const TaskProfileMsg& task_profile);

  void cancel_task_impl(const TaskProfileMsg& task_profile);
};

//==============================================================================
//==============================================================================
template<typename RequestMsg, typename StatusMsg>
std::shared_ptr<TaskActionServer<RequestMsg, StatusMsg>>
TaskActionServer<RequestMsg, StatusMsg>::make(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& fleet_name)
{
  return std::shared_ptr<TaskActionServer>(new
      TaskActionServer(node, fleet_name));
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
void TaskActionServer<RequestMsg, StatusMsg>::register_callbacks(
  AddTaskCallback add_task_cb_fn,
  CancelTaskCallback cancel_task_cb_fn)
{
  _add_task_cb_fn = std::move(add_task_cb_fn);
  _cancel_task_cb_fn = std::move(cancel_task_cb_fn);
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
void TaskActionServer<RequestMsg, StatusMsg>::update_status(
  const TaskStatus& task_status)
{
  auto msg = convert(task_status);
  msg.fleet_name = _fleet_name;
  _status_msg_pub->publish(msg);
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
TaskActionServer<RequestMsg, StatusMsg>::TaskActionServer(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& fleet_name)
: _node(node), _fleet_name(fleet_name)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_sub = _node->create_subscription<RequestMsg>(
    TaskRequestTopicName, dispatch_qos,
    [&](const std::unique_ptr<RequestMsg> msg)
    {
      if (msg->fleet_name != _fleet_name)
        return;// not me

      std::cout << "[action] Receive a task request!!!"<< std::endl;
      switch (msg->method)
      {
        case RequestMsg::ADD:
          this->add_task_impl(msg->task_profile);
          break;
        case RequestMsg::CANCEL:
          this->cancel_task_impl(msg->task_profile);
          break;
        default:
          std::cerr << "Request Method is not supported!!!"<< std::endl;
      }
    });

  _status_msg_pub = _node->create_publisher<StatusMsg>(
    TaskStatusTopicName, dispatch_qos);
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
void TaskActionServer<RequestMsg, StatusMsg>::add_task_impl(
  const TaskProfileMsg& task_profile)
{
  StatusMsg status_msg;
  status_msg.task_profile = task_profile;
  status_msg.fleet_name = _fleet_name;

  if (!_add_task_cb_fn)
    return;

  if (_add_task_cb_fn(convert(task_profile)))
    status_msg.state = (uint8_t)TaskStatus::State::Queued;
  else
    status_msg.state = (uint8_t)TaskStatus::State::Failed;

  _status_msg_pub->publish(status_msg);
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
void TaskActionServer<RequestMsg, StatusMsg>::cancel_task_impl(
  const TaskProfileMsg& task_profile)
{
  StatusMsg status_msg;
  status_msg.task_profile = task_profile;
  status_msg.fleet_name = _fleet_name;

  if (!_cancel_task_cb_fn)
    return;

  if (_cancel_task_cb_fn(convert(task_profile)))
    status_msg.state = (uint8_t)TaskStatus::State::Canceled;
  else
    status_msg.state = (uint8_t)TaskStatus::State::Failed;

  _status_msg_pub->publish(status_msg);
}

} // namespace action
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__ACTION_SERVER_HPP
