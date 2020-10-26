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

#include <rmf_task_ros2/action/ActionServer.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

namespace rmf_task_ros2 {
namespace action {

//==============================================================================
std::shared_ptr<TaskActionServer> TaskActionServer::make(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& fleet_name)
{
  return std::shared_ptr<TaskActionServer>(new
      TaskActionServer(node, fleet_name));
}

//==============================================================================
void TaskActionServer::register_callbacks(
  AddTaskCallback add_task_cb_fn,
  CancelTaskCallback cancel_task_cb_fn)
{
  _add_task_cb_fn = std::move(add_task_cb_fn);
  _cancel_task_cb_fn = std::move(cancel_task_cb_fn);
}

//==============================================================================
void TaskActionServer::update_status(const TaskStatus& task_status)
{
  auto msg = convert(task_status);
  msg.fleet_name = _fleet_name;
  _status_msg_pub->publish(msg);
}

//==============================================================================
TaskActionServer::TaskActionServer(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& fleet_name)
: _node(node), _fleet_name(fleet_name)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_sub = _node->create_subscription<RequestMsg>(
    TaskRequestTopicName, dispatch_qos,
    [&](const RequestMsg::UniquePtr msg)
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

void TaskActionServer::add_task_impl(const TaskProfileMsg& task_profile)
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

void TaskActionServer::cancel_task_impl(const TaskProfileMsg& task_profile)
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
