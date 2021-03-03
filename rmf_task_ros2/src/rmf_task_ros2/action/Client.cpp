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

#include "Client.hpp"
#include <rmf_traffic_ros2/Time.hpp>
#include "../internal_Description.hpp"

namespace rmf_task_ros2 {
namespace action {

std::shared_ptr<Client>
Client::make(std::shared_ptr<rclcpp::Node> node)
{
  return std::shared_ptr<Client>(new Client(node));
}

//==============================================================================
Client::Client(std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_pub = _node->create_publisher<RequestMsg>(
    DispatchRequestTopicName, dispatch_qos);

  _status_msg_sub = _node->create_subscription<StatusMsg>(
    TaskStatusTopicName, dispatch_qos,
    [&](const std::unique_ptr<StatusMsg> msg)
    {
      const auto task_id = msg->task_profile.task_id;
      // status update, check if task_id is previously known
      if (_active_task_status.count(task_id))
      {
        auto weak_status = _active_task_status[task_id].lock();

        if (!weak_status)
        {
          RCLCPP_INFO(_node->get_logger(), "Task was previously terminated");
          _active_task_status.erase(task_id);
          return;
        }

        // update status to ptr
        update_status_from_msg(weak_status, *msg);

        if (weak_status->is_terminated())
          RCLCPP_INFO(_node->get_logger(),
          "Received status from fleet [%s], task [%s] is now terminated",
          msg->fleet_name.c_str(), task_id.c_str());

        update_task_status(weak_status);
      }
      else
      {
        /// This is when the task_id is unknown to the dispatcher node. Here
        /// we will make and add the self-generated task from the fleet
        /// adapter to the dispatcher queue (e.g. ChargeBattery Task)
        RCLCPP_DEBUG(_node->get_logger(),
        "[action] Unknown task: [%s]", task_id.c_str());

        const auto& desc_msg = msg->task_profile.description;
        const auto desc = Description::make_description(
          rmf_traffic_ros2::convert(desc_msg.start_time),
          desc_msg.task_type.type,
          desc_msg.priority.value);

        const auto status_ptr = TaskStatus::make(
          task_id, std::chrono::steady_clock::now(), desc);
        update_status_from_msg(status_ptr, *msg);

        _active_task_status[task_id] = status_ptr;
        update_task_status(status_ptr);
      }
    });

  _ack_msg_sub = _node->create_subscription<AckMsg>(
    DispatchAckTopicName, dispatch_qos,
    [&](const std::unique_ptr<AckMsg> msg)
    {
      const auto task_id = msg->dispatch_request.task_profile.task_id;
      const auto weak_status = _active_task_status[task_id].lock();

      switch (msg->dispatch_request.method)
      {
        case RequestMsg::ADD:
          if (msg->success)
          {
            // update this as pending
            RCLCPP_INFO(_node->get_logger(),
            "Received dispatch ack from fleet [%s] that task [%s] is queued",
            msg->dispatch_request.fleet_name.c_str(), task_id.c_str());
            weak_status->state = TaskStatus::State::Queued;
          }
          else
          {
            // update this as failed
            RCLCPP_ERROR(_node->get_logger(),
            "Received dispatch ack from fleet [%s] that task [%s] Add Failed",
            msg->dispatch_request.fleet_name.c_str(), task_id.c_str());
            weak_status->state = TaskStatus::State::Failed;
          }
          break;
        case RequestMsg::CANCEL:
          // update this as Canceled
          if (msg->success)
          {
            RCLCPP_INFO(_node->get_logger(),
            "Received dispatch ack from fleet [%s] that task [%s] is canceled",
            msg->dispatch_request.fleet_name.c_str(), task_id.c_str());
            weak_status->state = TaskStatus::State::Canceled;
          }
          break;
        default:
          RCLCPP_ERROR(_node->get_logger(), "Invalid Dispatch ack method");
          return;
      }

      update_task_status(weak_status);
    });
}

//==============================================================================
void Client::update_task_status(const TaskStatusPtr status)
{
  // call on_change callback
  if (_on_change_callback)
    _on_change_callback(status);

  // erase terminated task and call on_terminate callback
  if (status->is_terminated())
  {
    _active_task_status.erase(status->task_id());
    if (_on_terminate_callback)
      _on_terminate_callback(status);
  }
}

//==============================================================================
void Client::dispatch_task(
  const std::string& fleet_name,
  TaskStatusPtr status_ptr)
{
  rmf_task_msgs::msg::TaskProfile task_profile;
  task_profile.task_id = status_ptr->task_id();
  task_profile.description = description::convert(status_ptr->description());
  task_profile.submission_time =
    rmf_traffic_ros2::convert(status_ptr->submission_time());

  // send request and wait for acknowledgement
  RequestMsg request_msg;
  request_msg.method = RequestMsg::ADD;
  request_msg.fleet_name = fleet_name;
  request_msg.task_profile = task_profile;
  _request_msg_pub->publish(request_msg);

  // save status ptr
  status_ptr->fleet_name = fleet_name;
  _active_task_status[status_ptr->task_id()] = status_ptr;
  RCLCPP_DEBUG(_node->get_logger(), "Assign task: [%s] to fleet [%s]",
    status_ptr->task_id().c_str(), fleet_name.c_str());
  return;
}

//==============================================================================
bool Client::cancel_task(const std::string& task_id)
{
  RCLCPP_DEBUG(_node->get_logger(),
    "[action] Cancel Task: [%s]", task_id.c_str());

  // check if task is previously added
  if (!_active_task_status.count(task_id))
  {
    RCLCPP_WARN(_node->get_logger(),
      "Canceling an unknown task [%s]", task_id.c_str());
    return false;
  }

  auto weak_status = _active_task_status[task_id].lock();

  if (!weak_status)
  {
    RCLCPP_WARN(_node->get_logger(), "Task was previously terminated");
    _active_task_status.erase(task_id);
    return false;
  }

  rmf_task_msgs::msg::TaskProfile task_profile;
  task_profile.task_id = weak_status->task_id();
  task_profile.description = description::convert(weak_status->description());
  task_profile.submission_time =
    rmf_traffic_ros2::convert(weak_status->submission_time());

  // send cancel
  RequestMsg request_msg;
  request_msg.method = RequestMsg::CANCEL;
  request_msg.fleet_name = weak_status->fleet_name;
  request_msg.task_profile = task_profile;
  _request_msg_pub->publish(request_msg);
  return true;
}

//==============================================================================
int Client::size()
{
  for (auto it = _active_task_status.begin(); it != _active_task_status.end(); )
  {
    if (auto weak_status = it->second.lock() )
    {
      if (weak_status->is_terminated())
        it = _active_task_status.erase(it);
      else
        ++it;
    }
    else
      it = _active_task_status.erase(it);
  }
  return _active_task_status.size();
}

//==============================================================================
void Client::on_change(
  StatusCallback status_cb_fn)
{
  _on_change_callback = std::move(status_cb_fn);
}

//==============================================================================
void Client::on_terminate(
  StatusCallback status_cb_fn)
{
  _on_terminate_callback = std::move(status_cb_fn);
}

} // namespace action

//==============================================================================
void update_status_from_msg(
  const TaskStatusPtr task_status_ptr,
  const StatusMsg msg)
{
  task_status_ptr->fleet_name = msg.fleet_name;
  task_status_ptr->start_time = rmf_traffic_ros2::convert(msg.start_time);
  task_status_ptr->end_time = rmf_traffic_ros2::convert(msg.end_time);
  task_status_ptr->robot_name = msg.robot_name;
  task_status_ptr->status = msg.status;
  task_status_ptr->state = static_cast<TaskStatus::State>(msg.state);
}

// ==============================================================================
StatusMsg convert_status(const TaskStatus& from)
{
  StatusMsg status;
  status.fleet_name = from.fleet_name;
  status.task_id = from.task_id();  // duplication
  status.task_profile.task_id = from.task_id();
  status.task_profile.submission_time =
    rmf_traffic_ros2::convert(from.submission_time());
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = static_cast<uint32_t>(from.state);

  if (from.description())
    status.task_profile.description = description::convert(from.description());

  return status;
}

} // namespace rmf_task_ros2
