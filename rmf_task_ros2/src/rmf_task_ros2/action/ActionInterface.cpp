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

#include <rmf_task_ros2/action/ActionInterface.hpp>
#include <rmf_task_ros2/StandardNames.hpp>

namespace rmf_task_ros2 {
namespace action {

//==============================================================================

std::shared_ptr<TaskActionClient> TaskActionClient::make(
  std::shared_ptr<rclcpp::Node> node)
{
  return std::shared_ptr<TaskActionClient>(new TaskActionClient(node));
}

TaskActionClient::TaskActionClient(
  std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_pub = _node->create_publisher<RequestMsg>(
    TaskRequestTopicName, dispatch_qos);

  _status_msg_sub = _node->create_subscription<StatusMsg>(
    TaskStatusTopicName, dispatch_qos,
    [&](const StatusMsg::UniquePtr msg)
    {
      auto task_profile = convert(msg->task_profile);

      // status update mode
      if (_active_task_status.count(task_profile))
      {
        auto weak_status = _active_task_status[task_profile].lock();
        
        if (!weak_status)
        {
          std::cout << "weak status has expired\n";
          _active_task_status.erase(task_profile);
        }

        // update status to ptr
        *weak_status = convert(*msg);

        if (_on_change_callback)
          _on_change_callback(weak_status);

        // if active task terminated
        if (weak_status->is_terminated())
        {
          std::cout << "[action] Done Terminated Task: "
                    << task_profile.task_id << std::endl;
          _active_task_status.erase(task_profile);

          if (_on_terminate_callback)
            _on_terminate_callback(weak_status);
        }
      }
  });
}

void TaskActionClient::add_task(
  const std::string& fleet_name,
  const TaskProfile& task_profile,
  TaskStatusPtr status_ptr)
{
  // send request and wait for acknowledgement
  RequestMsg request_msg;
  request_msg.fleet_name = fleet_name;
  request_msg.task_profile = convert(task_profile);
  request_msg.method = RequestMsg::ADD;
  _request_msg_pub->publish(request_msg);

  // save status ptr
  status_ptr->fleet_name = fleet_name;
  status_ptr->task_profile = task_profile;
  _active_task_status[task_profile] = status_ptr;
  std::cout<< " ~ Add Action Task: "<< task_profile.task_id << std::endl;

  return;
}

bool TaskActionClient::cancel_task(
  const TaskProfile& task_profile)
{
  std::cout<< " ~ Cancel Active Task: "<< task_profile.task_id << std::endl;

  // check if task is previously added
  if (!_active_task_status.count(task_profile))
  {
    std::cerr << " ~ Not found Task: "<< task_profile.task_id << std::endl;
    return false;
  }

  auto weak_status = _active_task_status[task_profile].lock();

  if (!weak_status)
  {
    std::cerr << "weak status is expired, canceled failed \n";
    _active_task_status.erase(task_profile);
    return false;
  }

  // send cancel
  RequestMsg request_msg;
  request_msg.fleet_name = weak_status->fleet_name;
  request_msg.task_profile = convert(task_profile);
  request_msg.method = RequestMsg::CANCEL;
  _request_msg_pub->publish(request_msg);
  return true;
}

int TaskActionClient::size()
{
  for (auto it = _active_task_status.begin(); it != _active_task_status.end(); )
  {
    if (auto weak_status = it->second.lock() )
    {
      if (weak_status->is_terminated())
        it = _active_task_status.erase(it);
      else
      {
        std::cout << "[Action Debug] active "
                  << weak_status->task_profile.task_id << "  state: "
                  << (int)weak_status->state  << std::endl;
        ++it;
      }
    }
    else
      it = _active_task_status.erase(it);
  }
  std::cout << " status: " << _active_task_status.size() << std::endl;
  return _active_task_status.size();
}

/// Callback when a task is changed
void TaskActionClient::on_change(StatusCallback status_cb_fn)
{
  _on_change_callback = std::move(status_cb_fn);
}

/// Callback when a task is terminated
void TaskActionClient::on_terminate(StatusCallback status_cb_fn)
{
  _on_terminate_callback = std::move(status_cb_fn);
}

//==============================================================================
//==============================================================================

std::shared_ptr<TaskActionServer> TaskActionServer::make(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& fleet_name)
{
  return std::shared_ptr<TaskActionServer>(new
      TaskActionServer(node, fleet_name));
}

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

void TaskActionServer::register_callbacks(
  AddTaskCallback add_task_cb_fn,
  CancelTaskCallback cancel_task_cb_fn)
{
  _add_task_cb_fn = std::move(add_task_cb_fn);
  _cancel_task_cb_fn = std::move(cancel_task_cb_fn);
}

void TaskActionServer::update_status(const TaskStatus& task_status)
{
  auto msg = convert(task_status);
  msg.fleet_name = _fleet_name;
  _status_msg_pub->publish(msg);
}

//==============================================================================
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
