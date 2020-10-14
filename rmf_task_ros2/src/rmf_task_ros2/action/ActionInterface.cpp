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

namespace rmf_task_ros2 {
namespace action {

//==============================================================================

std::shared_ptr<TaskActionClient> TaskActionClient::make(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& prefix_topic)
{
  return std::shared_ptr<TaskActionClient>(new 
    TaskActionClient(node, prefix_topic));
}

TaskActionClient::TaskActionClient(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& prefix_topic)
  : _node(node)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_pub = _node->create_publisher<RequestMsg>(
    prefix_topic + "_request", dispatch_qos);

  _status_msg_sub = _node->create_subscription<StatusMsg>(
    prefix_topic + "_status", dispatch_qos,
    [&](const StatusMsg::UniquePtr msg)
    {   
      auto task_profile = convert(msg->task_profile);
      
      // status update mode
      if (_active_task_status.count(task_profile))
      {
        if(auto weak_status = _active_task_status[task_profile].lock())
        {
          *weak_status = convert(*msg);
        }
        else
        {
          std::cout << "weak status has expired\n";
          _active_task_status.erase(task_profile);
        }

        // if active task terminated
        if( (msg->state == StatusMsg::TERMINAL_FAILED) || 
            (msg->state == StatusMsg::TERMINAL_COMPLETED) ||
            (msg->state == StatusMsg::TERMINAL_CANCELED)
        )
        {
          std::cout << "[action] Done Terminated Task: " 
                    << task_profile.task_id << std::endl;
          _active_task_status.erase(task_profile);
        }
      }

      // check if waiting for ack, TODO: think!!!
      if (_task_request_fut_ack.count(task_profile))
      {
        if (msg->state == StatusMsg::TERMINAL_FAILED)
          _task_request_fut_ack[task_profile].set_value(false);
        else
          _task_request_fut_ack[task_profile].set_value(true);
        
        _task_request_fut_ack.erase(task_profile);
      }
    });
}

void TaskActionClient::add_task(
    const std::string& fleet_name, 
    const TaskProfile& task_profile,
    std::future<bool>& add_success,
    TaskStatusPtr status_ptr)
{
   // send request and wait for acknowledgement
  RequestMsg request_msg;
  request_msg.fleet_name = fleet_name;
  request_msg.task_profile = convert(task_profile);
  request_msg.method = RequestMsg::ADD;
  _request_msg_pub->publish(request_msg);

  _task_request_fut_ack[task_profile] = std::promise<bool>();
  add_success = _task_request_fut_ack[task_profile].get_future();
  
  // status
  status_ptr->fleet_name = fleet_name;
  status_ptr->task_profile = task_profile;
  _active_task_status[task_profile] = status_ptr;
  std::cout<< " ~ Add Task: "<< task_profile.task_id << std::endl;
  return;
}

void TaskActionClient::cancel_task(
    const TaskProfile& task_profile,
    std::future<bool>& cancel_success)
{
  // check if task is previously added
  if (!_active_task_status.count(task_profile))
  {
    std::cerr << " ~ Not found Task: "<< task_profile.task_id << std::endl;
    std::promise<bool> prom;
    cancel_success = prom.get_future();
    prom.set_value(false);
    return;
  }

  if(auto weak_status = _active_task_status[task_profile].lock())
  {
    // send cancel and wait for acknowledgement
    RequestMsg request_msg;
    request_msg.fleet_name = weak_status->fleet_name;
    request_msg.task_profile = convert(task_profile);
    request_msg.method = RequestMsg::CANCEL;
    _request_msg_pub->publish(request_msg);

    _task_request_fut_ack[task_profile] = std::promise<bool>();
    cancel_success = _task_request_fut_ack[task_profile].get_future();
    std::cout<< " ~ Cancel Active Task: "<< task_profile.task_id << std::endl;
  }
  else
  {
    std::promise<bool> prom;
    cancel_success = prom.get_future();
    prom.set_value(false);
    std::cout << "weak status is expired, canceled failed \n";
    _active_task_status.erase(task_profile);
  }

  return;
}

int TaskActionClient::size()
{
  for (auto const& status : _active_task_status)
  {
    if(status.second.expired()) //todo iterate rm not clean
      _active_task_status.erase(status.first);
  }

  std::cout << " status: " << _active_task_status.size()
            << " promise: " << _task_request_fut_ack.size() << std::endl;
  return _active_task_status.size();
}

//==============================================================================
//==============================================================================

std::shared_ptr<TaskActionServer> TaskActionServer::make(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name,
    const std::string& prefix_topic)
{
  return std::shared_ptr<TaskActionServer>(new 
    TaskActionServer(node, fleet_name, prefix_topic));
}

TaskActionServer::TaskActionServer(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& fleet_name,
    const std::string& prefix_topic)
  : _node(node), _fleet_name(fleet_name)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_sub = _node->create_subscription<RequestMsg>(
    prefix_topic + "_request", dispatch_qos,
    [&](const RequestMsg::UniquePtr msg)
    {
      if (msg->fleet_name != _fleet_name)
        return; // not me

      std::cout << "[action] Receive a task request!!!"<< std::endl;
      switch(msg->method)
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
    prefix_topic + "_status", dispatch_qos);
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

  if(!_add_task_cb_fn) 
    return;

  if(_add_task_cb_fn(convert(task_profile)))
    status_msg.state = StatusMsg::ACTIVE_QUEUED;
  else
    status_msg.state = StatusMsg::TERMINAL_FAILED;

  _status_msg_pub->publish(status_msg);
}

void TaskActionServer::cancel_task_impl(const TaskProfileMsg& task_profile)
{
  StatusMsg status_msg;
  status_msg.task_profile = task_profile;
  status_msg.fleet_name = _fleet_name;
  
  if(!_cancel_task_cb_fn)
    return;

  if(_cancel_task_cb_fn(convert(task_profile)))
    status_msg.state = StatusMsg::TERMINAL_CANCELED;
  else
    status_msg.state = StatusMsg::TERMINAL_FAILED;

  _status_msg_pub->publish(status_msg);
}

} // namespace action

// ==============================================================================
inline action::TaskStatus convert(const action::StatusMsg& from)
{
  action::TaskStatus status;
  status.fleet_name = from.fleet_name;
  status.task_profile = convert(from.task_profile);
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = (action::TaskStatus::State)from.state;
  return status;
}

// ==============================================================================
inline action::StatusMsg convert(const action::TaskStatus& from)
{
  action::StatusMsg status;
  status.fleet_name = from.fleet_name;
  status.task_profile = convert(from.task_profile);
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = (uint8_t)from.state;
  return status;
}

} // namespace rmf_task_ros2
