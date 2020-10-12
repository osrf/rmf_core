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
      if(_status_callback_fn)
        _status_callback_fn(msg->server_id, msg->tasks);
    });
  
  _result_msg_sub = _node->create_subscription<ResultMsg>(
    prefix_topic + "_result", dispatch_qos,
    [&](const ResultMsg::UniquePtr msg)
    {
      // find task_profile if is waiting for ack list
      auto task_profile = convert(msg->task.task_profile);
      if (_task_request_fut_ack.count(task_profile))
      {
        if (msg->task.state != TaskMsg::TERMINAL_FAILED)
          _task_request_fut_ack[task_profile].set_value(true);
        else
          _task_request_fut_ack[task_profile].set_value(false);
        
        _task_request_fut_ack.erase(task_profile);
      }

      // termination mode
      if(_termination_callback_fn)
      {
        bool is_success = (msg->task.state == TaskMsg::TERMINAL_COMPLETED);
        _termination_callback_fn(msg->server_id, msg->task, is_success);
      }
    });
}

void TaskActionClient::register_callbacks(
    StatusCallback status_callback_fn, 
    TerminationCallback termination_callback_fn)
{
  _status_callback_fn = std::move(status_callback_fn);
  _termination_callback_fn = std::move(termination_callback_fn);
}

void TaskActionClient::add_task(
    const std::string& server_id, 
    const TaskProfile& task_profile,
    std::future<bool>& add_success)
{
  // reinitiaze promise
  _ack_promise = std::promise<bool>(); 
  
  // send request and wait for acknowledgement
  RequestMsg request_msg;
  request_msg.server_id = server_id;
  request_msg.task_profile = convert(task_profile);
  request_msg.method = RequestMsg::ADD;
  _request_msg_pub->publish(request_msg);

  _task_request_fut_ack[task_profile] = std::promise<bool>();
  add_success = _task_request_fut_ack[task_profile].get_future();
  return;
}

void TaskActionClient::cancel_task(
    const std::string& server_id, 
    const TaskProfile& task_profile,
    std::future<bool>& cancel_success)
{
  // reinitiaze promise
  _ack_promise = std::promise<bool>(); 

  // send cancel and wait for acknowledgement
  RequestMsg request_msg;
  request_msg.server_id = server_id;
  request_msg.task_profile = convert(task_profile);
  request_msg.method = RequestMsg::CANCEL;
  _request_msg_pub->publish(request_msg);

  _task_request_fut_ack[task_profile] = std::promise<bool>();
  cancel_success = _task_request_fut_ack[task_profile].get_future();
  return;
}

//==============================================================================
//==============================================================================

std::shared_ptr<TaskActionServer> TaskActionServer::make(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& server_id,
    const std::string& prefix_topic)
{
  return std::shared_ptr<TaskActionServer>(new 
    TaskActionServer(node, server_id, prefix_topic));
}

TaskActionServer::TaskActionServer(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& server_id,
    const std::string& prefix_topic)
  : _node(node), _server_id(server_id)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_sub = _node->create_subscription<RequestMsg>(
    prefix_topic + "_request", dispatch_qos,
    [&](const RequestMsg::UniquePtr msg)
    {
      if (msg->server_id != _server_id)
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

  _result_msg_pub = _node->create_publisher<ResultMsg>(
    prefix_topic + "_result", dispatch_qos);
}

void TaskActionServer::register_callbacks(
    AddTaskCallback add_task_cb_fn, 
    CancelTaskCallback cancel_task_cb_fn)
{
  _add_task_cb_fn = std::move(add_task_cb_fn);
  _cancel_task_cb_fn = std::move(cancel_task_cb_fn);
}

void TaskActionServer::update_progress(const std::vector<TaskMsg>& tasks)
{
  StatusMsg status_msg;
  status_msg.server_id = _server_id;
  status_msg.tasks = tasks;
  _status_msg_pub->publish(status_msg);
}

void TaskActionServer::terminate_task(
    const TaskMsg& task,
    const bool success)
{
  ResultMsg result_msg;
  result_msg.server_id = _server_id;
  result_msg.task = task;

  if (success)
    result_msg.task.state = TaskMsg::TERMINAL_COMPLETED;
  else
    result_msg.task.state = TaskMsg::TERMINAL_FAILED;

  _result_msg_pub->publish(result_msg);
}

//==============================================================================
void TaskActionServer::add_task_impl(const TaskProfileMsg& task_profile)
{
  ResultMsg result_msg;
  result_msg.task.task_profile = task_profile; 
  result_msg.server_id = _server_id;

  if(!_add_task_cb_fn) 
    return;

  if(_add_task_cb_fn(convert(task_profile)))
    result_msg.task.state = TaskMsg::ACTIVE_QUEUED;
  else
    result_msg.task.state = TaskMsg::TERMINAL_FAILED;

  _result_msg_pub->publish(result_msg);
}

void TaskActionServer::cancel_task_impl(const TaskProfileMsg& task_profile)
{
  ResultMsg result_msg;
  result_msg.task.task_profile = task_profile;
  result_msg.server_id = _server_id;
  
  if(!_cancel_task_cb_fn)
    return;

  if(_cancel_task_cb_fn(convert(task_profile)))
    result_msg.task.state = TaskMsg::TERMINAL_CANCELED;
  else
    result_msg.task.state = TaskMsg::TERMINAL_FAILED;

  _result_msg_pub->publish(result_msg);
}

} // namespace action
} // namespace rmf_task_ros2
