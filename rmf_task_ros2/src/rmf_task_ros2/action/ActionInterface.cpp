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
        _status_callback_fn(msg->tasks);
    });
  
  _result_msg_sub = _node->create_subscription<ResultMsg>(
    prefix_topic + "_result", dispatch_qos,
    [&](const ResultMsg::UniquePtr msg)
    {
      bool is_terminated = (msg->response == ResultMsg::TERMINATED);

      // Acknowledgment mode
      if(!is_terminated)
      {
        // todo: check if server_id and task_id match
        if (_request_msg.server_id != msg->server_id) return;
        if (_request_msg.task.task_id != msg->task.task_id) return;

        auto res = static_cast<ResultResponse>(msg->response);
        _ack_promise.set_value(res);
        return;
      }

      // termination mode
      if(_termination_callback_fn)
      {
        _termination_callback_fn(
          msg->task,
          static_cast<State::Terminal>(msg->terminal_state));
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
    const std::string& task_id, 
    const TaskMsg& task_description,
    std::future<ResultResponse>& future_res)
{
  // reinitiaze promise
  _ack_promise = std::promise<ResultResponse>(); 
  
  // send request and wait for acknowledgement
  _request_msg.server_id = server_id;
  _request_msg.task = task_description;
  _request_msg.method = RequestMsg::ADD;
  _request_msg.task.task_id = task_id; // need?
  _request_msg_pub->publish(_request_msg);
  future_res = _ack_promise.get_future();
  return;
}

void TaskActionClient::cancel_task(
    const std::string& server_id, 
    const std::string& task_id, 
    std::future<ResultResponse>& future_res)
{
  // reinitiaze promise
  _ack_promise = std::promise<ResultResponse>(); 

  // send cancel and wait for acknowledgement
  _request_msg.server_id = server_id;
  _request_msg.task.task_id = task_id;
  _request_msg.method = RequestMsg::CANCEL;
  _request_msg_pub->publish(_request_msg);
  future_res = _ack_promise.get_future();
  return;
}

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
      if(msg->method == RequestMsg::ADD)
        this->add_task_impl(msg->task);
      else if(msg->method == RequestMsg::CANCEL)
        this->cancel_task_impl(msg->task);
      else
        std::cerr << "Request Method is not supported!!!"<< std::endl;
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
    const State::Terminal state)
{
  ResultMsg result_msg;
  result_msg.task = task;
  result_msg.response = ResultMsg::TERMINATED;
  result_msg.terminal_state = static_cast<uint8_t>(state);
  _result_msg_pub->publish(result_msg);
}

void TaskActionServer::add_task_impl(const TaskMsg& task)
{
  if(!_add_task_cb_fn) 
    return;
  auto res = _add_task_cb_fn(task);
  
  ResultMsg result_msg;
  result_msg.task = task;
  result_msg.response = static_cast<uint8_t>(res);
  _result_msg_pub->publish(result_msg);
}

void TaskActionServer::cancel_task_impl(const TaskMsg& task)
{
  if(!_cancel_task_cb_fn) 
    return;
  auto res = _cancel_task_cb_fn(task.task_id);
  
  ResultMsg result_msg;
  result_msg.task = task;
  result_msg.response = static_cast<uint8_t>(res);
  _result_msg_pub->publish(result_msg);
}

} // namespace action
} // namespace rmf_task_ros2
