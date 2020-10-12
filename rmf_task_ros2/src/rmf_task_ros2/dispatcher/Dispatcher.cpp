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


#include <rmf_task_ros2/dispatcher/Dispatcher.hpp>

#include <rclcpp/node.hpp>

namespace rmf_task_ros2 {
namespace dispatcher {

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::make(
    std::shared_ptr<rclcpp::Node> node)
{
  auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher(node));

  dispatcher->_auctioneer = bidding::Auctioneer::make(node);
  dispatcher->_action_client = 
    action::TaskActionClient::make(node, DispatchActionTopicName);

  // bidding result callback
  using namespace std::placeholders;
  dispatcher->_auctioneer->receive_bidding_result(
    std::bind(&Dispatcher::receive_bidding_winner_cb, dispatcher, _1, _2));
  dispatcher->_action_client->register_callbacks(
    std::bind(&Dispatcher::action_status_cb, dispatcher, _1, _2),
    std::bind(&Dispatcher::action_finish_cb, dispatcher, _1, _2, _3));
  return dispatcher;
}

//==============================================================================
TaskID Dispatcher::submit_task(const TaskProfile& task)
{
  auto submitted_task = task;
  
  // auto generate a taskid with timestamp
  const auto p1 = std::chrono::system_clock::now();
  submitted_task.task_id = "task-" +
    std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
      p1.time_since_epoch()).count());

  DispatchTask dispatch_task {
    submitted_task, DispatchState::Bidding, rmf_utils::nullopt, 0.0 };
  _active_dispatch_tasks[submitted_task.task_id] = dispatch_task;
  bidding::BiddingTask bidding_task;
  bidding_task.task_profile = submitted_task;
  _auctioneer->start_bidding(bidding_task);
  return task.task_id;
}

bool Dispatcher::cancel_task(const TaskID& task_id)
{
  // check if key doesnt exist
  if (!_active_dispatch_tasks.count(task_id))
    return false;

  // todo: need to cancel bidding
  if (_active_dispatch_tasks[task_id].dispatch_state == DispatchState::Bidding)
    return false;

  assert(_active_dispatch_tasks[task_id].winner);
  std::future<bool> fut_task_success; // todo, confirm ack
  _action_client->cancel_task(
      _active_dispatch_tasks[task_id].winner->bidder_name,
      _active_dispatch_tasks[task_id].task_profile,
      fut_task_success);
  return true;
}

rmf_utils::optional<DispatchState> Dispatcher::get_task_dispatch_state(
    const TaskID& task_id)
{
  // check if key doesnt exist
  if (!_active_dispatch_tasks.count(task_id))
    return rmf_utils::nullopt;

  return _active_dispatch_tasks[task_id].dispatch_state;
}

DispatchTasksPtr Dispatcher::get_active_tasks()
{
  return std::make_shared<DispatchTasks>(_active_dispatch_tasks);
}

DispatchTasksPtr Dispatcher::get_terminated_tasks()
{
  return std::make_shared<DispatchTasks>(_terminal_dispatch_tasks);
}

//==============================================================================
Dispatcher::Dispatcher(std::shared_ptr<rclcpp::Node> node_)
: _node(std::move(node_))
{
  // Do Nothing
}

//==============================================================================
// Callback when a bidding winner is provided
void Dispatcher::receive_bidding_winner_cb(
    const TaskID& task_id, 
    const rmf_utils::optional<bidding::Submission> winner)
{
  std::cout << "[BiddingResultCallback] | task: " << task_id;
  if (!winner)
  {
    std::cerr << " | No winner found!" << std::endl;
    return;
  }
  std::cout << " | Found a winner! " << winner->bidder_name << std::endl;

  // we will initiate a task via task action here!
  _active_dispatch_tasks[task_id].winner = *winner;

  std::future<bool> fut_task_success; // todo, confirm ack
  _action_client->add_task(
      winner->bidder_name,
      _active_dispatch_tasks[task_id].task_profile, 
      fut_task_success);

  // when fut is received, change task state as queued
  _active_dispatch_tasks[task_id].dispatch_state = DispatchState::Queued;
}

//==============================================================================
// task action callback
void Dispatcher::action_status_cb(
    const std::string& server_id,
    const std::vector<action::TaskMsg>& tasks)
{
  std::cout << "[action status] number of on-going tasks"
            << tasks.size() << std::endl;
  // update task status here, todo: update estimated finish time?
  for(auto tsk : tasks)
  {
    _active_dispatch_tasks[tsk.task_profile.task_id].dispatch_state = 
      static_cast<DispatchState>(tsk.state);
  }
}

void Dispatcher::action_finish_cb(
    const std::string& server_id, 
    const action::TaskMsg& task, 
    const bool success)
{
  auto finish_task_id = task.task_profile.task_id;

  // check if key doesnt exist
  if (!_active_dispatch_tasks.count(finish_task_id)) return;

  _active_dispatch_tasks[finish_task_id].dispatch_state = 
      static_cast<DispatchState>(task.state);
  std::cout << "[action result] completed task: " << finish_task_id 
            << " scucess?: " << success << std::endl;

  _terminal_dispatch_tasks[finish_task_id] = 
      _active_dispatch_tasks[finish_task_id];
  _active_dispatch_tasks.erase(finish_task_id);
}

} // namespace dispatcher
} // namespace rmf_task_ros2
