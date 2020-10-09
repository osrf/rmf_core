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
  // this->Dispatcher(node);

  auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher(node));

  // auto node = std::shared_ptr<Dispatcher>(new Dispatcher);
  dispatcher->_auctioneer = bidding::Auctioneer::make(node);
  dispatcher->_action_client = 
    action::TaskActionClient::make(node, DispatchActionTopicName);

  // bidding result callback
  using namespace std::placeholders;
  dispatcher->_auctioneer->receive_bidding_result(
    std::bind(&Dispatcher::receive_bidding_winner_cb, dispatcher, _1, _2));
  dispatcher->_action_client->register_callbacks(
    std::bind(&Dispatcher::action_status_cb, dispatcher, _1),
    std::bind(&Dispatcher::action_finish_cb, dispatcher, _1, _2));
  return dispatcher;
}

//==============================================================================
TaskID Dispatcher::submit_task(const TaskProfile& task)
{
  // auto generate a taskid
  auto auction_task = task;
  const auto p1 = std::chrono::system_clock::now();
  auction_task.task_id = "task-" +
    std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
      p1.time_since_epoch()).count());

  ActiveTaskState task_state(auction_task, DispatchState::Bidding);
  _active_tasks[auction_task.task_id] = task_state;
  bidding::BiddingTask bidding_task;
  bidding_task.task_profile = auction_task;
  _auctioneer->start_bidding(bidding_task);
  return task.task_id;
}

bool Dispatcher::cancel_task(const TaskID& task_id)
{
  // check if key doesnt exist
  if (!_active_tasks.count(task_id))
    return false;

  // todo: need to cancel bidding
  if (_active_tasks[task_id].second == DispatchState::Bidding)
    return false;

  std::future<bool> fut_task_success;

  // assert(_active_tasks[task_id].first.submissions.size() != 0);
  // auto server_id = _active_tasks[task_id].first.submissions[0].bidder_name;

  // TODO!!! fix indentify server_id
  _action_client->cancel_task( 
      "server_id", _active_tasks[task_id].first, fut_task_success);
  return true;
}

rmf_utils::optional<DispatchState> Dispatcher::get_task_status(
    const TaskID& task_id)
{
  // check if key doesnt exist
  if (!_active_tasks.count(task_id))
    return rmf_utils::nullopt;

  return _active_tasks[task_id].second;
}

//==============================================================================
Dispatcher::Dispatcher(std::shared_ptr<rclcpp::Node> node_)
: _node(std::move(node_))
{
  std::cout << "~Initializing RMF Dispatcher~" << std::endl;
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

  // we will initiate a task via task action here! (TODO)
  // _active_tasks[task_id].first.submissions.push_back(*winner);
  // action::TaskMsg task = convert_task(_active_tasks[task_id].first);
  std::future<bool> fut_task_success;

  _action_client->add_task(
      winner->bidder_name, _active_tasks[task_id].first, fut_task_success);

  // when fut is received, change task state as queued
  _active_tasks[task_id].second = DispatchState::Queued;
}

//==============================================================================
// task action callback
void Dispatcher::action_status_cb(
    const std::vector<action::TaskMsg>& tasks)
{
  std::cout << "[action status] number of on-going tasks"
            << tasks.size() << std::endl;
  // update task status here, todo: update estimated finish time?
  for(auto tsk : tasks)
  {
    _active_tasks[tsk.task_profile.task_id].second = 
      static_cast<DispatchState>(tsk.state);
  }
}

void Dispatcher::action_finish_cb(
    const action::TaskMsg& task, 
    const bool success)
{
  std::cout << "[action result] completed task: " << task.task_profile.task_id 
            << " scucess?: " << success << std::endl;
  _active_tasks.erase(task.task_profile.task_id); // todo check if within
}

} // namespace dispatcher
} // namespace rmf_task_ros2
