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


#include "Dispatcher.hpp"

#include <rclcpp/node.hpp>

namespace rmf_task_ros2 {
namespace dispatcher {

//==============================================================================
std::shared_ptr<DispatcherNode> DispatcherNode::make_node()
{
  auto node = std::shared_ptr<DispatcherNode>(new DispatcherNode);
  node->_auctioneer = bidding::Auctioneer::make(node);
  node->_action_client = 
    action::TaskActionClient::make(node, DispatchActionTopicName);

  // bidding result callback
  using namespace std::placeholders;
  node->_auctioneer->receive_bidding_result(
    std::bind(&DispatcherNode::receive_bidding_winner_cb, node, _1, _2));
  node->_action_client->register_callbacks(
    std::bind(&DispatcherNode::action_status_cb, node, _1),
    std::bind(&DispatcherNode::action_finish_cb, node, _1, _2));
  return node;
}

//==============================================================================
TaskID DispatcherNode::submit_task(const TaskProfile& task)
{
  // auto generate a taskid
  auto auction_task = task;
  const auto p1 = std::chrono::system_clock::now();
  auction_task.task_id = "task-" +
    std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
      p1.time_since_epoch()).count());

  ActiveTaskState task_state(auction_task, action::State::Active::INVALID);
  _active_tasks[auction_task.task_id] = task_state;
  bidding::BiddingTask bidding_task;
  bidding_task.task_profile = auction_task;
  _auctioneer->start_bidding(bidding_task);
  return task.task_id;
}

bool DispatcherNode::cancel_task(const TaskID& task_id)
{
  // check if key doesnt exist
  if (!_active_tasks.count(task_id))
    return false;

  // todo: need to cancel bidding
  if (_active_tasks[task_id].second == action::State::Active::INVALID)
    return false;

  std::future<action::ResultResponse> test_fut;

  // assert(_active_tasks[task_id].first.submissions.size() != 0);
  // auto server_id = _active_tasks[task_id].first.submissions[0].bidder_name;

  // TODO!!! fix indentify server_id
  _action_client->cancel_task( 
      "server_id", _active_tasks[task_id].first, test_fut);
  return true;
}

rmf_utils::optional<action::State::Active> DispatcherNode::get_task_status(
    const TaskID& task_id)
{
  // check if key doesnt exist
  if (!_active_tasks.count(task_id))
    return rmf_utils::nullopt;

  return _active_tasks[task_id].second;
}

//==============================================================================
DispatcherNode::DispatcherNode()
: Node("rmf_task_dispatcher_node")
{
  std::cout << "~Initializing Dispatcher Node~" << std::endl;
}

//==============================================================================
// Callback when a bidding winner is provided
void DispatcherNode::receive_bidding_winner_cb(
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
  std::future<action::ResultResponse> test_fut;

  _action_client->add_task(
      winner->bidder_name, _active_tasks[task_id].first, test_fut);

  // when fut is received, change task state as queued
  _active_tasks[task_id].second = action::State::Active::QUEUED;
}

//==============================================================================
// task action callback
void DispatcherNode::action_status_cb(
    const std::vector<action::TaskMsg>& tasks)
{
  std::cout << "[action status] number of on-going tasks"
            << tasks.size() << std::endl;
  // update task status here, todo: update estimated finish time?
  for(auto tsk : tasks)
  {
    _active_tasks[tsk.task_profile.task_id].second = 
      static_cast<action::State::Active>(tsk.active_state);
  }
}

void DispatcherNode::action_finish_cb(
    const action::TaskMsg& task, 
    const action::State::Terminal state)
{
  std::cout << "[action result] completed task: " << task.task_profile.task_id 
            << " state: " << static_cast<uint8_t>(state) << std::endl;
  _active_tasks.erase(task.task_profile.task_id); // todo check if within
}

//==============================================================================
// action::TaskMsg convert_task(const bidding::BiddingTask& bid)
// {
//   action::TaskMsg task;
//   task.task_id = bid.task_id;
//   task.submission_time = rmf_traffic_ros2::convert(bid.submission_time);
//   task.type.value = static_cast<uint8_t>(bid.task_type);
//   return task;
// }

} // namespace dispatcher
} // namespace rmf_task_ros2
