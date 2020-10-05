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
void DispatcherNode::submit_task(const bidding::BiddingTask& task)
{
  TaskStatePair task_state(task, action::State::Active::INVALID);
  _active_tasks[task.task_id] = task_state;
  _auctioneer->start_bidding(task);
}

bool DispatcherNode::cancel_task(bidding::TaskID task_id)
{
  // check if key doesnt exist
  if (!_active_tasks.count(task_id))
    return false;

  // todo: need to cancel bidding
  if (_active_tasks[task_id].second == action::State::Active::INVALID)
    return false;

  std::future<action::ResultResponse> test_fut;

  assert(_active_tasks[task_id].first.submissions.size() != 0);
  auto server_id = _active_tasks[task_id].first.submissions[0].fleet_name;
  _action_client->cancel_task( server_id, task_id, test_fut);

  return true;
}

rmf_utils::optional<action::State::Active> DispatcherNode::get_task_status(
    bidding::TaskID task_id)
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

  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _loop_sub = create_subscription<Loop>(
    rmf_task_ros2::LoopTopicName, dispatch_qos,
    [&](const Loop::UniquePtr msg)
    {   
      bidding::BiddingTask bidding_task;
      bidding_task.task_id = msg->task_id;
      bidding_task.task_type = TaskType::Loop;
      bidding_task.announce_all = true;
      bidding_task.submission_time = std::chrono::steady_clock::now();
      for ( int i = 0; i < (int)msg->num_loops; i++ )
      {
        bidding_task.itinerary.push_back(msg->start_name);
        bidding_task.itinerary.push_back(msg->finish_name);
      }
      this->submit_task(bidding_task);
    });

  _delivery_sub = create_subscription<Delivery>(
    rmf_task_ros2::DeliveryTopicName, dispatch_qos,
    [&](const Delivery::UniquePtr msg)
    {
      bidding::BiddingTask bidding_task;
      bidding_task.task_id = msg->task_id;
      bidding_task.task_type = TaskType::Delivery;
      bidding_task.announce_all = true;
      bidding_task.submission_time = std::chrono::steady_clock::now();
      bidding_task.itinerary.push_back(msg->pickup_place_name);
      bidding_task.itinerary.push_back(msg->dropoff_place_name);
      this->submit_task(bidding_task);
    });

  _station_sub = create_subscription<Station>(
    rmf_task_ros2::StationTopicName, dispatch_qos,
    [&](const Station::UniquePtr msg)
    {
      bidding::BiddingTask bidding_task;
      bidding_task.task_id = msg->task_id;
      bidding_task.task_type = TaskType::Station;
      bidding_task.announce_all = true;
      bidding_task.submission_time = std::chrono::steady_clock::now();    
      bidding_task.itinerary.push_back(msg->place_name);
      this->submit_task(bidding_task);
    });
}

//==============================================================================
// Callback when a bidding winner is provided
void DispatcherNode::receive_bidding_winner_cb(
    const bidding::TaskID& task_id, 
    const rmf_utils::optional<bidding::Submission> winner)
{
  std::cout << "[BiddingResultCallback] | task: " << task_id;
  if (!winner)
  {
    std::cerr << " | No winner found!" << std::endl;
    return;
  }
  std::cout << " | Found a winner! " << winner->fleet_name << std::endl;

  // we will initiate a task via task action here! (TODO)
  _active_tasks[task_id].first.submissions.push_back(*winner);
  action::TaskMsg task = convert_task(_active_tasks[task_id].first);
  std::future<action::ResultResponse> test_fut;

  _action_client->add_task(
      winner->fleet_name, task_id, task, test_fut);

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
    _active_tasks[tsk.task_id].second = 
      static_cast<action::State::Active>(tsk.active_state);
  }
}

void DispatcherNode::action_finish_cb(
    const action::TaskMsg& task, 
    const action::State::Terminal state)
{
  std::cout << "[action result] completed task: " << task.task_id 
            << " state: " << static_cast<uint8_t>(state) << std::endl;
  _active_tasks.erase(task.task_id); // todo check if within
}

//==============================================================================
action::TaskMsg convert_task(const bidding::BiddingTask& bid)
{
  action::TaskMsg task;
  task.task_id = bid.task_id;
  task.submission_time = rmf_traffic_ros2::convert(bid.submission_time);
  task.type.value = static_cast<uint8_t>(bid.task_type);
  task.itinerary = bid.itinerary;
  return task;
}

} // namespace dispatcher
} // namespace rmf_task_ros2
