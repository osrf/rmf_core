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
    std::bind(&DispatcherNode::receive_bidding_winner_callback, node, _1, _2));

  return node;
}

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
      _auctioneer->start_bidding(bidding_task);
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
      _auctioneer->start_bidding(bidding_task);
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
      _auctioneer->start_bidding(bidding_task);
    });
}

} // namespace dispatcher
} // namespace rmf_task_ros2
