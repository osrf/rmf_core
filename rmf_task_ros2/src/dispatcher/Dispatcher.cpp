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
  return std::shared_ptr<DispatcherNode>(new DispatcherNode);
}

DispatcherNode::DispatcherNode()
: Node("rmf_task_dispatcher_node"),
 _auctioneer(enable_shared_from_this<DispatcherNode>::shared_from_this())
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _loop_sub = create_subscription<Loop>(
    rmf_task_ros2::LoopTopicName, dispatch_qos,
    [&](const Loop::UniquePtr msg)
    {   
      BiddingTask bidding_task;
      bidding_task.task_id = msg->task_id;
      bidding_task.start_time = std::chrono::steady_clock::now();
      for ( int i = 0; i < (int)msg->num_loops; i++ )
      {
        bidding_task.itinerary.push_back(msg->start_name);
        bidding_task.itinerary.push_back(msg->finish_name);
      }
      _auctioneer.start_bidding(bidding_task);
    });

  _delivery_sub = create_subscription<Delivery>(
    rmf_task_ros2::DeliveryTopicName, dispatch_qos,
    [&](const Delivery::UniquePtr msg)
    {
      BiddingTask bidding_task;
      bidding_task.task_id = msg->task_id;
      bidding_task.start_time = std::chrono::steady_clock::now();
      bidding_task.itinerary.push_back(msg->pickup_place_name);
      bidding_task.itinerary.push_back(msg->dropoff_place_name);
      bidding_task.bidders.push_back("dummybot"); // todo
      _auctioneer.start_bidding(bidding_task);
    });

  _station_sub = create_subscription<Station>(
    rmf_task_ros2::StationTopicName, dispatch_qos,
    [&](const Station::UniquePtr msg)
    {
      BiddingTask bidding_task;
      bidding_task.task_id = msg->task_id;
      bidding_task.start_time = std::chrono::steady_clock::now();    
      bidding_task.itinerary.push_back(msg->place_name);
      _auctioneer.start_bidding(bidding_task);
    });
}

} // namespace dispatcher
} // namespace rmf_task_ros2

//==============================================================================
// Trash Codes, TOdo remove!

  // // Nominate and Evaluate Here!
  // Nomination task_nomination(task_it->nominees);
  // Nomination::Nominee chosen_estimate = 
  //   task_nomination.evaluate(QuickestFinishEvaluator());

  // std::cout << " selected robot: " 
  //           << chosen_estimate.robot_name << std::endl;

  // // todo: this will need to change
  // // Sent Conclusion which will state the selected robot
  // DispatchRequest conclusion_msg;
  // conclusion_msg.task_id = msg.task_id;
  // conclusion_msg.fleet_name = "dummybot"; // chosen_estimate.fleet_name;
  // conclusion_msg.robot_name = chosen_estimate.robot_name;
  // _dispatch_conclusion_pub->publish(conclusion_msg);
// }

// void DispatcherNode::receive_conclusion_ack(const DispatchStatus& msg)
// {
//   std::cout << " Task is active, Done bidding! for task_id: " 
//             << msg.task.task_id << std::endl;

//   // Finish up task bidding
//   auto task_it = std::find_if(
//     _queue_bidding_tasks.begin(), _queue_bidding_tasks.end(),
//     [&](const BiddingTask& task){ return task.task_id == msg.task.task_id;});
  
//   _queue_bidding_tasks.erase(task_it);
// }
