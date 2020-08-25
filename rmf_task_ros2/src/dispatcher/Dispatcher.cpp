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
DispatcherNode::DispatcherNode(const rclcpp::NodeOptions& options)
: Node("rmf_task_dispatcher_node", options)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();
  
  double timeout_sec = 2;
  _bidding_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(timeout_sec));

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
      this->start_bidding(bidding_task);
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
      this->start_bidding(bidding_task);
    });

  _station_sub = create_subscription<Station>(
    rmf_task_ros2::StationTopicName, dispatch_qos,
    [&](const Station::UniquePtr msg)
    {
      BiddingTask bidding_task;
      bidding_task.task_id = msg->task_id;
      bidding_task.start_time = std::chrono::steady_clock::now();    
      bidding_task.itinerary.push_back(msg->place_name);
      this->start_bidding(bidding_task);
    });

  _dispatch_proposal_sub = create_subscription<DispatchProposal>(
    rmf_task_ros2::DispatchProposalTopicName, dispatch_qos,
    [&](const DispatchProposal::UniquePtr msg)
    {
      this->receive_proposal(*msg);
    });

  _dispatch_conclusion_pub = create_publisher<DispatchConclusion>(
    rmf_task_ros2::DispatchConclusionTopicName, dispatch_qos);

  _dispatch_ack_sub = create_subscription<DispatchAck>(
    rmf_task_ros2::DispatchAckTopicName, dispatch_qos,
    [&](const DispatchAck::UniquePtr msg)
    {
      this->receive_conclusion_ack(*msg);
    });

  _timer = create_wall_timer(std::chrono::milliseconds(500), [this]()
    {
      this->check_bidding_process();
    });

  _dispatch_notice_pub = create_publisher<DispatchNotice>(
    rmf_task_ros2::DispatchNoticeTopicName, dispatch_qos);
}

void DispatcherNode::start_bidding(const BiddingTask& bidding_task)
{
  _queue_bidding_tasks.push_back(bidding_task);  
  std::cout << " Start Task Bidding for task_id: " 
            << bidding_task.task_id << std::endl;

  // Populate notice msg with queue_task here
  DispatchNotice notice_msg;
  notice_msg.itinerary = bidding_task.itinerary;
  notice_msg.submission_time = this->now();
  notice_msg.fleet_names = bidding_task.bidders;
  _dispatch_notice_pub->publish(notice_msg);
}

void DispatcherNode::check_bidding_process()
{
  std::cout << " Checking Bidding... " << std::endl;
  
  // check if bidding task has reached timeout... sad
  auto task_it = std::find_if(
    _queue_bidding_tasks.begin(), _queue_bidding_tasks.end(),
    [&](const BiddingTask& task)
    { 
      auto duration = std::chrono::steady_clock::now() - task.start_time;
      return duration >= *_bidding_timeout; 
    });
  
  if (task_it == _queue_bidding_tasks.end())
    return;
  
  std::cout << " Timeout reached! ready to remove task_id: " 
            << task_it->task_id << std::endl;
  _queue_bidding_tasks.erase(task_it);
}

void DispatcherNode::receive_proposal(const DispatchProposal& msg)
{
  auto task_it = std::find_if(
    _queue_bidding_tasks.begin(), _queue_bidding_tasks.end(),
    [&](const BiddingTask& task){ return task.task_id == msg.task_id;});

  if (task_it == _queue_bidding_tasks.end())
    return;

  // add proposal to nominees list.
  auto nominee = Nomination::convert_msg(msg);
  task_it->nominees->push_back(nominee);

  // check if all bidders' proposals are received
  if ( task_it->bidders.size() != task_it->nominees->size())
    return;
  
  // Nominate and Evaluate Here!
  Nomination task_nomination(task_it->nominees);
  Nomination::Nominee chosen_estimate = 
    task_nomination.evaluate(QuickestFinishEvaluator());

  // Sent Conclusion which will state the selected robot
  DispatchConclusion conclusion_msg;
  conclusion_msg.fleet_name = chosen_estimate.fleet_name;
  conclusion_msg.robot_name = chosen_estimate.robot_name;
  _dispatch_conclusion_pub->publish(conclusion_msg);
}

void DispatcherNode::receive_conclusion_ack(const DispatchAck& msg)
{
  // Finish up task bidding
  auto task_it = std::find_if(
    _queue_bidding_tasks.begin(), _queue_bidding_tasks.end(),
    [&](const BiddingTask& task){ return task.task_id == msg.task.task_id;});
  
  _queue_bidding_tasks.erase(task_it);
}

//==============================================================================

std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions& options)
{
  return std::make_shared<DispatcherNode>(options);
}

} // namespace dispatcher
} // namespace rmf_task_ros2
