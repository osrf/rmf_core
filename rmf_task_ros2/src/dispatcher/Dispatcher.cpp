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

  _loop_sub = create_subscription<Loop>(
    rmf_task_ros2::LoopTopicName, dispatch_qos,
    [&](const Loop::UniquePtr msg)
    {   
      Itinerary itinerary;
      for ( int i = 0; i < (int)msg->num_loops; i++ )
      {
        itinerary.push_back(msg->start_name);
        itinerary.push_back(msg->finish_name);
      }      
      queue_tasks[msg->task_id] = itinerary;
      this->start_bidding();
    });

  _delivery_sub = create_subscription<Delivery>(
    rmf_task_ros2::DeliveryTopicName, dispatch_qos,
    [&](const Delivery::UniquePtr msg)
    {
      Itinerary itinerary;
      itinerary.push_back(msg->pickup_place_name);
      itinerary.push_back(msg->dropoff_place_name);
      queue_tasks[msg->task_id] = itinerary;      
      this->start_bidding();
    });

  _station_sub = create_subscription<Station>(
    rmf_task_ros2::StationTopicName, dispatch_qos,
    [&](const Station::UniquePtr msg)
    {
      Itinerary itinerary;
      itinerary.push_back(msg->place_name);
      queue_tasks[msg->task_id] = itinerary;
      this->start_bidding();
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

  _dispatch_notice_pub = create_publisher<DispatchNotice>(
    rmf_task_ros2::DispatchNoticeTopicName, dispatch_qos);

}

// TODO: async start bidding proccess (or make it a seperate class)
void DispatcherNode::start_bidding()
{
  DispatchNotice notice_msg;
  
  // Populate notice msg with queue_task here

  _dispatch_notice_pub->publish(notice_msg);
}

void DispatcherNode::receive_proposal(const DispatchProposal& msg)
{
  auto it = queue_tasks.find(msg.task_id);
  if (it == queue_tasks.end() )
    return;

  // TODO add proposal to nominees list.

  bool bidding_timeout = true;
  bool received_all_bidders = true;

  if ( !(bidding_timeout or received_all_bidders))
    return;
  
  // Nominate and Evaluate Here!
  Nomination::TaskEstimatesPtr dummy_estimates;
  Nomination task_nomination(dummy_estimates);
  Nomination::TaskEstimate chosen_estimate = 
    task_nomination.evaluate(QuickestFinishEvaluator());

  // Sent Conclusion which will state the selected robot
  DispatchConclusion conclusion_msg;
  conclusion_msg.fleet_name = chosen_estimate.fleet_name;
  conclusion_msg.robot_name = chosen_estimate.robot_name;
  _dispatch_conclusion_pub->publish(conclusion_msg);
}

void DispatcherNode::receive_conclusion_ack(const DispatchAck& msg)
{
  // Wrap up task bidding here
}

//==============================================================================

std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions& options)
{
  return std::make_shared<DispatcherNode>(options);
}

} // namespace dispatcher
} // namespace rmf_task_ros2
