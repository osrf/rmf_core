
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

#ifndef RMF_TASK_ROS2__DISPATCHER__NODE_HPP
#define RMF_TASK_ROS2__DISPATCHER__NODE_HPP

#include <rclcpp/node.hpp>
#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/Nomination.hpp>

#include <rmf_task_msgs/msg/loop.hpp>
#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/station.hpp>


namespace rmf_task_ros2 {
namespace dispatcher {

//==============================================================================
class DispatcherNode : public rclcpp::Node
{
public:

  DispatcherNode(const rclcpp::NodeOptions& options);

  ~DispatcherNode()
  {};


public:

  using Itinerary = std::vector<std::string>;
  std::map<std::string, Itinerary> queue_tasks;

private:

  using Loop = rmf_task_msgs::msg::Loop;
  using LoopSub = rclcpp::Subscription<Loop>;
  LoopSub::SharedPtr _loop_sub;

  using Delivery = rmf_task_msgs::msg::Delivery;
  using DeliverySub = rclcpp::Subscription<Delivery>;
  DeliverySub::SharedPtr _delivery_sub;

  using Station = rmf_task_msgs::msg::Station;
  using StationSub = rclcpp::Subscription<Station>;
  StationSub::SharedPtr _station_sub;

  using DispatchNoticePub = rclcpp::Publisher<DispatchNotice>;
  DispatchNoticePub::SharedPtr _dispatch_notice_pub;

  using DispatchProposalSub = rclcpp::Subscription<DispatchProposal>;
  DispatchProposalSub::SharedPtr _dispatch_proposal_sub;

  using DispatchConclusionPub = rclcpp::Publisher<DispatchConclusion>;
  DispatchConclusionPub::SharedPtr _dispatch_conclusion_pub;

  using DispatchAckSub = rclcpp::Subscription<DispatchAck>;
  DispatchAckSub::SharedPtr _dispatch_ack_sub;

private:

  // send dispatch notice to selected bidder
  void start_bidding();

  // Receive proposal and evaluate
  void receive_proposal(const DispatchProposal& msg);

  // Receive conclusion ack
  void receive_conclusion_ack(const DispatchAck& msg);

};


//==============================================================================

/// Make a DispatcherNode instance
std::shared_ptr<rclcpp::Node> make_node(
  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());


} // namespace dispatcher
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__DISPATCHER__NODE_HPP
