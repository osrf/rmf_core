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

#include <rmf_task_ros2/bidder/MinimalBidder.hpp>

namespace rmf_task_ros2 {
namespace bidder {

//==============================================================================

MinimalBidder::MinimalBidder(std::shared_ptr<rclcpp::Node> node)
: _node(std::move(node))
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _dispatch_notice_sub = _node->create_subscription<DispatchNotice>(
  rmf_task_ros2::DispatchNoticeTopicName, dispatch_qos,
  [&](const DispatchNotice::UniquePtr msg)
  {
    this->receive_notice(*msg);
  });

  _dispatch_proposal_pub = _node->create_publisher<DispatchProposal>(
    rmf_task_ros2::DispatchProposalTopicName, dispatch_qos);

  _dispatch_conclusion_sub = _node->create_subscription<DispatchConclusion>(
    rmf_task_ros2::DispatchConclusionTopicName, dispatch_qos,
    [&](const DispatchConclusion::UniquePtr msg)
    {
      this->receive_conclusion(*msg);
    });

  _dispatch_ack_pub = _node->create_publisher<DispatchAck>(
    rmf_task_ros2::DispatchAckTopicName, dispatch_qos);
}

void MinimalBidder::add_bidder(
  const std::string& fleet_name,
  ParseEstimatesCallback estimates_cb,
  ExecuteTaskCallback execute_cb)
{
  _fleet_name = fleet_name;
  _get_estimates_fn = std::move(estimates_cb);
  _execute_task_fn = std::move(execute_cb);
}

void MinimalBidder::receive_notice(const DispatchNotice& msg)
{
  std::cout << " Receive Bidding notice for task_id: " 
            << msg.task_id << std::endl;
  
  // check is it for me??
  auto bidders = msg.fleet_names;
  auto it = std::find(bidders.begin(), bidders.end(), _fleet_name);
  if (it == bidders.end())
  {
    std::cout << "not me!" <<  _fleet_name<< std::endl;
    return;
  }

  // get tasks estimates
  if (!_get_estimates_fn)
    return;

  auto nominees = _get_estimates_fn(msg.itinerary);
  if (nominees->size() == 0)
  {
    std::cerr << " no nominees are provided :( " << std::endl;
    return;
  }

  // Submit nominee estimations and get the chosen robot
  Nomination task_nomination(nominees);
  Nomination::Nominee chosen_estimate = 
    task_nomination.evaluate(QuickestFinishEvaluator());

  // Submit proposal
  auto best_proposal = Nomination::convert_msg(chosen_estimate);
  best_proposal.task_id = msg.task_id;
  _dispatch_proposal_pub->publish(best_proposal);
}

void MinimalBidder::receive_conclusion(const DispatchConclusion& msg)
{
  std::cout << " Receive Bidding conclusion for task_id: " 
            << msg.task_id << std::endl;
  
  if (_fleet_name != msg.fleet_name)
    return;

  // get tasks estimates
  if (!_execute_task_fn)
    return;

  this->_execute_task_fn(msg);
  
  // publish ack
  DispatchAck ack_msg;
  ack_msg.task.task_id = msg.task_id;
  _dispatch_ack_pub->publish(ack_msg);
}

} // namespace bidder
} // namespace rmf_task_ros2
