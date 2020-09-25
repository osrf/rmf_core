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

#include <rmf_task_ros2/bidding/MinimalBidder.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================

MinimalBidder::MinimalBidder(
  const Profile& profile,
  std::shared_ptr<rclcpp::Node> node)
: _profile(profile), _node(std::move(node))
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _dispatch_notice_sub = _node->create_subscription<BidNotice>(
  rmf_task_ros2::BidNoticeTopicName, dispatch_qos,
  [&](const BidNotice::UniquePtr msg)
  {
    this->receive_notice(*msg);
  });

  _dispatch_proposal_pub = _node->create_publisher<BidProposal>(
    rmf_task_ros2::BidProposalTopicName, dispatch_qos);
}

void MinimalBidder::call_for_bid(
  ParseSubmissionCallback submission_cb)
{
  _get_submission_fn = std::move(submission_cb);
}

void MinimalBidder::receive_notice(const BidNotice& msg)
{
  std::cout << " Receive Bidding notice for task_id: " 
            << msg.task_id << std::endl;
  
  // check if tasktype is supported by this F.A
  auto req_type = static_cast<TaskType>(msg.type.value);
  if (!_profile.valid_tasks.count(req_type));
  {
    std::cout << _profile.fleet_name << ": task type "
              << msg.type.value << " is invalid" << std::endl;
    return;
  }

  // check is the bidding annoucment is for me
  if (!msg.announce_all)
  {
    auto bidders = msg.fleet_names;
    auto it = std::find(bidders.begin(), bidders.end(), _profile.fleet_name);
    if (it == bidders.end())
    {
      std::cout << "not me!" <<  _profile.fleet_name<< std::endl;
      return;
    }
  }

  // get tasks estimates
  if (!_get_submission_fn)
    return;

  auto bid_submission = _get_submission_fn(msg);
  
  // TODO: check if within deadline
  auto now = std::chrono::steady_clock::now();

  // check if cost and end_time is infinite
  
  // Submit proposal
  auto best_proposal = convert(bid_submission);
  best_proposal.fleet_name = _profile.fleet_name;
  best_proposal.task_id = msg.task_id;
  best_proposal.submission_time = _node->now();
  _dispatch_proposal_pub->publish(best_proposal);
}

} // namespace bidder
} // namespace rmf_task_ros2
