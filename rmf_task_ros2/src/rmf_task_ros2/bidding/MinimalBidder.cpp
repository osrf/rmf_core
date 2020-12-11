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

#include <rmf_task_ros2/bidding/Bidding.hpp>
#include <rmf_task_ros2/bidding/MinimalBidder.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
class MinimalBidder::Implementation
{
public:

  std::shared_ptr<rclcpp::Node> node;
  std::string fleet_name;
  std::set<uint32_t> valid_tasks;
  ParseSubmissionCallback get_submission_fn;

  using BidNoticeSub = rclcpp::Subscription<BidNotice>;
  BidNoticeSub::SharedPtr dispatch_notice_sub;

  using BidProposalPub = rclcpp::Publisher<BidProposal>;
  BidProposalPub::SharedPtr dispatch_proposal_pub;

  Implementation(
    std::shared_ptr<rclcpp::Node> node_,
    const std::string& fleet_name_,
    const std::set<uint32_t>& valid_tasks_)
  : node(std::move(node_)), fleet_name(fleet_name_), valid_tasks(valid_tasks_)
  {
    const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

    dispatch_notice_sub = node->create_subscription<BidNotice>(
      rmf_task_ros2::BidNoticeTopicName, dispatch_qos,
      [&](const BidNotice::UniquePtr msg)
      {
        this->receive_notice(*msg);
      });

    dispatch_proposal_pub = node->create_publisher<BidProposal>(
      rmf_task_ros2::BidProposalTopicName, dispatch_qos);
  }

  // Callback fn when a dispatch notice is received
  void receive_notice(const BidNotice& msg)
  {
    std::cout << " [Bidder] Received Bidding notice for task_id: "
              << msg.task_profile.task_id << std::endl;

    // check if tasktype is supported by this F.A
    if (!valid_tasks.count(msg.task_profile.task_type.type))
    {
      std::cout << fleet_name << ": task type "
                << msg.task_profile.task_type.type <<" is invalid"<< std::endl;
      return;
    }

    // check if get submission function is declared
    if (!get_submission_fn)
      return;

    // Submit proposal
    auto bid_submission = get_submission_fn(msg);
    auto best_proposal = convert(bid_submission);
    best_proposal.fleet_name = fleet_name;
    best_proposal.task_profile = msg.task_profile;
    dispatch_proposal_pub->publish(best_proposal);
  }
};

//==============================================================================
std::shared_ptr<MinimalBidder> MinimalBidder::make(
  const std::shared_ptr<rclcpp::Node>& node,
  const std::string& fleet_name,
  const std::set<uint32_t>& valid_tasks)
{
  auto pimpl = rmf_utils::make_unique_impl<Implementation>(
    node, fleet_name, valid_tasks);

  if (pimpl)
  {
    auto bidder = std::shared_ptr<MinimalBidder>(new MinimalBidder());
    bidder->_pimpl = std::move(pimpl);
    return bidder;
  }
  return nullptr;
}

//==============================================================================
void MinimalBidder::call_for_bid(
  ParseSubmissionCallback submission_cb)
{
  _pimpl->get_submission_fn = std::move(submission_cb);
}

//==============================================================================
MinimalBidder::MinimalBidder()
{
  // do nothing
}

} // namespace bidder
} // namespace rmf_task_ros2
