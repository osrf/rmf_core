
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

// Skeleton for Auctioneer

#ifndef RMF_TASK_ROS2__AUCTIONEER__NODE_HPP
#define RMF_TASK_ROS2__AUCTIONEER__NODE_HPP

#include <rclcpp/node.hpp>
#include <rmf_utils/optional.hpp>
#include <rmf_task_ros2/bidding/Bidding.hpp>
#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/bidding/Nomination.hpp>

namespace rmf_task_ros2 {

// todo: move this to a correct namespace
using Itinerary = std::vector<std::string>;

struct BiddingTask
{
  std::string task_id;
  std::vector<std::string> bidders;
  Nomination::NomineesPtr nominees
    = std::make_shared<Nomination::Nominees>();
  Itinerary itinerary;
  rmf_traffic::Time start_time;
};

//==============================================================================
namespace bidding {

class Auctioneer
{
public: 
  Auctioneer(std::shared_ptr<rclcpp::Node> node);

  /// Start a bidding process
  void start_bidding(const BiddingTask& bidding_task);

  /// callback when a bid is completed
  void bidding_result_callback(
      std::function<void(const Submission& winner)> result_callback);

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::vector<BiddingTask> _queue_bidding_tasks;
  rmf_utils::optional<rmf_traffic::Duration> _bidding_timeout;
  rclcpp::TimerBase::SharedPtr _timer;

  // Biding msg 
  using BidNoticePub = rclcpp::Publisher<BidNotice>;
  BidNoticePub::SharedPtr _bid_notice_pub;

  using BidProposalSub = rclcpp::Subscription<BidProposal>;
  BidProposalSub::SharedPtr _bid_proposal_sub;

  // Receive proposal and evaluate // todo think
  void receive_proposal(const bidding::BidProposal& msg);

  // periodic callback
  void check_bidding_process();

  // void evaluate_proposal()

};

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__AUCTIONEER__NODE_HPP
