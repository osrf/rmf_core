
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
#include <rmf_task_ros2/bidding/Nomination.hpp>

namespace rmf_task_ros2 {
namespace bidding {
//==============================================================================

using BiddingTaskPtr = std::shared_ptr<BiddingTask>;

class Auctioneer: public std::enable_shared_from_this<Auctioneer>
{
public: 

  /// Create an instance of the Auctioneer. Handling all the bidding mechanism
  ///
  /// \param[in] ros2 node which will manage the pub sub
  static std::shared_ptr<Auctioneer> make(std::shared_ptr<rclcpp::Node> node);

  /// Start a bidding process 
  ///
  /// \param[in] Task to bid
  void start_bidding(const BiddingTask& bidding_task);

  /// callback which will provide the winner when a bid is concluded
  ///
  /// \param[out] single winner submission
  using BiddingResultCallback = 
    std::function<void(const Submission& winner)>;

  /// Provide a callback fn which will be called when a bid is concluded
  ///
  /// \param[in] bid result callback fn
  void receive_bidding_result(BiddingResultCallback result_callback);

private:
  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::TimerBase::SharedPtr _timer;
  BiddingResultCallback _bidding_result_callback;
  std::map<TaskID, BiddingTaskPtr> _queue_bidding_tasks;

  using BidNoticePub = rclcpp::Publisher<BidNotice>;
  BidNoticePub::SharedPtr _bid_notice_pub;

  using BidProposalSub = rclcpp::Subscription<BidProposal>;
  BidProposalSub::SharedPtr _bid_proposal_sub;
  
  // Class Constuctor
  Auctioneer(std::shared_ptr<rclcpp::Node> node);

  // Receive proposal and evaluate // todo think
  void receive_proposal(const BidProposal& msg);

  // periodic callback
  void check_bidding_process();

  // determine the winner within a bidding task instance
  void determine_winner(BiddingTaskPtr bidding_task);
};

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__AUCTIONEER__NODE_HPP
