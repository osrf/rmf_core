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

#ifndef SRC__RMF_TASK_ROS2__BIDDER_HPP
#define SRC__RMF_TASK_ROS2__BIDDER_HPP

#include <rclcpp/node.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/Nomination.hpp>

namespace rmf_task_ros2 {
namespace bidder {

//==============================================================================
// Skeleton for a TaskBidder

class MinimalBidder
{
public: 
  MinimalBidder(std::shared_ptr<rclcpp::Node> node);

  using Itinerary = std::vector<std::string>;
  using ParseEstimatesCallback =
    std::function<Nomination::NomineesPtr(const Itinerary& itinerary)>;
  using ExecuteTaskCallback =
    std::function<void(const DispatchConclusion dispatch_task)>;

  /// Create a bidder to bid for incoming task requests from Task Dispatcher
  ///
  /// \param[in] fleet_name
  /// \param[in] callback function to provide task estimate
  /// \param[in] callback function to start execute task
  void add_bidder(
      const std::string& fleet_name,
      ParseEstimatesCallback estimates_cb,
      ExecuteTaskCallback execute_cb);

  // // evaluator to use for selection
  // Nomination::Evaluator evaluator;

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::string _fleet_name;
  ParseEstimatesCallback _get_estimates_fn;
  ExecuteTaskCallback _execute_task_fn;

  using DispatchNoticeSub = rclcpp::Subscription<DispatchNotice>;
  DispatchNoticeSub::SharedPtr _dispatch_notice_sub;
  
  using DispatchProposalPub = rclcpp::Publisher<DispatchProposal>;
  DispatchProposalPub::SharedPtr _dispatch_proposal_pub;

  using DispatchConclusionSub = rclcpp::Subscription<DispatchConclusion>;
  DispatchConclusionSub::SharedPtr _dispatch_conclusion_sub;

  using DispatchAckPub = rclcpp::Publisher<DispatchAck>;
  DispatchAckPub::SharedPtr _dispatch_ack_pub;

  // Callback fn when a dispatch notice is received
  void receive_notice(const DispatchNotice& msg);

  // Callback fn when a dispatch conclusion is received
  void receive_conclusion(const DispatchConclusion& msg);
};

} // namespace bidder
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__BIDDER_HPP
