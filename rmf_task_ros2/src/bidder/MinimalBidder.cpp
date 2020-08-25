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

// Skeleton for a TaskBidder

#ifndef SRC__RMF_TASK_ROS2__BIDDER_HPP
#define SRC__RMF_TASK_ROS2__BIDDER_HPP

#include <rclcpp/node.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/Nomination.hpp>
// #include <rmf_task_ros2/TaskManager.hpp>

#include <rmf_traffic/agv/Graph.hpp>

namespace rmf_task_ros2 {
namespace bidder {

//==============================================================================

class MinimalBidder
{
public: 
  MinimalBidder(std::shared_ptr<rclcpp::Node> node)
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

  using Itinerary = std::vector<std::string>;
  using ParseEstimatesCallback =
    std::function<Nomination::NomineesPtr(const Itinerary& itinerary)>;

  // Non-blocking abstract func, Get tasks estimation
  /// \param[in] Itinerary of the Requested Task
  /// \return All Robots Task Estimation
  void get_estimations_callback(ParseEstimatesCallback cb_fn)
  {
    _get_estimates_fn = std::move(cb_fn);
  };

  // TODO set task manager to add awarded task to queue 
  // std::shared_ptr<TaskManager> fleet_task_manager;

  // // evaluator to use for selection
  // Nomination::Evaluator evaluator;

private:
  // Pub Sub 
  using DispatchNoticeSub = rclcpp::Subscription<DispatchNotice>;
  DispatchNoticeSub::SharedPtr _dispatch_notice_sub;
  
  using DispatchProposalPub = rclcpp::Publisher<DispatchProposal>;
  DispatchProposalPub::SharedPtr _dispatch_proposal_pub;

  using DispatchConclusionSub = rclcpp::Subscription<DispatchConclusion>;
  DispatchConclusionSub::SharedPtr _dispatch_conclusion_sub;

  using DispatchAckPub = rclcpp::Publisher<DispatchAck>;
  DispatchAckPub::SharedPtr _dispatch_ack_pub;
  
  std::shared_ptr<rclcpp::Node> _node;
  ParseEstimatesCallback _get_estimates_fn;

  // Callback fn when a dispatch notice is received
  void receive_notice(const DispatchNotice& msg)
  {
    // get tasks estimates
    auto nominees = _get_estimates_fn(msg.itinerary);

    // Submit nominee estimations and get the chosen robot
    Nomination task_nomination(nominees);
    Nomination::Nominee chosen_estimate = 
      task_nomination.evaluate(QuickestFinishEvaluator());

    // Submit proposal
    auto best_proposal = Nomination::convert_msg(chosen_estimate);
    _dispatch_proposal_pub->publish(best_proposal);
  };

  // Callback fn when a dispatch conclusion is received
  void receive_conclusion(const DispatchConclusion& msg)
  {
    // // TODO create a task with a make func
    // Task task = make_tasks(msg)

    // // TODO insert task to task_manager
    // fleet_task_manager->queue_task(task);
    
    // publish ack
    DispatchAck ack_msg;
    ack_msg.task.task_id = msg.task_id;
    _dispatch_ack_pub->publish(ack_msg);
  };
};

} // namespace bidder
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__BIDDER_HPP
