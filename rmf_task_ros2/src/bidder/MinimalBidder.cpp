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

#include <rclcpp/node.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/Nomination.hpp>

#include <rmf_traffic/agv/Graph.hpp>

namespace rmf_task_ros2 {
namespace bidder {

//==============================================================================

class MinimalBidder: public rclcpp::Node
{
  public: 
    MinimalBidder( 
      rclcpp::Node& node, 
      std::string& fleet_adapter_name)
    {
      // create pubsub topics 
    }

    // Non-blocking abstract func, Get tasks estimation
    /// \param[in] Itinerary of the Requested Task
    /// \return All Robots Task Estimation
    virtual TaskEstimationsPtr get_tasks_estimation(  
      const Itinerary& itinerary) const = 0;

    // Todo, should we use this or above???
    using ParseTaskEstimation =
      std::function<bool(const rmf_task_msgs::msg::Delivery& request)>;

    void get_estimations_callback(ParseTaskEstimation cb);

    // set task manager to add awarded task to queue 
    std::shared_ptr<TaskManager> fleet_task_manager;

    // evaluator to use for selection
    Nomination::Evaluator evaluator;

  // TODO: Place all these under Implementation(?)
  private:

    // Callback fn when a dispatch notice is received
    void receive_dispatch_notice(const DispatchNotice& msg)
    {
      // get tasks estimates
      auto estimates = this->get_tasks_estimation(msg->itenerary)

      // Submit task estimations and get the chosen robot
      Nomination task_nomination(estimates);
      Nomination::TaskEstimate chosen_estimate = 
        task_nomination.evaluate(QuickestFinishEvaluator());

      // Submit proposal
      auto best_proposal = convert_msg(chosen_estimate);
      _dispatch_proposal_pub->publish(best_proposal);
    };

    // Callback fn when a dispatch conclusion is received
    void receive_dispatch_conclusion(const DispatchConclusion& msg)
    {
      // create a task with 
      Task task = make_tasks(msg)

      // insert task to task_manager
      fleet_task_manager->queue_task(task);
      
      // publish ack
      _dispatch_ack_pub->publish(ack_msg);
    }

    // Pub Sub 
    using DispatchNoticeSub = rclcpp::Subscription<DispatchNotice>;
    DispatchNoticeSub::SharedPtr _dispatch_notice_sub;
    
    using DispatchProposalPub = rclcpp::Publisher<DispatchProposal>;
    DispatchProposalPub::SharedPtr _dispatch_proposal_pub;

    using DispatchConclusionSub = rclcpp::Subscription<DispatchConclusion>;
    DispatchConclusionSub::SharedPtr _dispatch_conclusion_sub;

    using DispatchAckPub = rclcpp::Publisher<DispatchAck>;
    DispatchAckSub::SharedPtr _dispatch_ack_sub;
}

} // namespace bidder
} // namespace rmf_task_ros2
