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


// Skeleton for Task Nomination
// This is a class to select the best taskestimate via an evaluator

#ifndef RMF_TASK_ROS2__NOMINATION_HPP
#define RMF_TASK_ROS2__NOMINATION_HPP

#include <rmf_traffic/Time.hpp>

#include <rmf_task_msgs/msg/dispatch_notice.hpp>
#include <rmf_task_msgs/msg/dispatch_proposal.hpp>
#include <rmf_task_msgs/msg/dispatch_conclusion.hpp>
#include <rmf_task_msgs/msg/dispatch_ack.hpp>


namespace rmf_task_ros2 {

//==============================================================================


using DispatchNotice = rmf_task_msgs::msg::DispatchNotice;
using DispatchProposal = rmf_task_msgs::msg::DispatchProposal;
using DispatchConclusion = rmf_task_msgs::msg::DispatchConclusion;
using DispatchAck = rmf_task_msgs::msg::DispatchAck;

//==============================================================================

// This is a client class for evaluator to select the best Task Estimates 
// from multiple submitted task estimations
class Nomination
{
public:

  // should this be named as Nominee??
  struct TaskEstimate
  {
    std::string fleet_name;
    std::string robot_name;
    rmf_traffic::Time start_time;
    rmf_traffic::Time end_time;
    // resources, e.g: SOC and payload
    double battery_soc;
  };

  using TaskEstimatesPtr = std::shared_ptr<std::vector<TaskEstimate>>;

  // Summit task estimations
  /// \param[in] input submissions of all potential estimates
  Nomination(const TaskEstimatesPtr estimates)
  {
    _estimates = std::move(estimates);
  }

  /// A pure abstract interface class for choosing the best proposal.
  class Evaluator
  {
  public:

    /// Given a set of proposals, choose the one that is the "best". It is up to
    /// the implementation of the Evaluator to decide how to rank proposals.
    virtual std::size_t choose(const TaskEstimatesPtr estimates) const = 0;

    virtual ~Evaluator() = default;
  };

  // Get the best TaskEstimate
  /// \param[in] Itinerary of the Requested Task
  /// \return Best chosen task estimate
  TaskEstimate evaluate(const Evaluator& evaluator)
  {
    const std::size_t choice = evaluator.choose(_estimates);
    assert(choice < _estimates->size());
    return (*_estimates)[choice];
  }

  // Utils Function, (TODO) need to think of a better design
  // use by dipatcher
  static DispatchProposal convert_msg(const TaskEstimate& estimate);

  // // use by bidder
  static TaskEstimate convert_msg(const DispatchProposal& proposal);

private:

  TaskEstimatesPtr _estimates;
};

//==============================================================================
// Sample Evaluator
// QuickestWithBatteryLimitEvaluator

class QuickestFinishEvaluator : public Nomination::Evaluator
{
public:

  // Documentation inherited
  std::size_t choose(const Nomination::TaskEstimatesPtr estimates) const final
  {
    // TODO implementation Here!!! choose the least finish time 
  };

};

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__NOMINATION_HPP
