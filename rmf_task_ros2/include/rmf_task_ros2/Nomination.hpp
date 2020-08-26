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
// This is a class to select the best Nominee via an evaluator

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

// This is a client class for evaluator to select the best Task nominees 
// from multiple submitted task estimations (nominees)
class Nomination
{
public:

  struct Nominee
  {
    std::string fleet_name;
    std::string robot_name;
    rmf_traffic::Time start_time;
    rmf_traffic::Time end_time;
    // resources, e.g: SOC and payload
    double battery_end_soc;
  };

  using Nominees = std::vector<Nominee>;
  using NomineesPtr = std::shared_ptr<Nominees>;

  /// Constructor
  /// \param[in] input submissions of all potential nominees
  Nomination(NomineesPtr nominees)
    : _nominees(std::move(nominees))
  {
    // Do Nothing
  }

  /// A pure abstract interface class for choosing the best nominee.
  class Evaluator
  {
  public:

    /// Given a list of nominee, choose the one that is the "best". It is up to
    /// the implementation of the Evaluator to decide how to rank.
    virtual std::size_t choose(const NomineesPtr nominees) const = 0;

    virtual ~Evaluator() = default;
  };

  /// Get the best winner from nominees
  ///
  /// \param[in] Evaluator 
  /// \return Best chosen Nominee
  Nominee evaluate(const Evaluator& evaluator)
  {
    const std::size_t choice = evaluator.choose(_nominees);
    assert(choice < _nominees->size());
    return (*_nominees)[choice];
  }

  // Conversion utils for bidder: (TODO) a better way?
  static DispatchProposal convert_msg(const Nominee& nominee)
  {
    DispatchProposal proposal;
    proposal.fleet_name = nominee.fleet_name;
    proposal.robot_name = nominee.robot_name;
    // TODO: convert time and resources
    return proposal;
  };

  // Conversion utils for dispatcher
  static Nominee convert_msg(const DispatchProposal& proposal)
  {
    Nominee nominee;
    nominee.fleet_name = proposal.fleet_name;
    nominee.robot_name = proposal.robot_name;
    // TODO: convert time and resources
    return nominee;
  };

private:

  NomineesPtr _nominees;
};

//==============================================================================
// Sample Evaluator
// QuickestWithBatteryLimitEvaluator

class QuickestFinishEvaluator : public Nomination::Evaluator
{
public:
  // Documentation inherited
  std::size_t choose(const Nomination::NomineesPtr nominees) const final
  {
    std::vector<Nomination::Nominee>::iterator winner_it = nominees->begin();
    for (auto it = nominees->begin(); it != nominees->end(); ++it)
    {
      // TODO implementation Here!!! choose the least finish time 
      if (it->end_time < winner_it->end_time)
        winner_it = it;
    }
    return std::distance( nominees->begin(), winner_it );
  };
};

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__NOMINATION_HPP
