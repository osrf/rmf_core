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

#ifndef RMF_TASKS__AGV__TASKPLANNER_HPP
#define RMF_TASKS__AGV__TASKPLANNER_HPP

#include <rmf_tasks/Request.hpp>
#include <rmf_tasks/agv/State.hpp>
#include <rmf_tasks/agv/StateConfig.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <vector>
#include <memory>
#include <functional>

namespace rmf_tasks {
namespace agv {



//==============================================================================
class TaskPlanner
{
public:

  // The type of filter used for solving the task assignment problem
  enum class FilterType
  {
    Passthrough,
    Trie,
    Hash
  };
  
  /// The Configuration class contains planning parameters that are immutable
  /// for each TaskPlanner instance and should not change in between plans.
  class Configuration
  {
  public:
    /// Constructor
    ///
    /// \param[in] charge_battery_request
    ///   A pointer to the ChargeBattery request for this AGV
    ///
    /// \param[in] filter_type
    ///   The type of filter used for planning
    Configuration(
      Request::SharedPtr charge_battery_request,
      FilterType filter_type= FilterType::Hash);

    /// Get the pointer to the ChargeBattery request
    Request::SharedPtr charge_battery_request() const;

    /// Set the pointer to the ChargeBattery request
    Configuration& charge_battery_request(Request::SharedPtr charge_battery);

    /// Get the filter type
    FilterType filter_type() const;

    /// Set the filter type
    Configuration& filter_type(FilterType filter_type);

    class Implementation;

  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class Assignment
  {
  public:

    /// Constructor
    ///
    /// \param[in] task_id
    ///   The task id for this assignment
    ///
    /// \param[in] state
    ///   The state of the agent at the end of the assigned task
    ///
    /// \param[in] earliest_start_time
    ///   The earliest time the agent will begin exececuting this task
    Assignment(
      std::size_t task_id,
      State state,
      rmf_traffic::Time earliest_start_time);

      
    // Get a const reference to the task_id
    std::size_t task_id() const;

    // Get a const reference to the state
    const State& state() const;

    // Get a const reference to the earliest start time
    const rmf_traffic::Time& earliest_start_time() const;

    class Implementation;
  
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Container for assignments for each agent
  using Assignments = std::vector<std::vector<Assignment>>;

  /// Constructor
  ///
  /// \param[in] config
  /// The configuration for the planner
  TaskPlanner(std::shared_ptr<Configuration> config);

  /// Get the greedy planner based assignments for a set of initial states and 
  /// requests
  Assignments greedy_plan(
    rmf_traffic::Time relative_start_time,
    std::vector<State> initial_states,
    std::vector<StateConfig> state_configs,
    std::vector<Request::SharedPtr> requests);

  /// Get the optimal planner based assignments for a set of initial states and 
  /// requests
  /// \note When the number of requests exceed 10 for the same start time
  /// segment, this plan may take a while to be generated. Hence, it is
  /// recommended to call plan() method and use the greedy solution for bidding.
  /// If a bid is awarded, the optimal solution may be used for assignments.
  Assignments optimal_plan(
    rmf_traffic::Time relative_start_time,
    std::vector<State> initial_states,
    std::vector<StateConfig> state_configs,
    std::vector<Request::SharedPtr> requests,
    std::function<bool()> interrupter);

  double compute_cost(
    const Assignments& assignments,
    rmf_traffic::Time relative_start_time);

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl; 

};


} // namespace agv
} // namespace rmf_tasks

#endif // RMF_TASKS__AGV__TASKPLANNER_HPP
