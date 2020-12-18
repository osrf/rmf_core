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

#ifndef RMF_TASK__AGV__TASKPLANNER_HPP
#define RMF_TASK__AGV__TASKPLANNER_HPP

#include <rmf_task/Request.hpp>
#include <rmf_task/agv/State.hpp>
#include <rmf_task/agv/StateConfig.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <vector>
#include <memory>
#include <functional>
#include <variant>

namespace rmf_task {
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
    /// \param[in] battery_system
    ///   The battery system of the robot
    ///
    /// \param[in] motion_sink
    ///   The motion sink of the robot
    ///
    /// \param[in] device_sink
    ///   The ambient device sink of the robot
    ///
    /// \param[in] planner
    ///   The planner for a robot in this fleet
    ///
    /// \param[in] filter_type
    ///   The type of filter used for planning
    Configuration(
      rmf_battery::agv::BatterySystem battery_system,
      std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
      std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
      std::shared_ptr<rmf_traffic::agv::Planner> planner,
      FilterType filter_type= FilterType::Hash);

    /// Get the battery system
    rmf_battery::agv::BatterySystem& battery_system();

    /// Set the battery_system
    Configuration& battery_system(rmf_battery::agv::BatterySystem battery_system);

    /// Get the motion sink
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink() const;

    /// Get the ambient device sink
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink() const;

    /// Get the planner
    std::shared_ptr<rmf_traffic::agv::Planner> planner() const;    

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
    /// \param[in] request
    ///   The task request for this assignment
    ///
    /// \param[in] state
    ///   The state of the agent at the end of the assigned task
    ///
    /// \param[in] earliest_start_time
    ///   The earliest time the agent will begin exececuting this task
    Assignment(
      rmf_task::ConstRequestPtr request,
      State state,
      rmf_traffic::Time deployment_time);

    // Get the request of this task
    rmf_task::ConstRequestPtr request() const;

    // Get a const reference to the predicted state at the end of the assignment
    const State& state() const;

    // Get the time when the robot begins executing
    // this assignment
    const rmf_traffic::Time deployment_time() const;

    class Implementation;
  
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  enum class TaskPlannerError
  {
    /// None of the agents in the initial states have sufficient initial charge 
    /// to even head back to their charging stations. Manual intervention is
    /// needed to recharge one or more agents.
    low_battery,

    /// None of the agents in the initial states have sufficient battery
    /// capacity to accommodate one or more requests. This may be remedied by
    /// increasing the battery capacity or by lowering the threshold_soc in the 
    /// state configs of the agents or by modifying the original request.
    limited_capacity
  };

  /// Container for assignments for each agent
  using Assignments = std::vector<std::vector<Assignment>>;
  using Result = std::variant<Assignments, TaskPlannerError>;

  /// Constructor
  ///
  /// \param[in] config
  /// The configuration for the planner
  TaskPlanner(std::shared_ptr<Configuration> config);

  /// Get a shared pointer to the configuration of this task planner
  const std::shared_ptr<Configuration> config() const;

  /// Get the greedy planner based assignments for a set of initial states and 
  /// requests
  Result greedy_plan(
    rmf_traffic::Time time_now,
    std::vector<State> initial_states,
    std::vector<StateConfig> state_configs,
    std::vector<ConstRequestPtr> requests);

  /// Get the optimal planner based assignments for a set of initial states and 
  /// requests
  /// \note When the number of requests exceed 10 for the same start time
  /// segment, this plan may take a while to be generated. Hence, it is
  /// recommended to call plan() method and use the greedy solution for bidding.
  /// If a bid is awarded, the optimal solution may be used for assignments.
  Result optimal_plan(
    rmf_traffic::Time time_now,
    std::vector<State> initial_states,
    std::vector<StateConfig> state_configs,
    std::vector<ConstRequestPtr> requests,
    std::function<bool()> interrupter);

  /// Compute the cost of a set of assignments
  double compute_cost(const Assignments& assignments);

  /// Retrieve the task planner cache
  const std::shared_ptr<EstimateCache> estimate_cache() const;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl; 

};


} // namespace agv
} // namespace rmf_task

#endif // RMF_TASK__AGV__TASKPLANNER_HPP
