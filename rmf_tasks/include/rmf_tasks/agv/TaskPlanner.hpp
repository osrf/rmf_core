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

#include <rmf_utils/impl_ptr.hpp>

#include <vector>

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
      const FilterType filter_type= FilterType::Hash);

    /// Get the pointer to the ChargeBattery request
    Request::SharedPtr charge_battery_request() const;

    /// Set the pointer to the ChargeBattery request
    Configuration& charge_battery_request(Request::SharedPtr charge_battery);
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
    const std::size_t task_id() const;

    // Get a const reference to the state
    const State& state() const;

    class Implementation;
  
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  using Assignments = std::vector<std::vector<Assignment>>;

  // Forward declaration
  class Result;

  TaskPlanner(Configuration configuration);

  Result plan(
    std::vector<State> initial_states,
    std::vector<Request::SharedPtr> requests);


  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl; 

};


class TaskPlanner::Result
{
public:
  // Get the results of the greedy algorithm based planner
  Assignments& greedy_solution() const;
  

}

} // namespace agv
} // namespace rmf_tasks

#endif // RMF_TASKS__AGV__TASKPLANNER_HPP
