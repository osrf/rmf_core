/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "internal_task_planning.hpp"

namespace rmf_task {
namespace agv {

// ============================================================================
std::shared_ptr<Candidates> Candidates::make(
  const std::vector<State>& initial_states,
  const std::vector<Constraints>& constraints_set,
  const rmf_task::Request& request,
  const rmf_task::requests::ChargeBattery& charge_battery_request,
  const std::shared_ptr<EstimateCache> estimate_cache,
  TaskPlanner::TaskPlannerError& error)
{
  Map initial_map;
  for (std::size_t i = 0; i < initial_states.size(); ++i)
  {
    const auto& state = initial_states[i];
    const auto& constraints = constraints_set[i];
    const auto finish = request.estimate_finish(
      state, constraints, estimate_cache);
    if (finish.has_value())
    {
      initial_map.insert({
        finish.value().finish_state().finish_time(),
        Entry{
          i,
          finish.value().finish_state(),
          finish.value().wait_until(),
          state,
          false}});
    }
    else
    {
      auto battery_estimate =
        charge_battery_request.estimate_finish(
          state, constraints, estimate_cache);
      if (battery_estimate.has_value())
      {
        auto new_finish = request.estimate_finish(
          battery_estimate.value().finish_state(), constraints, estimate_cache);
        if (new_finish.has_value())
        {
          initial_map.insert(
            {new_finish.value().finish_state().finish_time(),
            Entry{
              i,
              new_finish.value().finish_state(),
              new_finish.value().wait_until(),
              state,
              true}});
        }
        else
        {
          error = TaskPlanner::TaskPlannerError::limited_capacity;
        }
        
      }
      else
      {
        // Control reaches here either if ChargeBattery::estimate_finish() was
        // called on initial state with full battery or low battery such that
        // agent is unable to make it back to the charger
        if (abs(
          state.battery_soc() - charge_battery_request.max_charge_soc()) < 1e-3) 
            error = TaskPlanner::TaskPlannerError::limited_capacity;
        else
          error = TaskPlanner::TaskPlannerError::low_battery;
      }
    }
  }

  if (initial_map.empty())
  {
    return nullptr;
  }

  std::shared_ptr<Candidates> candidates(
    new Candidates(std::move(initial_map)));
  return candidates;
}

// ============================================================================
std::shared_ptr<PendingTask> PendingTask::make(
    std::vector<rmf_task::agv::State>& initial_states,
    std::vector<rmf_task::agv::Constraints>& constraints_set,
    rmf_task::ConstRequestPtr request_,
    rmf_task::ConstRequestPtr charge_battery_request,
    std::shared_ptr<EstimateCache> estimate_cache,
    TaskPlanner::TaskPlannerError& error)
{

  auto battery_request = std::dynamic_pointer_cast<
    const rmf_task::requests::ChargeBattery>(charge_battery_request);

  const auto candidates = Candidates::make(initial_states, constraints_set,
        *request_, *battery_request, estimate_cache, error);

  if (!candidates)
    return nullptr;

  std::shared_ptr<PendingTask> pending_task(
    new PendingTask(request_, *candidates));
  return pending_task;
}

} // namespace agv
} // namespace rmf_task
