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

#ifndef RMF_TASK__TASK_HPP
#define RMF_TASK__TASK_HPP

#include <memory>

#include <rmf_task/Estimate.hpp>
#include <rmf_task/agv/State.hpp>
#include <rmf_task/agv/Constraints.hpp>
#include <rmf_task/Priority.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {

/// Implement this for new type of requests.
class Request
{
public:

  /// Constructor
  ///
  /// \param[in] id
  ///   The unique id for this task
  ///
  /// \param[in] earliest_start_time
  ///   The earliest time this task should begin execution. This is usually the
  ///   requested start time for the task.
  ///
  /// \param[in] priority
  ///   The priority for this task. This is provided by the CostCalculator. For
  ///   requests that do not have any priority this is a nullptr
  Request(
    std::string& id,
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority);

  /// Get the id of the task
  std::string id() const;

  /// Get the earliest start time that this task may begin
  rmf_traffic::Time earliest_start_time() const;

  /// Get the priority scheme of this task
  ConstPriorityPtr priority() const;

  /// Estimate the state of the robot when the task is finished along with the
  /// time the robot has to wait before commencing the task
  virtual std::optional<Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::Constraints& task_planning_constraints,
    const std::shared_ptr<EstimateCache> estimate_cache) const = 0;

  /// Estimate the invariant component of the task's duration
  virtual rmf_traffic::Duration invariant_duration() const = 0;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using RequestPtr = std::shared_ptr<Request>;
using ConstRequestPtr = std::shared_ptr<const Request>;

} // namespace rmf_task

#endif // RMF_TASK__REQUEST_HPP
