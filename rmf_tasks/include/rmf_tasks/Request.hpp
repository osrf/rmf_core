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

#ifndef INCLUDE__RMF_TASKS__TASK_HPP
#define INCLUDE__RMF_TASKS__TASK_HPP

#include <memory>

#include <rmf_tasks/Estimate.hpp>
#include <rmf_tasks/agv/State.hpp>
#include <rmf_tasks/agv/StateConfig.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_tasks {

/// Implement this for new tasks
class Request
{
public:

  using SharedPtr = std::shared_ptr<Request>;

  // Get the id of the task
  virtual std::size_t id() const = 0;

  // Estimate the state of the robot when the task is finished along with the
  // time the robot has to wait before commencing the task
  virtual rmf_utils::optional<Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::StateConfig& state_config) const = 0;

  // Estimate the invariant component of the task's duration
  virtual rmf_traffic::Duration invariant_duration() const = 0;

  // Get the earliest start time that this task may begin
  virtual rmf_traffic::Time earliest_start_time() const = 0;

  virtual ~Request() = default;
};

} // namespace rmf_tasks

#endif // INCLUDE__RMF_TASKS__TASK_HPP
