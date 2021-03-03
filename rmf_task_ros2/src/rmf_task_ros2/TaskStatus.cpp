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

#include <rmf_task_ros2/TaskStatus.hpp>
#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_task_ros2 {

// ==============================================================================
class TaskStatus::Implementation
{
public:
  Implementation()
  {}

  std::string task_id;
  rmf_traffic::Time submission_time;
  ConstDescriptionPtr description;
};

// ==============================================================================
TaskStatusPtr TaskStatus::make(
  std::string task_id,
  rmf_traffic::Time submission_time,
  ConstDescriptionPtr description)
{
  std::shared_ptr<TaskStatus> desc(new TaskStatus());
  desc->_pimpl->task_id = task_id;
  desc->_pimpl->submission_time = std::move(submission_time);
  desc->_pimpl->description = std::move(description);
  desc->state = State::Pending;
  return desc;
}

// ==============================================================================
ConstDescriptionPtr TaskStatus::description() const
{
  return _pimpl->description;
}

// ==============================================================================
std::string TaskStatus::task_id() const
{
  return _pimpl->task_id;
}

// ==============================================================================
rmf_traffic::Time TaskStatus::submission_time() const
{
  return _pimpl->submission_time;
}

// ==============================================================================
bool TaskStatus::is_terminated() const
{
  return (state == State::Failed) ||
    (state == State::Completed) ||
    (state == State::Canceled);
}

//==============================================================================
TaskStatus::TaskStatus()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

} // namespace rmf_task_ros2
