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
void TaskStatus::update_from_msg(const StatusMsg& msg)
{
  fleet_name = msg.fleet_name;
  start_time = rmf_traffic_ros2::convert(msg.start_time);
  end_time = rmf_traffic_ros2::convert(msg.end_time);
  robot_name = msg.robot_name;
  status = msg.status;
  state = static_cast<TaskStatus::State>(msg.state);
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

// ==============================================================================
StatusMsg convert_status(const TaskStatus& from)
{
  StatusMsg status;
  status.fleet_name = from.fleet_name;
  status.task_id = from.task_id();  // duplication
  status.task_profile.task_id = from.task_id();
  status.task_profile.submission_time =
    rmf_traffic_ros2::convert(from.submission_time());
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = static_cast<uint32_t>(from.state);

  if (from.description())
    status.task_profile.description = from.description()->to_msg();

  return status;
}

} // namespace rmf_task_ros2
