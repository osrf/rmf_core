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

#ifndef RMF_TASK_ROS2__TASK_STATUS_HPP
#define RMF_TASK_ROS2__TASK_STATUS_HPP

#include <unordered_map>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

namespace rmf_task_ros2 {

//==============================================================================
using TaskProfile = rmf_task_msgs::msg::TaskProfile;
using TaskType = rmf_task_msgs::msg::TaskType;
using TaskID = std::string;

//==============================================================================

/// \note This is a local stuct of a task status (now is based on TaskSummary)
struct TaskStatus
{
  enum class State : uint8_t
  {
    Queued    = 0, // StatusMsg::STATE_QUEUED
    Executing = 1, // StatusMsg::STATE_ACTIVE
    Completed = 2, // StatusMsg::STATE_COMPLETED
    Failed    = 3, // StatusMsg::STATE_FAILED
    Canceled  = 4, // StatusMsg::STATE_CANCELED
    Pending   = 5, // StatusMsg::STATE_PENDING
  };

  std::string fleet_name;
  TaskProfile task_profile;
  rmf_traffic::Time start_time;
  rmf_traffic::Time end_time;
  std::string robot_name;
  std::string status; // verbose msg
  State state = State::Pending; // default

  bool is_terminated() const
  {
    return (state == State::Failed) ||
      (state == State::Completed) ||
      (state == State::Canceled);
  }
};

using TaskStatusPtr = std::shared_ptr<TaskStatus>;

// ==============================================================================
template<typename StatusMsg>
TaskStatus convert_status(const StatusMsg& from)
{
  TaskStatus status;
  status.fleet_name = from.fleet_name;
  status.task_profile = from.task_profile;
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = (TaskStatus::State)from.state;
  return status;
}

// ==============================================================================
template<typename StatusMsg>
StatusMsg convert_status(const TaskStatus& from)
{
  StatusMsg status;
  status.fleet_name = from.fleet_name;
  status.task_id = from.task_profile.task_id;  // duplication
  status.task_profile = from.task_profile;
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = (uint32_t)from.state;
  return status;
}

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__TASK_STATUS_HPP
