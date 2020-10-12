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

#ifndef RMF_TASK_ROS2__TASK_PROFILE_HPP
#define RMF_TASK_ROS2__TASK_PROFILE_HPP

#include <unordered_map>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

namespace rmf_task_ros2 {

//==============================================================================
enum class TaskType { 
  Station,
  Loop,
  Delivery,
  Charging,
  Cleaning,
  Patrol
};

//==============================================================================
using TaskProfileMsg = rmf_task_msgs::msg::TaskProfile;
using TaskParams = std::unordered_map<std::string, std::string>;
using TaskID = std::string;

//==============================================================================
struct TaskProfile
{
  TaskID task_id;
  rmf_traffic::Time submission_time;
  TaskType task_type;
  bool priority;
  TaskParams task_params;

  bool operator==(const TaskProfile& tsk) const
  {
    return (this->task_id == tsk.task_id);
  }

  bool operator<(const TaskProfile& tsk) const
  {
    return (this->task_id < tsk.task_id);
  }
};

//==============================================================================
inline TaskProfile convert(const TaskProfileMsg& from)
{
  TaskProfile profile;
  profile.task_id = from.task_id;
  profile.task_type = static_cast<TaskType>(from.type.value);
  profile.submission_time = rmf_traffic_ros2::convert(from.submission_time);
  profile.priority = from.priority;
  for(auto param : from.task_params)
  {
    profile.task_params[param.name] = param.value;
  }
  return profile;
}

//==============================================================================
// Vice Versa
inline TaskProfileMsg convert(const TaskProfile& from)
{
  TaskProfileMsg profile_msg;
  profile_msg.task_id = from.task_id;
  profile_msg.type.value = static_cast<uint8_t>(from.task_type);
  profile_msg.submission_time = rmf_traffic_ros2::convert(from.submission_time);
  profile_msg.priority = from.priority;
  for(auto param : from.task_params)
  {
    rmf_task_msgs::msg::BehaviorParameter param_msg; 
    param_msg.name = param.first;
    param_msg.value = param.second;
    profile_msg.task_params.push_back(param_msg);
  }
  return profile_msg;
}

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__TASK_PROFILE_HPP
