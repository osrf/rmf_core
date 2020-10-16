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

#include <rmf_task_ros2/TaskProfile.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

namespace rmf_task_ros2 {

//==============================================================================
TaskProfile convert(const TaskProfileMsg& from)
{
  TaskProfile profile;
  profile.task_id = from.task_id;
  profile.task_type = (TaskType)from.type.value;
  profile.start_time = rmf_traffic_ros2::convert(from.start_time);
  profile.submission_time = rmf_traffic_ros2::convert(from.submission_time);
  for(auto param : from.params)
  {
    profile.params[param.name] = param.value;
  }
  return profile;
}

//==============================================================================
TaskProfileMsg convert(const TaskProfile& from)
{
  TaskProfileMsg profile_msg;
  profile_msg.task_id = from.task_id;
  profile_msg.type.value = (uint8_t)from.task_type;
  profile_msg.start_time = rmf_traffic_ros2::convert(from.start_time);
  profile_msg.submission_time = rmf_traffic_ros2::convert(from.submission_time);
  for(auto param : from.params)
  {
    rmf_task_msgs::msg::BehaviorParameter param_msg; 
    param_msg.name = param.first;
    param_msg.value = param.second;
    profile_msg.params.push_back(param_msg);
  }
  return profile_msg;
}

// ==============================================================================
TaskStatus convert(const StatusMsg& from)
{
  TaskStatus status;
  status.fleet_name = from.fleet_name;
  status.task_profile = convert(from.task_profile);
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = (TaskStatus::State)from.state;
  return status;
}

// ==============================================================================
StatusMsg convert(const TaskStatus& from)
{
  StatusMsg status;
  status.fleet_name = from.fleet_name;
  status.task_profile = convert(from.task_profile);
  status.start_time = rmf_traffic_ros2::convert(from.start_time);
  status.end_time = rmf_traffic_ros2::convert(from.end_time);
  status.robot_name = from.robot_name;
  status.status = from.status;
  status.state = (uint8_t)from.state;
  return status;
}

} // namespace rmf_task_ros2
