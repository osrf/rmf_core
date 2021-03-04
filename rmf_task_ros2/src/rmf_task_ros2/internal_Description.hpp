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

#ifndef SRC__RMF_TASK_ROS2__DESCRIPTION__INTERNAL_DESCRIPTION_HPP
#define SRC__RMF_TASK_ROS2__DESCRIPTION__INTERNAL_DESCRIPTION_HPP

#include <rmf_task_ros2/Description.hpp>
#include <rmf_task_msgs/msg/task_description.hpp>

namespace rmf_task_ros2 {
namespace description {

using TaskDescription = rmf_task_msgs::msg::TaskDescription;

//==============================================================================
std::shared_ptr<const Delivery> make_delivery_from_msg(
  const TaskDescription& msg);

//==============================================================================
std::shared_ptr<const Loop> make_loop_from_msg(
  const TaskDescription& msg);

//==============================================================================
std::shared_ptr<const Clean> make_clean_from_msg(
  const TaskDescription& msg);

//==============================================================================
TaskDescription convert(const ConstDescriptionPtr& description);

} // namespace description
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__DESCRIPTION__INTERNAL_DESCRIPTION_HPP
