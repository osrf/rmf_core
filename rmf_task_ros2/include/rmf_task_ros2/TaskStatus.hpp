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
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_task_ros2/Description.hpp>

#include <rmf_task_msgs/msg/task_summary.hpp>

namespace rmf_task_ros2 {

using StatusMsg = rmf_task_msgs::msg::TaskSummary;

//==============================================================================
/// This represents the Task Status of each unique task
/// \note TaskStatus struct is based on TaskSummary.msg
class TaskStatus
{
public:
  static std::shared_ptr<TaskStatus> make(
    std::string task_id,
    rmf_traffic::Time submission_time,
    ConstDescriptionPtr task_description);

  /// Get Task ID
  std::string task_id() const;

  /// Get submission time
  rmf_traffic::Time submission_time() const;

  /// Get Task Description
  ConstDescriptionPtr description() const;

  /// The fleet which will execute the task
  std::string fleet_name;

  /// The robot which will execute the task
  std::string robot_name;

  /// The estimated time which the task will start executing
  rmf_traffic::Time start_time;

  /// The estimated time which the task will finish
  rmf_traffic::Time end_time;

  /// Verbose status of this task
  std::string status;

  enum class State : uint8_t
  {
    Queued    = StatusMsg::STATE_QUEUED,
    Executing = StatusMsg::STATE_ACTIVE,
    Completed = StatusMsg::STATE_COMPLETED,
    Failed    = StatusMsg::STATE_FAILED,
    Canceled  = StatusMsg::STATE_CANCELED,
    Pending   = StatusMsg::STATE_PENDING
  };

  /// Current State of the task
  State state;

  /// Check if the current task is terminated
  bool is_terminated() const;

  /// This helper function is to only update status elements in TaskStatus, in
  /// which static task descriptions (e.g. id, tasktype...) will not be changed
  /// \note experimental (TODO)
  void update_from_msg(const StatusMsg& msg);

  class Implementation;
private:
  TaskStatus();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using TaskStatusPtr = std::shared_ptr<TaskStatus>;

// ==============================================================================
StatusMsg convert_status(const TaskStatus& from);

} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__TASK_STATUS_HPP
