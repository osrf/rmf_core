
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

#ifndef RMF_TASK_ROS2__DISPATCHER__NODE_HPP
#define RMF_TASK_ROS2__DISPATCHER__NODE_HPP

#include <rclcpp/node.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rmf_task_ros2/action/ActionInterface.hpp>

namespace rmf_task_ros2 {
namespace dispatcher {

//==============================================================================
using DispatchTasks = std::map<TaskID, action::TaskStatusPtr>;
using DispatchTasksPtr = std::shared_ptr<DispatchTasks>;
using DispatchState = action::TaskStatus::State;

//==============================================================================

class Dispatcher: public std::enable_shared_from_this<Dispatcher>
{
public:
  /// Create the node of a Task Dispatcher, inherited of rclcpp node
  ///
  /// \return Pointer to the dispatcher node
  static std::shared_ptr<Dispatcher> make(
    std::shared_ptr<rclcpp::Node> node);
  
  /// submit task to dispatcher
  ///
  /// \param [in] task to dispatch
  /// \return generated task_id
  TaskID submit_task(const TaskProfile& task);

  /// cancel task in dispatcher
  ///
  /// \param [in] task to dispatch
  bool cancel_task(const TaskID& task_id);

  /// check status of a submited task
  ///
  /// \param [in] to identify task_id
  /// \return State of the task
  rmf_utils::optional<DispatchState> get_task_state(
      const TaskID& task_id);

  /// Get active tasks map list ref
  ///
  /// \return const ref to active tasks
  const DispatchTasksPtr get_active_tasks();

  /// Get terminated tasks map list ref
  ///
  /// \return const ref to terminated tasks
  const DispatchTasksPtr get_terminated_tasks();

  class Implementation;

private: 
  Dispatcher();
  std::shared_ptr<Implementation> _pimpl;
  // rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace dispatcher
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__DISPATCHER__NODE_HPP
