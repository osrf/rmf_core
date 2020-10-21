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
#include <rclcpp/rclcpp.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rmf_task_ros2/action/ActionInterface.hpp>

namespace rmf_task_ros2 {
namespace dispatcher {

//==============================================================================
using DispatchTasks = std::map<TaskID, TaskStatusPtr>;
using DispatchTasksPtr = std::shared_ptr<DispatchTasks>;
using DispatchState = TaskStatus::State;

//==============================================================================

class Dispatcher : public std::enable_shared_from_this<Dispatcher>
{
public:
  /// Create the node of a Task Dispatcher, inherited of rclcpp node
  ///
  /// \param[in] node
  ///   The ROS 2 node to manage the Dispatching of Task
  ///
  /// \sa make()
  static std::shared_ptr<Dispatcher> make(
    const std::string dispatcher_node_name);

  /// Submit task to dispatcher node
  ///
  /// \param [in] task
  ///   Submit a task to dispatch
  ///
  /// \return task_id
  ///   self-generated task_id
  TaskID submit_task(const TaskProfile& task);

  /// Cancel task in dispatcher
  ///
  /// \param [in] task_id
  ///   Task to cancel
  ///
  /// \return true if success
  bool cancel_task(const TaskID& task_id);

  /// check the state of a submited task
  ///
  /// \param [in] task_id
  ///   request task id
  ///
  /// \return State of the task
  rmf_utils::optional<DispatchState> get_task_state(
    const TaskID& task_id);

  /// Get active tasks map list handled by dispatcher
  ///
  /// \return ptr to a map of active tasks
  const DispatchTasksPtr active_tasks() const;

  /// Get terminated tasks map list
  ///
  /// \return ptr to a map of terminated tasks
  const DispatchTasksPtr terminated_tasks() const;

  /// Get the rclcpp::Node that this dispatcher will be using for communication.
  std::shared_ptr<rclcpp::Node> node();

  /// spin dispatcher node
  void spin();

  class Implementation;

private:
  Dispatcher();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace dispatcher
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__DISPATCHER__NODE_HPP
