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

#ifndef SRC__RMF_TASK_ROS2__ACTION_INTERFACE_HPP
#define SRC__RMF_TASK_ROS2__ACTION_INTERFACE_HPP

#include <rclcpp/node.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/TaskProfile.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_task_msgs/msg/dispatch_request.hpp>
#include <rmf_task_msgs/msg/dispatch_status.hpp>
#include <rmf_task_msgs/msg/dispatch_result.hpp>

namespace rmf_task_ros2 {
namespace action {

//==============================================================================
// Skeleton for a rmf_action

// todo: use template?
// template<typename RequestMsg, typename StatusMsg, typename ResultMsg> 
using RequestMsg = rmf_task_msgs::msg::DispatchRequest;
using StatusMsg = rmf_task_msgs::msg::DispatchStatus;
using ResultMsg = rmf_task_msgs::msg::DispatchResult;
using TaskMsg = rmf_task_msgs::msg::Task;

// TODO: user define? to replace TaskMsg
struct ActionTaskStatus
{
  enum class ActiveState : uint8_t
  {
    INVALID = TaskMsg::INVALID,
    QUEUED = TaskMsg::ACTIVE_QUEUED,
    EXECUTING = TaskMsg::ACTIVE_EXECUTING
  };

  rmf_traffic::Time start_time;
  rmf_traffic::Time end_time;
  std::string robot_name;
  ActiveState state;
};

//==============================================================================
// Task Action Client -- responsible of initiating a rmf task to the server. A 
// status update will be provided to the client when the task progresses. 
// Termination will be triggered when the task ends.

class TaskActionClient
{
public:

  /// initialize action client
  ///
  /// \param[in] ros2 node instance
  /// \param[in] prefix_topic for action interface
  static std::shared_ptr<TaskActionClient> make(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& prefix_topic);
  
  // TODO: Design this

  /// Provide all task status msg published by task action servers
  ///
  /// \param[in] action server id
  /// \param[in] all active tasks
  using StatusCallback = 
    std::function<void(
        const std::string& server_id, 
        const std::vector<TaskMsg>& tasks)>;

  /// Provide the task which is terminated by the action server
  ///
  /// \param[in] action server id
  /// \param[in] terminated task
  /// \param[in] task is completed or failed
  using TerminationCallback = 
    std::function<void(
        const std::string& server_id, 
        const TaskMsg& task,
        const bool success)>;

  /// Add event callback fns
  ///
  /// \param[in] periodic status callback function
  /// \param[in] event based result callback function
  void register_callbacks(
      StatusCallback status_callback_fn, 
      TerminationCallback termination_callback_fn);

  /// add a task to a targetted server
  ///
  /// \param[in] target server which will complete this task
  /// \param[in] task profile to execute
  /// \param[in] bool which indicate if add task is success
  void add_task(
      const std::string& server_id, 
      const TaskProfile& task_profile,
      std::future<bool>& add_success);
  
  /// cancel an added task
  ///
  /// \param[in] target server which will cancel this task
  /// \param[in] task profile to cancel
  /// \param[in] bool which indicate if cancel task is success
  void cancel_task(
      const std::string& server_id, 
      const TaskProfile& task_profile,
      std::future<bool>& cancel_success);

private:
  std::shared_ptr<rclcpp::Node> _node;
  StatusCallback _status_callback_fn;
  TerminationCallback _termination_callback_fn;

  rclcpp::Publisher<RequestMsg>::SharedPtr _request_msg_pub;
  rclcpp::Subscription<StatusMsg>::SharedPtr _status_msg_sub;
  rclcpp::Subscription<ResultMsg>::SharedPtr _result_msg_sub;

  // Private Constructor
  TaskActionClient( 
      std::shared_ptr<rclcpp::Node> node,
      const std::string& prefix_topic);

  // wait acknowledgment
  std::map<TaskProfile, std::promise<bool>> _task_request_fut_ack;
  std::promise<bool> _ack_promise;
};

// ==============================================================================
// Task Action Server - responsible for listening to request, and execute
// incoming task which is being requested by the client. "server_id" is key 
// to differentiate between multiple action servers.

class TaskActionServer
{
public:

  /// initialize action server
  ///
  /// \param[in] ros2 node instance
  /// \param[in] action server id (e.g. fleet adapter name)
  /// \param[in] prefix_topic for action interface
  static std::shared_ptr<TaskActionServer> make(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& server_id,
      const std::string& prefix_topic);

  using AddTaskCallback = 
      std::function<bool(const TaskProfile& task_profile)>;
  using CancelTaskCallback = 
      std::function<bool(const TaskProfile& task_profile)>;

  /// Add event callback fns
  ///
  /// \param[in] When a add task is called
  /// \param[in] when a cancel task is called
  void register_callbacks(
      AddTaskCallback add_task_cb_fn, 
      CancelTaskCallback cancel_task_cb_fn);

  /// Use this to send a status update to action client
  /// A periodic update is recommended to inform the task progress
  ///
  /// \param[in] Queued tasks
  void update_progress(const std::vector<TaskMsg>& tasks);
  
  /// use this to inform client that the task is terminated
  ///
  /// \param[in] Terminated task
  /// \param[in] success/completed
  void terminate_task(
      const TaskMsg& task, 
      const bool success);

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::string _server_id;
  AddTaskCallback _add_task_cb_fn;
  CancelTaskCallback _cancel_task_cb_fn;

  rclcpp::Subscription<RequestMsg>::SharedPtr _request_msg_sub;
  rclcpp::Publisher<StatusMsg>::SharedPtr _status_msg_pub;
  rclcpp::Publisher<ResultMsg>::SharedPtr _result_msg_pub;

  // Private Constructor
  TaskActionServer(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& server_id,
      const std::string& prefix_topic);

  void add_task_impl(const TaskProfileMsg& task_profile);

  void cancel_task_impl(const TaskProfileMsg& task_profile);
};

} // namespace action
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__ACTION_INTERFACE_HPP
