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

#ifndef SRC__RMF_TASK_ROS2__INTERNAL_ACTION_CLIENT_TPP
#define SRC__RMF_TASK_ROS2__INTERNAL_ACTION_CLIENT_TPP

namespace rmf_task_ros2 {
namespace action {

template<typename RequestMsg, typename StatusMsg>
std::shared_ptr<TaskActionClient<RequestMsg, StatusMsg>>
TaskActionClient<RequestMsg, StatusMsg>::make(
  std::shared_ptr<rclcpp::Node> node)
{
  return std::shared_ptr<TaskActionClient>(new TaskActionClient(node));
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
TaskActionClient<RequestMsg, StatusMsg>::TaskActionClient(
  std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _request_msg_pub = _node->create_publisher<RequestMsg>(
    TaskRequestTopicName, dispatch_qos);

  _status_msg_sub = _node->create_subscription<StatusMsg>(
    TaskStatusTopicName, dispatch_qos,
    [&](const std::unique_ptr<StatusMsg> msg)
    {
      auto task_id = msg->task_profile.task_id;
      // status update mode
      if (_active_task_status.count(task_id))
      {
        auto weak_status = _active_task_status[task_id].lock();

        if (!weak_status)
        {
          std::cout << "weak status has expired\n";
          _active_task_status.erase(task_id);
          return;
        }

        // TODO: hack to retain task profile and fleet name (to remove)
        auto cache_profile = weak_status->task_profile;
        // update status to ptr
        *weak_status = convert_status(*msg);
        weak_status->task_profile = cache_profile;

        if (_on_change_callback)
          _on_change_callback(weak_status);

        // if active task terminated
        if (weak_status->is_terminated())
        {
          std::cout << "[action] Done Terminated Task: "
                    << task_id << std::endl;
          _active_task_status.erase(task_id);

          if (_on_terminate_callback)
            _on_terminate_callback(weak_status);
        }
      }
      else
      {
        // will still provide onchange even if the task_id is unknown.
        std::cout << "[action] Unknown task: "  << task_id << std::endl;
        auto task_status = std::make_shared<TaskStatus>(convert_status(*msg));
        
        if (_on_change_callback)
          _on_change_callback(task_status);
        
        if(!task_status->is_terminated())
          _active_task_status[task_id] = task_status;
      }
    });
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
void TaskActionClient<RequestMsg, StatusMsg>::add_task(
  const std::string& fleet_name,
  const TaskProfile& task_profile,
  TaskStatusPtr status_ptr)
{
  // send request and wait for acknowledgement
  RequestMsg request_msg;
  request_msg.fleet_name = fleet_name;
  request_msg.task_profile = task_profile;
  request_msg.method = RequestMsg::ADD;
  _request_msg_pub->publish(request_msg);

  // save status ptr
  status_ptr->fleet_name = fleet_name;
  status_ptr->task_profile = task_profile;
  _active_task_status[task_profile.task_id] = status_ptr;
  std::cout<< " ~ Add Action Task: "<< task_profile.task_id << std::endl;

  return;
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
bool TaskActionClient<RequestMsg, StatusMsg>::cancel_task(
  const TaskProfile& task_profile)
{
  const auto task_id = task_profile.task_id;
  std::cout<< " ~ Cancel Active Task: "<< task_id << std::endl;
  
  // check if task is previously added
  if (!_active_task_status.count(task_id))
  {
    std::cerr << " ~ Not found Task: "<< task_id << std::endl;
    return false;
  }

  auto weak_status = _active_task_status[task_id].lock();

  if (!weak_status)
  {
    std::cerr << "weak status is expired, canceled failed \n";
    _active_task_status.erase(task_id);
    return false;
  }

  // send cancel
  RequestMsg request_msg;
  request_msg.fleet_name = weak_status->fleet_name;
  request_msg.task_profile = task_profile;
  request_msg.method = RequestMsg::CANCEL;
  _request_msg_pub->publish(request_msg);
  return true;
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
int TaskActionClient<RequestMsg, StatusMsg>::size()
{
  for (auto it = _active_task_status.begin(); it != _active_task_status.end(); )
  {
    if (auto weak_status = it->second.lock() )
    {
      if (weak_status->is_terminated())
        it = _active_task_status.erase(it);
      else
      {
        std::cout << "[Action Debug] active "
                  << weak_status->task_profile.task_id << "  state: "
                  << (int)weak_status->state  << std::endl;
        ++it;
      }
    }
    else
      it = _active_task_status.erase(it);
  }
  std::cout << " status: " << _active_task_status.size() << std::endl;
  return _active_task_status.size();
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
void TaskActionClient<RequestMsg, StatusMsg>::on_change(
  StatusCallback status_cb_fn)
{
  _on_change_callback = std::move(status_cb_fn);
}

//==============================================================================
template<typename RequestMsg, typename StatusMsg>
void TaskActionClient<RequestMsg, StatusMsg>::on_terminate(
  StatusCallback status_cb_fn)
{
  _on_terminate_callback = std::move(status_cb_fn);
}

} // namespace action
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__INTERNAL_ACTION_TPP
