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

#include "TaskManager.hpp"

namespace rmf_fleet_adapter {

//==============================================================================
TaskManager::TaskManager(agv::RobotContextPtr context)
  : _context(std::move(context))
{
  _emergency_sub = _context->node()->emergency_notice()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
      [w = weak_from_this()](const auto& msg)
    {
      if (auto mgr = w.lock())
      {
        if (auto task = mgr->_active_task)
        {
          if (auto phase = task->current_phase())
            phase->emergency_alarm(msg->data);
        }
      }
    });
}

//==============================================================================
void TaskManager::queue_task(std::shared_ptr<Task> task, Start expected_finish)
{
  _queue.push_back(std::move(task));
  _expected_finish_location = std::move(expected_finish);

  std::cout << "Queuing new task for [" << _context->requester_id()
            << "]. New queue size: " << _queue.size() << std::endl;
  RCLCPP_INFO(
        _context->node()->get_logger(),
        "Queuing new task [%s] for [%s]. New queue size: %d",
         _queue.back()->id().c_str(), _context->requester_id().c_str(),
        _queue.size());

  if (!_active_task)
  {
    _begin_next_task();
  }
  else
  {
    rmf_task_msgs::msg::TaskSummary msg;
    msg.task_id = _queue.back()->id();
    msg.state = msg.STATE_QUEUED;
    this->_context->node()->task_summary()->publish(msg);
  }
}

//==============================================================================
auto TaskManager::expected_finish_location() const -> StartSet
{
  if (_expected_finish_location)
    return {*_expected_finish_location};

  return _context->location();
}

//==============================================================================
const agv::RobotContextPtr& TaskManager::context()
{
  return _context;
}

//==============================================================================
agv::ConstRobotContextPtr TaskManager::context() const
{
  return _context;
}

//==============================================================================
void TaskManager::_begin_next_task()
{
  if (_queue.empty())
  {
    _task_sub.unsubscribe();
    _expected_finish_location = rmf_utils::nullopt;

    RCLCPP_INFO(
          _context->node()->get_logger(),
          "Finished all remaining tasks for [%s]",
          _context->requester_id().c_str());

    return;
  }

  _active_task = std::move(_queue.front());
  _queue.erase(_queue.begin());

  RCLCPP_INFO(
        _context->node()->get_logger(),
        "Beginning new task [%s] for [%s]. Remaining queue size: %d",
        _active_task->id().c_str(), _context->requester_id().c_str(),
        _queue.size());

  _task_sub = _active_task->observe()
      .observe_on(rxcpp::identity_same_worker(_context->worker()))
      .subscribe(
        [this, id = _active_task->id()](Task::StatusMsg msg)
  {
    msg.task_id = id;
    _context->node()->task_summary()->publish(msg);
  },
        [this, id = _active_task->id()](std::exception_ptr e)
  {
    rmf_task_msgs::msg::TaskSummary msg;
    msg.state = msg.STATE_FAILED;

    try {
      std::rethrow_exception(e);
    }
    catch(const std::exception& e) {
      msg.status = e.what();
    }

    msg.task_id = id;
    _context->node()->task_summary()->publish(msg);
    _begin_next_task();
  },
        [this, id = _active_task->id()]()
  {
    rmf_task_msgs::msg::TaskSummary msg;
    msg.task_id = id;
    msg.state = msg.STATE_COMPLETED;
    this->_context->node()->task_summary()->publish(msg);

    _active_task = nullptr;
    _begin_next_task();
  });

  _active_task->begin();
}

} // namespace rmf_fleet_adapter
