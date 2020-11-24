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

#include <rmf_task/requests/ChargeBattery.hpp>
#include <rmf_task/requests/Clean.hpp>
#include <rmf_task/requests/Delivery.hpp>
#include <rmf_task/requests/Loop.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include "tasks/Clean.hpp"
#include "tasks/ChargeBattery.hpp"
#include "tasks/Delivery.hpp"
#include "tasks/Loop.hpp"

#include <iostream>

namespace rmf_fleet_adapter {

//==============================================================================
TaskManagerPtr TaskManager::make(agv::RobotContextPtr context)
{
  auto mgr = TaskManagerPtr(new TaskManager(std::move(context)));
  mgr->_emergency_sub = mgr->_context->node()->emergency_notice()
    .observe_on(rxcpp::identity_same_worker(mgr->_context->worker()))
    .subscribe(
      [w = mgr->weak_from_this()](const auto& msg)
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

  mgr->_timer = mgr->context()->node()->create_wall_timer(
    std::chrono::seconds(1),
    [w = mgr->weak_from_this()]()
    {
      if (auto mgr = w.lock())
      {
        mgr->_begin_next_task();
      }
    });
  return mgr;
}

//==============================================================================
TaskManager::TaskManager(agv::RobotContextPtr context)
  : _context(std::move(context))
{
  // Do nothing
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
    msg.task_profile.task_id = _queue.back()->id();
    msg.state = msg.STATE_QUEUED;
    msg.robot_name = _context->name();
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
auto TaskManager::expected_finish_state() const -> State
{
  // If an active task exists, return the estimated finish state of that task
  /// else update the current time and battery level for the state and return
  if (_active_task)
    return _context->state();

  // Update battery soc and finish time in the current state
  auto& finish_state = _context->state();
  auto location = finish_state.location();
  location.time(rmf_traffic_ros2::convert(_context->node()->now()));
  finish_state.location(location);
  
  const double current_battery_soc = _context->current_battery_soc();
  finish_state.battery_soc(current_battery_soc);
  
  return finish_state;
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
void TaskManager::set_queue(
  const std::vector<TaskManager::Assignment>& assignments)
{
  _queue.clear();
  // We use dynamic cast to determine the type of request and then call the
  // appropriate make(~) function to convert the request into a task
  for (std::size_t i = 0; i < assignments.size(); ++i)
  {
    const auto& a = assignments[i];
    auto start = _context->state().location();
    if (i != 0)
      start = assignments[i-1].state().location();
    start.time(a.deployment_time());
    rmf_task_msgs::msg::TaskType task_type_msg;

    if (const auto request =
      std::dynamic_pointer_cast<const rmf_task::requests::Clean>(a.request()))
    {
      task_type_msg.type = task_type_msg.TYPE_CLEAN;
      auto task = rmf_fleet_adapter::tasks::make_clean(
        request,
        _context,
        start,
        a.deployment_time(),
        a.state());
      
      std::lock_guard<std::mutex> guard(_mutex);

      _queue.push_back(task);
    }

    else if (const auto request =
      std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
        a.request()))
    {
      task_type_msg.type = task_type_msg.TYPE_CHARGE_BATTERY;
      const auto task = tasks::make_charge_battery(
        request,
        _context,
        start,
        a.deployment_time(),
        a.state());
      
      std::lock_guard<std::mutex> guard(_mutex);

      _queue.push_back(task);
    }

    else if (const auto request =
      std::dynamic_pointer_cast<const rmf_task::requests::Delivery>(
        a.request()))
    {
      task_type_msg.type = task_type_msg.TYPE_DELIVERY;
      const auto task = tasks::make_delivery(
        request,
        _context,
        start,
        a.deployment_time(),
        a.state());

      _queue.push_back(task);
    }

    else if (const auto request =
      std::dynamic_pointer_cast<const rmf_task::requests::Loop>(a.request()))
    {
      task_type_msg.type = task_type_msg.TYPE_LOOP;
      const auto task = tasks::make_loop(
        request,
        _context,
        start,
        a.deployment_time(),
        a.state());

      _queue.push_back(task);
    }

    else
    {
      continue;
    }

    // publish queued task
    rmf_task_msgs::msg::TaskSummary msg;
    msg.task_id = _queue.back()->id();
    msg.task_profile.task_id = _queue.back()->id();
    msg.state = msg.STATE_QUEUED;
    msg.robot_name = _context->name();
    msg.fleet_name = _context->description().owner();
    msg.task_profile.task_type = task_type_msg;
    msg.start_time = rmf_traffic_ros2::convert(
      _queue.back()->deployment_time());
    msg.start_time = rmf_traffic_ros2::convert(
      _queue.back()->finish_state().finish_time());
    this->_context->node()->task_summary()->publish(msg);
  }
}

//==============================================================================
const std::vector<rmf_task::ConstRequestPtr> TaskManager::requests() const
{
  std::vector<rmf_task::ConstRequestPtr> requests;
  requests.reserve(_queue.size());
  for (const auto& task : _queue)
  {
    if (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
      task->request()))
      continue;
    requests.push_back(task->request());
  }
  return requests;
}

//==============================================================================
void TaskManager::_begin_next_task()
{
  if (_active_task)
    return;

  if (_queue.empty())
  {
    // _task_sub.unsubscribe();
    // _expected_finish_location = rmf_utils::nullopt;

    // RCLCPP_INFO(
    //       _context->node()->get_logger(),
    //       "Finished all remaining tasks for [%s]",
    //       _context->requester_id().c_str());

    return;
  }

  std::lock_guard<std::mutex> guard(_mutex);
  const rmf_traffic::Time now = rmf_traffic_ros2::convert(
    _context->node()->now());
  const auto next_task = _queue.front();
  const auto deployment_time = next_task->deployment_time();

  if (now > deployment_time)
  {
    // Update state in RobotContext and Assign active task
    _context->state(_queue.front()->finish_state());
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
      msg.task_profile.task_id = id;
      msg.robot_name = _context->name();
      msg.fleet_name = _context->description().owner();
      msg.start_time = rmf_traffic_ros2::convert(
        _active_task->deployment_time());
      msg.end_time = rmf_traffic_ros2::convert(
        _active_task->finish_state().finish_time());
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
      msg.task_profile.task_id = id;
      msg.robot_name = _context->name();
      msg.fleet_name = _context->description().owner();
      msg.start_time = rmf_traffic_ros2::convert(
        _active_task->deployment_time());
      msg.end_time = rmf_traffic_ros2::convert(
        _active_task->finish_state().finish_time());
      _context->node()->task_summary()->publish(msg);
      // _begin_next_task();
    },
          [this, id = _active_task->id()]()
    {
      rmf_task_msgs::msg::TaskSummary msg;
      msg.task_id = id;
      msg.task_profile.task_id = id;
      msg.state = msg.STATE_COMPLETED;
      msg.robot_name = _context->name();
      msg.fleet_name = _context->description().owner();
      msg.start_time = rmf_traffic_ros2::convert(
        _active_task->deployment_time());
      msg.end_time = rmf_traffic_ros2::convert(
        _active_task->finish_state().finish_time());
      this->_context->node()->task_summary()->publish(msg);

      _active_task = nullptr;
    });

    _active_task->begin();
  }
}

//==============================================================================
void TaskManager::clear_queue()
{
  std::lock_guard<std::mutex> guard(_mutex);
  _queue.clear();
}

} // namespace rmf_fleet_adapter
