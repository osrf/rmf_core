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

#include <rmf_task_ros2/Dispatcher.hpp>

#include <rclcpp/node.hpp>

#include "action/Client.hpp"

#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/srv/get_task_list.hpp>

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_task_ros2 {

//==============================================================================
class Dispatcher::Implementation
{
public:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<bidding::Auctioneer> auctioneer;
  std::shared_ptr<action::Client> action_client;

  using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
  using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
  using GetTaskListSrv = rmf_task_msgs::srv::GetTaskList;

  rclcpp::Service<SubmitTaskSrv>::SharedPtr submit_task_srv;
  rclcpp::Service<CancelTaskSrv>::SharedPtr cancel_task_srv;
  rclcpp::Service<GetTaskListSrv>::SharedPtr get_task_list_srv;

  StatusCallback on_change_fn;

  std::queue<bidding::BidNotice> queue_bidding_tasks;
  DispatchTasks active_dispatch_tasks;
  DispatchTasks terminal_dispatch_tasks;
  std::size_t task_counter = 0; // index for generating task_id
  double bidding_time_window;
  int terminated_tasks_max_size;

  std::unordered_map<std::size_t, std::string> task_type_name =
  {
    {0, "Station"},
    {1, "Loop"},
    {2, "Delivery"},
    {3, "ChargeBattery"},
    {4, "Clean"},
    {5, "Patrol"}
  };

  Implementation(std::shared_ptr<rclcpp::Node> node_)
  : node{std::move(node_)}
  {
    // ros2 param
    bidding_time_window =
      node->declare_parameter<double>("bidding_time_window", 2.0);
    RCLCPP_INFO(node->get_logger(),
      " Declared Time Window Param as: %f secs", bidding_time_window);
    terminated_tasks_max_size =
      node->declare_parameter<int>("terminated_tasks_max_size", 100);
    RCLCPP_INFO(node->get_logger(),
      " Declared Terminated Tasks Max Size Param as: %d",
      terminated_tasks_max_size);

    // Setup up stream srv interfaces
    submit_task_srv = node->create_service<SubmitTaskSrv>(
      rmf_task_ros2::SubmitTaskSrvName,
      [this](
        const std::shared_ptr<SubmitTaskSrv::Request> request,
        std::shared_ptr<SubmitTaskSrv::Response> response)
      {
        switch (request->evaluator)
        {
          using namespace rmf_task_ros2::bidding;
          case SubmitTaskSrv::Request::LOWEST_DIFF_COST_EVAL:
            this->auctioneer->select_evaluator(
              std::make_shared<LeastFleetDiffCostEvaluator>());
            break;
          case SubmitTaskSrv::Request::LOWEST_COST_EVAL:
            this->auctioneer->select_evaluator(
              std::make_shared<LeastFleetCostEvaluator>());
            break;
          case SubmitTaskSrv::Request::QUICKEST_FINISH_EVAL:
            this->auctioneer->select_evaluator(
              std::make_shared<QuickestFinishEvaluator>());
            break;
          default:
            RCLCPP_WARN(this->node->get_logger(),
            "Selected Evaluator is invalid, switch back to previous");
            break;
        }

        const auto id = this->submit_task(request->description);
        if (id == std::nullopt)
        {
          response->success = false;
          return;
        }

        response->task_id = *id;
        response->success = true;
      }
    );

    cancel_task_srv = node->create_service<CancelTaskSrv>(
      rmf_task_ros2::CancelTaskSrvName,
      [this](
        const std::shared_ptr<CancelTaskSrv::Request> request,
        std::shared_ptr<CancelTaskSrv::Response> response)
      {
        auto id = request->task_id;
        response->success = this->cancel_task(id);
      }
    );

    get_task_list_srv = node->create_service<GetTaskListSrv>(
      rmf_task_ros2::GetTaskListSrvName,
      [this](
        const std::shared_ptr<GetTaskListSrv::Request> request,
        std::shared_ptr<GetTaskListSrv::Response> response)
      {
        for (auto task : (this->active_dispatch_tasks))
        {
          response->active_tasks.push_back(
            rmf_task_ros2::convert_status(*(task.second)));
        }

        // Terminated Tasks
        for (auto task : (this->terminal_dispatch_tasks))
        {
          response->terminated_tasks.push_back(
            rmf_task_ros2::convert_status(*(task.second)));
        }
        response->success = true;
      }
    );
  }

  void start()
  {
    using namespace std::placeholders;
    auctioneer = bidding::Auctioneer::make(node,
        std::bind(&Implementation::receive_bidding_winner_cb, this, _1, _2));
    action_client->on_terminate(
      std::bind(&Implementation::terminate_task, this, _1));
    action_client->on_change(
      std::bind(&Implementation::task_status_cb, this, _1));
  }

  std::optional<TaskID> submit_task(const TaskDescription& description)
  {
    TaskProfile submitted_task;
    submitted_task.submission_time = node->now();
    submitted_task.description = description;

    const auto task_type = static_cast<std::size_t>(description.task_type.type);

    if (!task_type_name.count(task_type))
    {
      RCLCPP_ERROR(node->get_logger(), "TaskType: %d is invalid", task_type);
      return std::nullopt;
    }

    // auto generate a task_id for a given submitted task
    submitted_task.task_id =
      task_type_name[task_type] + std::to_string(task_counter++);

    RCLCPP_INFO(node->get_logger(),
      "Received Task Submission [%s]", submitted_task.task_id.c_str());

    // add task to internal cache
    TaskStatus status;
    status.task_profile = submitted_task;
    auto new_task_status = std::make_shared<TaskStatus>(status);
    active_dispatch_tasks[submitted_task.task_id] = new_task_status;

    if (on_change_fn)
      on_change_fn(new_task_status);

    bidding::BidNotice bid_notice;
    bid_notice.task_profile = submitted_task;
    bid_notice.time_window = rmf_traffic_ros2::convert(
      rmf_traffic::time::from_seconds(bidding_time_window));
    queue_bidding_tasks.push(bid_notice);

    if (queue_bidding_tasks.size() == 1)
      auctioneer->start_bidding(queue_bidding_tasks.front());

    return submitted_task.task_id;
  }

  bool cancel_task(const TaskID& task_id)
  {
    // check if key exists
    const auto it = active_dispatch_tasks.find(task_id);
    if (it == active_dispatch_tasks.end())
    {
      RCLCPP_ERROR(node->get_logger(),
        "Task [%s] is not found in active_tasks", task_id.c_str());
      return false;
    }

    RCLCPP_WARN(node->get_logger(), "Cancel task: [%s]", task_id.c_str());

    // Cancel bidding. This will remove the bidding process
    const auto& cancel_task_status = it->second;
    if (cancel_task_status->state == TaskStatus::State::Pending)
    {
      cancel_task_status->state = TaskStatus::State::Canceled;
      terminate_task(cancel_task_status);

      if (on_change_fn)
        on_change_fn(cancel_task_status);

      return true;
    }

    // Charging task doesnt support cancel task
    if (cancel_task_status->task_profile.description.task_type.type ==
      rmf_task_msgs::msg::TaskType::TYPE_CHARGE_BATTERY)
    {
      RCLCPP_ERROR(node->get_logger(), "Charging task is not cancelled-able");
      return false;
    }

    // Curently cancel can only work on Queued Task in Fleet Adapter
    if (cancel_task_status->state != TaskStatus::State::Queued)
    {
      RCLCPP_ERROR(node->get_logger(),
        "Unable to cancel task [%s] as it is not a Queued Task",
        task_id.c_str());
      return false;
    }

    // Remove previous self-generated charging task from "active_dispatch_tasks"
    // this is to prevent duplicated charging task (as certain queued charging
    // tasks are not terminated when task is reassigned).
    // TODO: a better way to impl this
    for (auto it = active_dispatch_tasks.begin();
      it != active_dispatch_tasks.end(); )
    {
      const auto type = it->second->task_profile.description.task_type.type;
      const bool is_fleet_name =
        (cancel_task_status->fleet_name == it->second->fleet_name);
      const bool is_charging_task =
        (type == rmf_task_msgs::msg::TaskType::TYPE_CHARGE_BATTERY);

      if (is_charging_task && is_fleet_name)
        it = active_dispatch_tasks.erase(it);
      else
        ++it;
    }

    // Cancel action task, this will only send a cancel to FA. up to
    // the FA whether to cancel the task. On change is implemented
    // internally in action client
    return action_client->cancel_task(cancel_task_status->task_profile);
  }

  const std::optional<TaskStatus::State> get_task_state(
    const TaskID& task_id) const
  {
    // check if taskid exists in active tasks
    auto it = active_dispatch_tasks.find(task_id);
    if (it != active_dispatch_tasks.end())
      return it->second->state;

    // check if taskid exists in terminated tasks
    it = terminal_dispatch_tasks.find(task_id);
    if (it != terminal_dispatch_tasks.end())
      return it->second->state;

    return std::nullopt;
  }

  void receive_bidding_winner_cb(
    const TaskID& task_id,
    const rmf_utils::optional<bidding::Submission> winner)
  {
    const auto it = active_dispatch_tasks.find(task_id);
    if (it == active_dispatch_tasks.end())
      return;
    const auto& pending_task_status = it->second;

    if (!winner)
    {
      RCLCPP_WARN(node->get_logger(), "Dispatcher Bidding Result: task [%s]"
        " has no submissions during bidding, Task Failed", task_id.c_str());
      pending_task_status->state = TaskStatus::State::Failed;
      terminate_task(pending_task_status);

      if (on_change_fn)
        on_change_fn(pending_task_status);

      queue_bidding_tasks.pop();
      if (!queue_bidding_tasks.empty())
        auctioneer->start_bidding(queue_bidding_tasks.front());
      return;
    }

    // now we know which fleet will execute the task
    pending_task_status->fleet_name = winner->fleet_name;

    RCLCPP_INFO(node->get_logger(), "Dispatcher Bidding Result: task [%s]"
      " is accepted by fleet adapter [%s]",
      task_id.c_str(), winner->fleet_name.c_str());

    // Remove previous self-generated charging task from "active_dispatch_tasks"
    // this is to prevent duplicated charging task (as certain queued charging
    // tasks are not terminated when task is reassigned).
    // TODO: a better way to impl this
    for (auto it = active_dispatch_tasks.begin();
      it != active_dispatch_tasks.end(); )
    {
      const auto type = it->second->task_profile.description.task_type.type;
      const bool is_fleet_name = (winner->fleet_name == it->second->fleet_name);
      const bool is_charging_task =
        (type == rmf_task_msgs::msg::TaskType::TYPE_CHARGE_BATTERY);

      if (is_charging_task && is_fleet_name)
        it = active_dispatch_tasks.erase(it);
      else
        ++it;
    }

    // add task to action server
    action_client->add_task(
      winner->fleet_name,
      pending_task_status->task_profile,
      pending_task_status);
  }

  void terminate_task(const TaskStatusPtr terminate_status)
  {
    assert(terminate_status->is_terminated());

    // prevent terminal_dispatch_tasks from piling up meaning
    if (terminal_dispatch_tasks.size() >= terminated_tasks_max_size)
    {
      RCLCPP_WARN(node->get_logger(),
        "Terminated tasks reached max size, remove earliest submited task");

      auto rm_task = terminal_dispatch_tasks.begin();
      for (auto it = rm_task++; it != terminal_dispatch_tasks.end(); it++)
      {
        const auto t1 = it->second->task_profile.submission_time;
        const auto t2 = rm_task->second->task_profile.submission_time;
        if (rmf_traffic_ros2::convert(t1) < rmf_traffic_ros2::convert(t2))
          rm_task = it;
      }
      terminal_dispatch_tasks.erase(terminal_dispatch_tasks.begin() );
    }

    const auto id = terminate_status->task_profile.task_id;

    // destroy prev status ptr and recreate one
    auto status = std::make_shared<TaskStatus>(*terminate_status);
    (terminal_dispatch_tasks)[id] = status;
    active_dispatch_tasks.erase(id);
  }

  void task_status_cb(const TaskStatusPtr status)
  {
    // This is to solve the issue that the dispatcher is not aware of those
    // "stray" tasks that are not dispatched by the dispatcher. This will add
    // the stray tasks when an unknown TaskSummary is heard.
    const std::string id = status->task_profile.task_id;
    const auto it = active_dispatch_tasks.find(id);
    if (it == active_dispatch_tasks.end())
    {
      active_dispatch_tasks[id] = status;
      RCLCPP_WARN(node->get_logger(),
        "Add previously unheard task: [%s]", id.c_str());
    }

    // check if there's a change in state for the previous completed bidding task
    // TODO, better way to impl this
    if (!queue_bidding_tasks.empty()
      && id == queue_bidding_tasks.front().task_profile.task_id)
    {
      queue_bidding_tasks.pop();
      if (!queue_bidding_tasks.empty())
        auctioneer->start_bidding(queue_bidding_tasks.front());
    }

    if (on_change_fn)
      on_change_fn(status);
  }
};

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::init_and_make_node(
  const std::string dispatcher_node_name)
{
  rclcpp::init(0, nullptr);
  return make_node(dispatcher_node_name);
}

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::make_node(
  const std::string dispatcher_node_name)
{
  return make(rclcpp::Node::make_shared(dispatcher_node_name));
}

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::make(
  const std::shared_ptr<rclcpp::Node>& node)
{
  auto pimpl = rmf_utils::make_impl<Implementation>(node);
  pimpl->action_client = action::Client::make(node);

  auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher());
  dispatcher->_pimpl = std::move(pimpl);
  dispatcher->_pimpl->start();
  return dispatcher;
}

//==============================================================================
std::optional<TaskID> Dispatcher::submit_task(
  const TaskDescription& task_description)
{
  return _pimpl->submit_task(task_description);
}

//==============================================================================
bool Dispatcher::cancel_task(const TaskID& task_id)
{
  return _pimpl->cancel_task(task_id);
}

//==============================================================================
const std::optional<TaskStatus::State> Dispatcher::get_task_state(
  const TaskID& task_id) const
{
  return _pimpl->get_task_state(task_id);
}

//==============================================================================
const Dispatcher::DispatchTasks& Dispatcher::active_tasks() const
{
  return _pimpl->active_dispatch_tasks;
}

//==============================================================================
const Dispatcher::DispatchTasks& Dispatcher::terminated_tasks() const
{
  return _pimpl->terminal_dispatch_tasks;
}

//==============================================================================
void Dispatcher::on_change(StatusCallback on_change_fn)
{
  _pimpl->on_change_fn = on_change_fn;
}

//==============================================================================
void Dispatcher::evaluator(
  std::shared_ptr<bidding::Auctioneer::Evaluator> evaluator)
{
  _pimpl->auctioneer->select_evaluator(evaluator);
}

//==============================================================================
std::shared_ptr<rclcpp::Node> Dispatcher::node()
{
  return _pimpl->node;
}

//==============================================================================
void Dispatcher::spin()
{
  rclcpp::spin(_pimpl->node);
}

//==============================================================================
Dispatcher::Dispatcher()
{
  // Do Nothing
}

} // namespace rmf_task_ros2
