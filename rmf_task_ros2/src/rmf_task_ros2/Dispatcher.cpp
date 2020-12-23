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

  DispatchTasks active_dispatch_tasks;
  DispatchTasks terminal_dispatch_tasks;
  std::size_t i = 0; // temp index for generating task_id
  double bidding_time_window;
  int terminated_tasks_max_size;


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
        rmf_task_ros2::TaskProfile task_profile;
        task_profile.task_type = request->task_type;
        task_profile.start_time = request->start_time;
        task_profile.clean = request->clean;
        task_profile.loop = request->loop;
        task_profile.delivery = request->delivery;
        task_profile.station = request->station;

        switch (request->evaluator)
        {
          using namespace rmf_task_ros2::bidding;
          case SubmitTaskSrv::Request::LOWEST_DIFF_COST_EVAL:
            {
              this->auctioneer->select_evaluator(
                std::make_shared<LeastFleetDiffCostEvaluator>());
              break;
            }
          case SubmitTaskSrv::Request::LOWEST_COST_EVAL:
            {
              this->auctioneer->select_evaluator(
                std::make_shared<LeastFleetCostEvaluator>());
              break;
            }
          case SubmitTaskSrv::Request::QUICKEST_FINISH_EVAL:
            {
              this->auctioneer->select_evaluator(
                std::make_shared<QuickestFinishEvaluator>());
              break;
            }
        }

        response->task_id = this->submit_task(task_profile);;
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

  TaskID submit_task(const TaskProfile& task)
  {
    auto submitted_task = task;

    // TODO: generate a unique task_id based on clock
    // auto generate a task_id for a given submitted task
    submitted_task.task_id =
      std::to_string( ((int)task.task_type.type)*1000 + (i++) );
    // "task" + std::to_string((int)task.task_type)
    //   + "-" + std::to_string((int)(node->now().seconds()));

    submitted_task.submission_time = node->now();

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
    auctioneer->start_bidding(bid_notice);

    return submitted_task.task_id;
  }

  bool cancel_task(const TaskID& task_id)
  {
    // check if key exists
    const auto it = active_dispatch_tasks.find(task_id);
    if (it == active_dispatch_tasks.end())
      return false;

    TaskProfile profile;
    {
      // make sure status will expired, todo: cleaner way
      const auto& cancel_task_status = it->second;
      profile = cancel_task_status->task_profile;

      // Cancel bidding. This will remove the bidding process
      if (cancel_task_status->state == TaskStatus::State::Pending)
      {
        cancel_task_status->state = TaskStatus::State::Canceled;
        terminate_task(cancel_task_status);

        if (on_change_fn)
          on_change_fn(cancel_task_status);

        return true;
      }
    }

    // Cancel action task, this will only send a cancel to FA. up to
    // the FA whether to cancel the task. On change is implemented
    // internally in action client
    return action_client->cancel_task(profile);
  }

  const rmf_utils::optional<TaskStatus::State> get_task_state(
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

    return rmf_utils::nullopt;
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
      RCLCPP_WARN(node->get_logger(), "[Dispatch::Bidding Result] task"
        "%s has no bidding valid submissions :(", task_id.c_str());
      pending_task_status->state = TaskStatus::State::Failed;
      terminate_task(pending_task_status);

      if (on_change_fn)
        on_change_fn(pending_task_status);

      return;
    }

    // now we know which fleet will execute the task
    pending_task_status->fleet_name = winner->fleet_name;

    RCLCPP_INFO(node->get_logger(), "[Dispatch::Bidding Result] task"
      "%s is accepted by Fleet adapter %s",
      task_id.c_str(), winner->fleet_name.c_str());

    // Remove previous self-generated charging task from "active_dispatch_tasks"
    // this is to prevent duplicated charging task (as certain queued charging
    // tasks are not terminated when task is reassigned).
    // TODO: a better way to impl this
    for (auto it = active_dispatch_tasks.begin();
      it != active_dispatch_tasks.end(); )
    {
      auto task_type = it->second->task_profile.task_type.type;
      bool is_fleet_name = (winner->fleet_name == it->second->fleet_name);
      bool is_charging_task =
        (task_type == rmf_task_msgs::msg::TaskType::TYPE_CHARGE_BATTERY);

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

    // Note: this might be untrue since task might be ignored by server
    pending_task_status->state = TaskStatus::State::Queued;
  }

  void terminate_task(const TaskStatusPtr terminate_status)
  {
    assert(terminate_status->is_terminated());

    // prevent terminal_dispatch_tasks from piling up meaning
    if (terminal_dispatch_tasks.size() >= terminated_tasks_max_size)
    {
      RCLCPP_WARN(node->get_logger(), 
        "Terminated tasks reached max size, remove first element");
      terminal_dispatch_tasks.erase ( terminal_dispatch_tasks.begin() );
    }

    const auto id = terminate_status->task_profile.task_id;
    RCLCPP_WARN(node->get_logger(), "Terminate Task ID: %s", id.c_str());

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
    const auto it = active_dispatch_tasks.find(status->task_profile.task_id);
    if (it != active_dispatch_tasks.end())
      it->second = status;

    if (on_change_fn)
      on_change_fn(status);
  }
};

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::init_and_make(
  const std::string dispatcher_node_name)
{
  rclcpp::init(0, nullptr);
  return make(dispatcher_node_name);
}

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::make(
  const std::string dispatcher_node_name)
{
  std::shared_ptr<rclcpp::Node> node =
    rclcpp::Node::make_shared(dispatcher_node_name);

  auto pimpl = rmf_utils::make_impl<Implementation>(node);
  pimpl->action_client = action::Client::make(node);

  auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher());
  dispatcher->_pimpl = std::move(pimpl);
  dispatcher->_pimpl->start();
  return dispatcher;
}

//==============================================================================
TaskID Dispatcher::submit_task(const TaskProfile& task)
{
  return _pimpl->submit_task(task);
}

//==============================================================================
bool Dispatcher::cancel_task(const TaskID& task_id)
{
  return _pimpl->cancel_task(task_id);
}

//==============================================================================
const rmf_utils::optional<TaskStatus::State> Dispatcher::get_task_state(
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
