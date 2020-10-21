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


#include <rmf_task_ros2/dispatcher/Dispatcher.hpp>

#include <rclcpp/node.hpp>

namespace rmf_task_ros2 {
namespace dispatcher {

//==============================================================================
class Dispatcher::Implementation
{
public:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<bidding::Auctioneer> auctioneer;
  std::shared_ptr<action::TaskActionClient> action_client;

  DispatchTasksPtr active_dispatch_tasks; // todo: mutex
  DispatchTasksPtr terminal_dispatch_tasks; // todo limit size

  Implementation(std::shared_ptr<rclcpp::Node> node_)
  : node(std::move(node_))
  {
    active_dispatch_tasks = std::make_shared<DispatchTasks>();
    terminal_dispatch_tasks = std::make_shared<DispatchTasks>();
  }

  TaskID submit_task(const TaskProfile& task)
  {
    auto submitted_task = task;

    // auto generate a taskid with timestamp
    const auto now = std::chrono::steady_clock::now();
    submitted_task.task_id = "task" + std::to_string((int)task.task_type)
      + "-" + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
          now.time_since_epoch()).count());

    submitted_task.submission_time = rmf_traffic_ros2::convert(node->now());

    // add task to internal cache
    TaskStatus status;
    status.task_profile = submitted_task;
    (*active_dispatch_tasks)[submitted_task.task_id] =
      std::make_shared<TaskStatus>(status);

    // using default 2s timewindow
    bidding::BidNotice bid_notice;
    bid_notice.task_profile = convert(submitted_task);
    bid_notice.time_window = rmf_traffic_ros2::convert(
      rmf_traffic::time::from_seconds(2.0)); // 2s timeout as default
    auctioneer->start_bidding(bid_notice);

    return submitted_task.task_id;
  }

  bool cancel_task(const TaskID& task_id)
  {
    // check if key exists
    if (!(*active_dispatch_tasks).count(task_id))
      return false;

    // Cancel bidding
    if ((*active_dispatch_tasks)[task_id]->state == DispatchState::Pending)
      terminate_task((*active_dispatch_tasks)[task_id]);

    // Cancel action
    return action_client->cancel_task(
      (*active_dispatch_tasks)[task_id]->task_profile);
  }

  rmf_utils::optional<DispatchState> get_task_state(
    const TaskID& task_id)
  {
    // check if key doesnt exist
    if ((*active_dispatch_tasks).count(task_id))
      return (*active_dispatch_tasks)[task_id]->state;

    if ((*terminal_dispatch_tasks).count(task_id))
      return (*terminal_dispatch_tasks)[task_id]->state;

    return rmf_utils::nullopt;
  }

  void receive_bidding_winner_cb(
    const TaskID& task_id,
    const rmf_utils::optional<bidding::Submission> winner)
  {
    if (!(*active_dispatch_tasks).count(task_id))
      return;

    std::cout << "[Dispatch::BiddingResultCb] | task: " << task_id;
    if (!winner)
    {
      std::cerr << " | No winner found :( " <<  std::endl;
      (*active_dispatch_tasks)[task_id]->state = DispatchState::Failed;
      terminate_task((*active_dispatch_tasks)[task_id]);
      return;
    }

    // action_client->size(); // debugging

    // now we know which fleet will execute the task
    std::cout << " | Found a winner! " << winner->fleet_name << std::endl;
    (*active_dispatch_tasks)[task_id]->fleet_name = winner->fleet_name;

    // add task to action server
    action_client->add_task(
      winner->fleet_name,
      (*active_dispatch_tasks)[task_id]->task_profile,
      (*active_dispatch_tasks)[task_id]);

    // Todo: this might not be untrue since task might be ignored by server
    (*active_dispatch_tasks)[task_id]->state = DispatchState::Queued;
  }

  void terminate_task(const TaskStatusPtr terminate_status)
  {
    assert(terminate_status->is_terminated());

    auto id = terminate_status->task_profile.task_id;
    RCLCPP_WARN(node->get_logger(), " Terminated Task!! ID: %s", id.c_str());
    
    (*terminal_dispatch_tasks)[id] = std::move((*active_dispatch_tasks)[id]);
    active_dispatch_tasks->erase(id);
    return;
  }
};

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::make(
  const std::string dispatcher_node_name)
{
  std::shared_ptr<rclcpp::Node> node = 
    rclcpp::Node::make_shared(dispatcher_node_name);

  auto pimpl = rmf_utils::make_impl<Implementation>(node);

  if (pimpl)
  {
    pimpl->auctioneer = bidding::Auctioneer::make(node);
    pimpl->action_client = action::TaskActionClient::make(node);

    auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher());
    dispatcher->_pimpl = pimpl;

    // bidding result callback
    using namespace std::placeholders;
    dispatcher->_pimpl->auctioneer->receive_bidding_result(
      std::bind(&Implementation::receive_bidding_winner_cb,
      dispatcher->_pimpl, _1, _2));
    dispatcher->_pimpl->action_client->on_terminate(
      std::bind(&Implementation::terminate_task,
      dispatcher->_pimpl, _1));
    return dispatcher;
  }
  return nullptr;
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
rmf_utils::optional<DispatchState> Dispatcher::get_task_state(
  const TaskID& task_id)
{
  return _pimpl->get_task_state(task_id);
}

//==============================================================================
const DispatchTasksPtr Dispatcher::active_tasks() const
{
  return _pimpl->active_dispatch_tasks;
}

//==============================================================================
const DispatchTasksPtr Dispatcher::terminated_tasks() const
{
  return _pimpl->terminal_dispatch_tasks;
}

//==============================================================================
std::shared_ptr<rclcpp::Node> Dispatcher::node()
{
  return _pimpl->node;
}

void Dispatcher::spin()
{
  rclcpp::spin(_pimpl->node);
}

//==============================================================================
Dispatcher::Dispatcher()
{
  // Do Nothing
}

} // namespace dispatcher
} // namespace rmf_task_ros2
