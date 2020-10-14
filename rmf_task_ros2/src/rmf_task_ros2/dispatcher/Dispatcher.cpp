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
  
  DispatchTasks active_dispatch_tasks;
  DispatchTasks terminal_dispatch_tasks; // todo limit size

  Implementation(std::shared_ptr<rclcpp::Node> node_)
  : node(std::move(node_))
  {
    // do nothing
  }

  TaskID submit_task(const TaskProfile& task)
  {
    auto submitted_task = task;
    
    // auto generate a taskid with timestamp
    const auto p1 = std::chrono::system_clock::now();
    submitted_task.task_id = "task-" +
      std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
        p1.time_since_epoch()).count());

    DispatchTask dispatch_task {
      submitted_task, DispatchState::Bidding, rmf_utils::nullopt };
    active_dispatch_tasks[submitted_task.task_id] = dispatch_task;
    bidding::BiddingTask bidding_task;
    bidding_task.task_profile = submitted_task;
    auctioneer->start_bidding(bidding_task);
    return task.task_id;
  }

  bool cancel_task(const TaskID& task_id)
  {
    // check if key doesnt exist
    if (!active_dispatch_tasks.count(task_id))
      return false;

    // todo: need to cancel bidding
    if (active_dispatch_tasks[task_id].dispatch_state == DispatchState::Bidding)
      return false;

    assert(active_dispatch_tasks[task_id].winner);
    std::future<bool> fut_task_success; // todo, confirm ack
    action_client->cancel_task(
        active_dispatch_tasks[task_id].task_profile,
        fut_task_success);
    return true;
  }

  rmf_utils::optional<DispatchState> get_task_state(
      const TaskID& task_id)
  {
    // check if key doesnt exist
    if (!active_dispatch_tasks.count(task_id))
      return rmf_utils::nullopt;

    return active_dispatch_tasks[task_id].dispatch_state;
  }

  void receive_bidding_winner_cb(
      const TaskID& task_id, 
      const rmf_utils::optional<bidding::Submission> winner)
  {
    std::cout << "[BiddingResultCallback] | task: " << task_id;
    if (!winner)
    {
      std::cerr << " | No winner found!" << std::endl;
      return;
    }
    std::cout << " | Found a winner! " << winner->fleet_name << std::endl;

    // we will initiate a task via task action here!
    active_dispatch_tasks[task_id].winner = *winner;
    
    std::future<bool> fut_task_success; // todo, confirm ack
    action_client->add_task(
        winner->fleet_name,
        active_dispatch_tasks[task_id].task_profile, 
        fut_task_success,
        std::make_shared<action::TaskStatus>(
          active_dispatch_tasks[task_id].task_status));

    // when fut is received, change task state as queued
    active_dispatch_tasks[task_id].dispatch_state = DispatchState::Queued;

    // todo: find a way to move active to termination
  }
};

//==============================================================================
std::shared_ptr<Dispatcher> Dispatcher::make(
    std::shared_ptr<rclcpp::Node> node)
{
  auto pimpl = std::shared_ptr<Implementation>(new Implementation(node));
  // auto pimpl = rmf_utils::make_unique_impl<Implementation>(node);

  if (pimpl)
  {
    pimpl->auctioneer = bidding::Auctioneer::make(node);
    pimpl->action_client = 
      action::TaskActionClient::make(node, DispatchActionTopicName);

    auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher());
    dispatcher->_pimpl = std::move(pimpl);

    // bidding result callback
    using namespace std::placeholders;
    dispatcher->_pimpl->auctioneer->receive_bidding_result(
      std::bind(&Implementation::receive_bidding_winner_cb, pimpl, _1, _2));
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
const DispatchTasks& Dispatcher::get_active_tasks()
{
  return _pimpl->active_dispatch_tasks;
}

//==============================================================================
const DispatchTasks& Dispatcher::get_terminated_tasks()
{
  return _pimpl->terminal_dispatch_tasks;
}

//==============================================================================
Dispatcher::Dispatcher()
{
  // Do Nothing
}

} // namespace dispatcher
} // namespace rmf_task_ros2
