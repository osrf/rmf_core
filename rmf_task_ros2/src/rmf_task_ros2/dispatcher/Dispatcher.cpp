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
  std::shared_ptr<action::TaskActionClient<RequestMsg,
    StatusMsg>> action_client;

  StatusCallback on_change_fn;

  DispatchTasksPtr active_dispatch_tasks;
  DispatchTasksPtr terminal_dispatch_tasks; // todo limit size
  int i = 0; // temp index for generating task_id
  double bidding_time_window;

  Implementation(std::shared_ptr<rclcpp::Node> node_)
  : node(std::move(node_))
  {
    active_dispatch_tasks = std::make_shared<DispatchTasks>();
    terminal_dispatch_tasks = std::make_shared<DispatchTasks>();

    // ros2 param
    bidding_time_window =
      node->declare_parameter<double>("bidding_time_window", 2.0);
    RCLCPP_WARN(node->get_logger(),
      " Declared Time Window Param as: %f secs", bidding_time_window);
  }

  void start()
  {
    using namespace std::placeholders;
    auctioneer->receive_bidding_result(
      std::bind(&Implementation::receive_bidding_winner_cb, this, _1, _2));
    action_client->on_terminate(
      std::bind(&Implementation::terminate_task, this, _1));
    action_client->on_change(
      std::bind(&Implementation::task_status_cb, this, _1));
  }

  TaskID submit_task(const TaskProfile& task)
  {
    auto submitted_task = task;

    // auto generate a taskid for a given submitted task
    // todo: fix a way to generate unique task_id
    submitted_task.task_id =
      // "task" + std::to_string((int)task.task_type)
      //   + "-" + std::to_string((int)(node->now().seconds()));
      std::to_string( ((int)task.task_type.type)*1000 + (i++) );

    submitted_task.submission_time = node->now();

    // add task to internal cache
    TaskStatus status;
    status.task_profile = submitted_task;
    auto new_task_status = std::make_shared<TaskStatus>(status);
    (*active_dispatch_tasks)[submitted_task.task_id] = new_task_status;

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
    if (!(*active_dispatch_tasks).count(task_id))
      return false;

    TaskProfile profile;
    {
      // make sure status will expired, todo: cleaner way
      auto cancel_task_status = (*active_dispatch_tasks)[task_id];
      profile = cancel_task_status->task_profile;

      // Cancel bidding. This will remove the bidding process
      if (cancel_task_status->state == DispatchState::Pending)
      {
        cancel_task_status->state = DispatchState::Canceled;
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
    auto pending_task_status = (*active_dispatch_tasks)[task_id];

    if (!winner)
    {
      std::cerr << " | No winner found :( " <<  std::endl;
      pending_task_status->state = DispatchState::Failed;
      terminate_task(pending_task_status);

      if (on_change_fn)
        on_change_fn(pending_task_status);

      return;
    }

    // now we know which fleet will execute the task
    std::cout << " | Found a winner! " << winner->fleet_name << std::endl;
    pending_task_status->fleet_name = winner->fleet_name;

    // add task to action server
    action_client->add_task(
      winner->fleet_name,
      pending_task_status->task_profile,
      pending_task_status);

    // Todo: this might not be untrue since task might be ignored by server
    pending_task_status->state = DispatchState::Queued;
  }

  void terminate_task(const TaskStatusPtr terminate_status)
  {
    assert(terminate_status->is_terminated());

    auto id = terminate_status->task_profile.task_id;
    RCLCPP_WARN(node->get_logger(), " Terminated Task!! ID: %s", id.c_str());

    // destroy prev status ptr and recreate one
    auto status = std::make_shared<TaskStatus>(*terminate_status);
    (*terminal_dispatch_tasks)[id] = status;
    active_dispatch_tasks->erase(id);
  }

  void task_status_cb(const TaskStatusPtr status_msg)
  {
    // check if task exist, if not add new
    auto id = status_msg->task_profile.task_id;
    if (!(*active_dispatch_tasks).count(id))
      (*active_dispatch_tasks)[id] = status_msg;

    if (on_change_fn)
      on_change_fn(status_msg);
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

  if (pimpl)
  {
    pimpl->auctioneer = bidding::Auctioneer::make(node);
    pimpl->action_client =
      action::TaskActionClient<RequestMsg, StatusMsg>::make(node);

    auto dispatcher = std::shared_ptr<Dispatcher>(new Dispatcher());
    dispatcher->_pimpl = pimpl;
    dispatcher->_pimpl->start();
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
const DispatchTasksPtr& Dispatcher::active_tasks() const
{
  return _pimpl->active_dispatch_tasks;
}

//==============================================================================
const DispatchTasksPtr& Dispatcher::terminated_tasks() const
{
  return _pimpl->terminal_dispatch_tasks;
}

//==============================================================================
void Dispatcher::on_change(StatusCallback on_change_fn)
{
  // _pimpl->action_client->on_change(on_change_fn);
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

//==============================================================================
Dispatcher::~Dispatcher()
{
  rclcpp::shutdown();
}

} // namespace dispatcher

namespace action {

template class TaskActionClient<RequestMsg, StatusMsg>;

} // namespace action

} // namespace rmf_task_ros2
