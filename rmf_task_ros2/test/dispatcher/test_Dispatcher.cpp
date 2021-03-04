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

#include <rclcpp/rclcpp.hpp>
#include <rmf_task_ros2/Dispatcher.hpp>

// mock Fleet Adapter to test dispatcher
#include "../../src/rmf_task_ros2/action/Server.hpp"
#include "../../src/rmf_task_ros2/bidding/MinimalBidder.hpp"

#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/srv/get_task_list.hpp>

#include <chrono>
#include <thread>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {

//==============================================================================
SCENARIO("Dispatcehr API Test", "[Dispatcher]")
{
  const auto now = std::chrono::steady_clock::now();
  const auto clean_task = description::Clean::make(now, "clean_here");
  const auto delivery_task = description::Delivery::make(
    now, "pickup", "dispenser1", "dropoff", "ingestor2", {});

  //============================================================================
  auto dispatcher = Dispatcher::init_and_make_node();

  auto spin_thread = std::thread(
    [&dispatcher]()
    {
      dispatcher->spin();
    });
  spin_thread.detach();


  WHEN("Check service interfaces")
  {
    using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
    using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
    using GetTaskListSrv = rmf_task_msgs::srv::GetTaskList;

    auto submit_client = dispatcher->node()->create_client<SubmitTaskSrv>(
      rmf_task_ros2::SubmitTaskSrvName);
    REQUIRE(submit_client->wait_for_service(std::chrono::milliseconds(0)));
    auto cancel_client = dispatcher->node()->create_client<CancelTaskSrv>(
      rmf_task_ros2::CancelTaskSrvName);
    REQUIRE(cancel_client->wait_for_service(std::chrono::milliseconds(0)));
    auto get_tasks_client = dispatcher->node()->create_client<GetTaskListSrv>(
      rmf_task_ros2::GetTaskListSrvName);
    REQUIRE(get_tasks_client->wait_for_service(std::chrono::milliseconds(0)));
  }

  WHEN("Add 2 tasks and cancel 1 task")
  {
    // add 2 tasks
    const auto id = dispatcher->submit_task(delivery_task);
    const auto id2 = dispatcher->submit_task(clean_task);
    REQUIRE(dispatcher->active_tasks().size() == 2);
    REQUIRE(dispatcher->terminated_tasks().size() == 0);
    REQUIRE(dispatcher->get_task_state(*id) == TaskStatus::State::Pending);
    REQUIRE(dispatcher->get_task_state(*id2) == TaskStatus::State::Pending);

    // cancel task during bidding
    REQUIRE(dispatcher->cancel_task(*id));
    REQUIRE(dispatcher->active_tasks().size() == 1);
    REQUIRE(dispatcher->terminated_tasks().size() == 1);
    REQUIRE(dispatcher->get_task_state(*id) == TaskStatus::State::Canceled);

    // wait task2 to fail due to no submissions from F.Adapters
    std::this_thread::sleep_for(std::chrono::milliseconds(5500));
    REQUIRE(dispatcher->get_task_state(*id2) == TaskStatus::State::Failed);
    REQUIRE(dispatcher->terminated_tasks().size() == 2);

    // check random id
    REQUIRE(!(dispatcher->get_task_state("non_existence_id")));

    // add an invalid task
    const auto null_task = Description::make_description(now, 10);
    REQUIRE(dispatcher->submit_task(null_task) == std::nullopt);
  }

  //============================================================================
  // test on change fn callback
  int change_times = 0;
  std::string test_dispatcher_received_id = "";
  dispatcher->on_change(
    [&change_times, &test_dispatcher_received_id](const TaskStatusPtr status)
    {
      test_dispatcher_received_id = status->task_id();
      change_times++;
    }
  );

  WHEN("Track Task till Bidding Failed")
  {
    // Submit first task and wait for bidding
    auto id = dispatcher->submit_task(delivery_task);
    REQUIRE(dispatcher->active_tasks().size() == 1);
    REQUIRE(dispatcher->get_task_state(*id) == TaskStatus::State::Pending);

    // Default 2s timeout, wait 3s for timetout, should fail here
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Failed);
    REQUIRE(dispatcher->terminated_tasks().size() == 1);
    REQUIRE(test_dispatcher_received_id == id);
    CHECK(change_times == 2); // add and failed

    // Submit another task
    id = dispatcher->submit_task(clean_task);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    REQUIRE(dispatcher->terminated_tasks().size() == 2);
    REQUIRE(test_dispatcher_received_id == *id);
    CHECK(change_times == 4); // add and failed x2
  }

  //============================================================================
  // Setup Mock Fleetadapter: mock bidder to test
  using TaskType = bidding::MinimalBidder::TaskType;
  auto node = dispatcher->node();
  auto bidder = bidding::MinimalBidder::make(
    node,
    "dummy_fleet",
    { TaskType::Delivery, TaskType::Clean },
    [](const rmf_task_msgs::msg::BidNotice&)
    {
      // Provide a best estimate
      bidding::MinimalBidder::Submission best_robot_estimate;
      best_robot_estimate.new_cost = 13.5;
      return best_robot_estimate;
    }
  );

  //============================================================================
  // Setup Mock Fleetadapter: action server to test
  auto action_server = action::Server::make(node, "dummy_fleet");

  bool task_canceled_flag = false;

  action_server->register_callbacks(
    // Add Task callback
    [&action_server, &task_canceled_flag](const auto& task_profile)
    {
      // Start action task
      auto t = std::thread(
        [&action_server, &task_canceled_flag](auto profile)
        {
          const auto status = TaskStatus::make(
            profile.task_id, std::chrono::steady_clock::now(), nullptr);
          status->robot_name = "dumbot";
          std::this_thread::sleep_for(std::chrono::seconds(2));

          if (task_canceled_flag)
          {
            // std::cout << "[task impl] Cancelled!" << std::endl;
            return;
          }

          // Executing
          status->state = TaskStatus::State::Executing;
          action_server->update_status(*status);
          std::this_thread::sleep_for(std::chrono::seconds(1));

          // Completed
          status->state = TaskStatus::State::Completed;
          action_server->update_status(*status);
        }, task_profile
      );
      t.detach();
      return true; //successs (send State::Queued)
    },
    // Cancel Task callback
    [&action_server, &task_canceled_flag](const auto&)
    {
      task_canceled_flag = true;
      return true; //success ,send State::Canceled when dispatcher->cancel_task
    }
  );

  //============================================================================
  WHEN("Full Dispatch cycle")
  {
    const auto id = dispatcher->submit_task(delivery_task);
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Pending);
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));

    // now should queue the task
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Queued);
    REQUIRE(dispatcher->terminated_tasks().size() == 0);
    CHECK(change_times == 2); // Pending and Queued

    std::this_thread::sleep_for(std::chrono::seconds(3));
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Completed);
    REQUIRE(dispatcher->active_tasks().size() == 0);
    REQUIRE(dispatcher->terminated_tasks().size() == 1);
    CHECK(change_times == 4); // Pending > Queued > Executing > Completed

    // Add auto generated ChargeBattery Task from fleet adapter
    const auto status = TaskStatus::make(
      "ChargeBattery10", std::chrono::steady_clock::now(), nullptr);
    status->state = TaskStatus::State::Queued;

    action_server->update_status(*status);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    CHECK(change_times == 5); // new stray charge task
    REQUIRE(dispatcher->active_tasks().size() == 1);
  }

  WHEN("Half way cancel Dispatch cycle")
  {
    const auto id = dispatcher->submit_task(clean_task);
    CHECK(dispatcher->get_task_state(*id) == TaskStatus::State::Pending);
    REQUIRE(dispatcher->active_tasks().size() == 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // cancel the task after QUEUED State
    REQUIRE(dispatcher->cancel_task(*id));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    REQUIRE(dispatcher->active_tasks().size() == 0);
    REQUIRE(dispatcher->terminated_tasks().size() == 1);
    REQUIRE(dispatcher->terminated_tasks().begin()->first == *id);
    const auto status = dispatcher->terminated_tasks().begin()->second;
    CHECK(status->state == TaskStatus::State::Canceled);
    CHECK(change_times == 3); // Pending -> Queued -> Canceled
  }

  rclcpp::shutdown();
}

} // namespace rmf_task_ros2
