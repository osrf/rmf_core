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

#include <rmf_utils/optional.hpp>
#include <rmf_task_ros2/dispatcher/Dispatcher.hpp>
#include <rclcpp/rclcpp.hpp>

// mock system to test dispatcher
#include <rmf_task_ros2/bidding/MinimalBidder.hpp>
#include <rmf_task_ros2/action/ActionServer.hpp>

#include <chrono>
#include <thread>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {
namespace dispatcher {

// ==============================================================================
TaskProfile task_profile1;
TaskProfile task_profile2;

//==============================================================================
SCENARIO("Dispatcehr API Test", "[Dispatcher]")
{
  task_profile1.task_type.type = TaskType::TYPE_STATION;
  task_profile2.task_type.type = TaskType::TYPE_CLEAN;

//==============================================================================
  rclcpp::shutdown(); // todo: temp hack
  auto dispatcher = Dispatcher::init_and_make("test_dispatcher_node");

  auto spin_thread = std::thread(
    [&dispatcher]()
    {
      dispatcher->spin();
    });
  spin_thread.detach();

  WHEN("Add 1 and cancel task")
  {
    // add task
    auto id = dispatcher->submit_task(task_profile1);
    REQUIRE(dispatcher->active_tasks()->size() == 1);
    REQUIRE(dispatcher->terminated_tasks()->size() == 0);
    REQUIRE(dispatcher->get_task_state(id) == TaskStatus::State::Pending);

    // check random id
    REQUIRE(!(dispatcher->get_task_state("non_existence_id")));

    // cancel task
    dispatcher->cancel_task(id);
    REQUIRE(dispatcher->active_tasks()->size() == 0);
    REQUIRE(dispatcher->terminated_tasks()->size() == 1);
    REQUIRE(dispatcher->get_task_state(id) == TaskStatus::State::Canceled);
  }

//==============================================================================
  // test on change fn callback
  int change_times = 0;
  TaskProfile test_taskprofile;
  dispatcher->on_change(
    [&change_times, &test_taskprofile](const TaskStatusPtr status)
    {
      test_taskprofile = status->task_profile;
      std::cout << " On change! id > " << test_taskprofile.task_id
                << " | state >" << (int)status->state << std::endl;
      change_times++;
    }
  );

  WHEN("Track Task till Bidding Failed")
  {
    // Submit first task and wait for bidding
    auto id = dispatcher->submit_task(task_profile1);
    task_profile1.task_id = id; // update id
    REQUIRE(dispatcher->active_tasks()->size() == 1);
    REQUIRE(dispatcher->get_task_state(id) == TaskStatus::State::Pending);

    // Default 2s timeout, wait 3s for timetout, should fail here
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    REQUIRE(dispatcher->get_task_state(id) == TaskStatus::State::Failed);
    REQUIRE(dispatcher->terminated_tasks()->size() == 1);
    REQUIRE(test_taskprofile.task_id == task_profile1.task_id);
    REQUIRE(change_times == 2); // add and failed

    // Submit another task
    id = dispatcher->submit_task(task_profile2);
    task_profile2.task_id = id;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    REQUIRE(dispatcher->terminated_tasks()->size() == 2);
    REQUIRE(test_taskprofile.task_id == task_profile2.task_id);
    REQUIRE(change_times == 4); // add and failed x2
  }

//==============================================================================
  // setup a mock bidder to test
  bidding::MinimalBidder::Profile profile{
    "dummy_fleet", { TaskType::TYPE_STATION, TaskType::TYPE_CLEAN}};

  auto node = dispatcher->node();
  auto bidder = bidding::MinimalBidder::make(node, profile);

  bidder->call_for_bid(
    [](const bidding::BidNotice& notice)
    {
      std::cout << "[Bidding] Providing best estimates" << std::endl;
      bidding::Submission best_robot_estimate;
      best_robot_estimate.new_cost = 13.5;
      return best_robot_estimate;
    }
  );

//==============================================================================
  // setup the action server to test
  auto action_server = action::TaskActionServer<RequestMsg, StatusMsg>::make(
    node, profile.fleet_name);

  bool task_canceled_flag = false;

  action_server->register_callbacks(
    // Add Task callback
    [&action_server, &task_canceled_flag](const TaskProfile& task_profile)
    {
      std::cout << "[Action] ~Start Queue Task: "
                << task_profile.task_id<<std::endl;
      auto t = std::thread(
        [&action_server, &task_canceled_flag](auto profile)
        {
          TaskStatus status;
          status.task_profile = profile;
          status.robot_name = "dumbot";
          std::cout << " [task impl] Queued " << profile.task_id << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1));
          
          if(task_canceled_flag)
          {
            std::cout << "[task impl] Cancelled!" << std::endl;
            return;
          }

          std::cout << " [task impl] Executing" << profile.task_id <<std::endl;
          status.state = TaskStatus::State::Executing;
          action_server->update_status(status);
          std::this_thread::sleep_for(std::chrono::seconds(1));
          std::cout << " [task impl] Completed" << profile.task_id << std::endl;
          status.state = TaskStatus::State::Completed;
          action_server->update_status(status);
        }, task_profile
      );
      t.detach();
      return true; //successs (send State::Queued)
    },
    // Cancel Task callback
    [&action_server, &task_canceled_flag](const TaskProfile& task_profile)
    {
      task_canceled_flag = true;
      return true; //success ,send State::Canceled when dispatcher->cancel_task
    }
  );

//==============================================================================
  WHEN("Full Dispatch cycle")
  {
    std::cout << "TEST: FULL Dispatch Cycle Test!" << std::endl;
    auto id = dispatcher->submit_task(task_profile2);
    task_profile2.task_id = id;
    REQUIRE(dispatcher->get_task_state(id) == TaskStatus::State::Pending);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // now should queue the task
    REQUIRE(dispatcher->get_task_state(id) == TaskStatus::State::Queued);
    REQUIRE(dispatcher->terminated_tasks()->size() == 0);
    REQUIRE(change_times == 2); // Pending and Queued

    std::this_thread::sleep_for(std::chrono::seconds(3));
    REQUIRE(dispatcher->get_task_state(id) == TaskStatus::State::Completed);
    REQUIRE(dispatcher->terminated_tasks()->size() == 1);
    REQUIRE(change_times == 4); // Pending > Queued > Executing > Completed
  }

  WHEN("Half way cancel Dispatch cycle")
  {
    std::cout << "TEST: Cancel Dispatch Test!" << std::endl;
    auto id = dispatcher->submit_task(task_profile2);
    task_profile2.task_id = id;
    REQUIRE(dispatcher->get_task_state(id) == TaskStatus::State::Pending);
    REQUIRE(dispatcher->active_tasks()->size() == 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    std::cout << " Cancel task after Queued !!! " << id << std::endl;
    dispatcher->cancel_task(id);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    REQUIRE(dispatcher->terminated_tasks()->begin()->first == id);
    auto status = dispatcher->terminated_tasks()->begin()->second;
    REQUIRE(status->state == TaskStatus::State::Canceled);
    REQUIRE(change_times == 3); // Pending -> Queued -> Canceled
  }
}

} // namespace action
} // namespace rmf_task_ros2
