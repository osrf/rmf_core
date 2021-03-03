/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_task_ros2/Description.hpp>
#include "../../src/rmf_task_ros2/internal_Description.hpp"

#include <rmf_task_ros2/TaskStatus.hpp>
#include <rmf_utils/catch.hpp>

#include <rmf_task_msgs/msg/task_summary.hpp>

namespace rmf_task_ros2 {

//==============================================================================
SCENARIO("TaskStatus and Description Test", "[TaskStatus]")
{
  std::cout << "[Testing Description Task Type]" << std::endl;

  using TaskDescription = rmf_task_msgs::msg::TaskDescription;
  using TaskType = rmf_task_msgs::msg::TaskType;
  using StatusMsg = rmf_task_msgs::msg::TaskSummary;

//==============================================================================
  // Check State Enum Val, to sync with msg
  REQUIRE((uint8_t)TaskStatus::State::Queued    == StatusMsg::STATE_QUEUED);
  REQUIRE((uint8_t)TaskStatus::State::Executing == StatusMsg::STATE_ACTIVE);
  REQUIRE((uint8_t)TaskStatus::State::Completed == StatusMsg::STATE_COMPLETED);
  REQUIRE((uint8_t)TaskStatus::State::Failed    == StatusMsg::STATE_FAILED);
  REQUIRE((uint8_t)TaskStatus::State::Canceled  == StatusMsg::STATE_CANCELED);
  REQUIRE((uint8_t)TaskStatus::State::Pending   == StatusMsg::STATE_PENDING);

//==============================================================================
  const auto now = std::chrono::steady_clock::now();

  // test delivery description msg
  auto delivery = description::Delivery::make(
    now, "pick", "dis", "drop", "ing", {});
  REQUIRE(delivery->type() == TaskType::TYPE_DELIVERY);

  // By default this will be a Station Type, which is not supported
  TaskDescription msg;

  auto delivery2 = description::make_delivery_from_msg(msg);
  REQUIRE(!delivery2);

  // Create TaskStatus Object
  auto delivery_status = TaskStatus::make("Delivery001", now, delivery);
  REQUIRE(delivery_status->task_id() == "Delivery001");
  REQUIRE(delivery_status->state == TaskStatus::State::Pending);
  REQUIRE(!delivery_status->is_terminated());

//==============================================================================
  // test loop descripttion msg
  auto loop = description::Loop::make(now, "start_yo", "end_yo", 1);
  REQUIRE(loop->type() == TaskType::TYPE_LOOP);

  auto d_loop = std::dynamic_pointer_cast<const description::Loop>(loop);
  auto loop2 = description::make_loop_from_msg(msg);
  REQUIRE(!loop2);

//==============================================================================
  // test clean descripttion msg
  auto clean = description::Clean::make(now, "clean_here");
  REQUIRE(clean->type() == TaskType::TYPE_CLEAN);
  auto clean2 = description::make_clean_from_msg(msg);
  REQUIRE(!clean2);
}

} // namespace rmf_task_ros2
