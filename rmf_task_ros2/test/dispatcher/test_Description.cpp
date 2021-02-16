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
#include <rmf_task_ros2/Dispatcher.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {

//==============================================================================
SCENARIO("Task Description Test", "[Description]")
{
  std::cout << "Testing Description Task Type"<< std::cout;

  using TaskDescription = rmf_task_msgs::msg::TaskDescription;
  using TaskType = rmf_task_msgs::msg::TaskType;

  const auto now = std::chrono::steady_clock::now();

  // test delivery description msg
  auto delivery = description::Delivery::make(
    now, "pick", "dis", "drop", "ing", {});

  std::cout << "[test description] Delivery out msg: "
            << delivery->pickup_place_name()
            << "  to "
            << delivery->dropoff_place_name()
            << std::endl;

  Execution::execute_task(delivery);

  REQUIRE(delivery->type() == TaskType::TYPE_DELIVERY);
  TaskDescription msg;
  auto delivery2 = description::Delivery::make_from_msg(msg);
  REQUIRE(!delivery2);

//==============================================================================
  // test loop descripttion msg
  auto loop = description::Loop::make(now, "start_yo", "end_yo", 1);
  REQUIRE(loop->type() == TaskType::TYPE_LOOP);

  auto d_loop = std::dynamic_pointer_cast<const description::Loop>(loop);
  std::cout << "[test description] Loop out msg: "
            << loop->start_name()
            << "  to "
            << loop->finish_name()
            << std::endl;
  auto loop2 = description::Loop::make_from_msg(msg);
  REQUIRE(!loop2);
  Execution::execute_task(loop);

//==============================================================================
  // test clean descripttion msg
  auto clean = description::Clean::make(now, "clean_here");
  REQUIRE(clean->type() == TaskType::TYPE_CLEAN);
  auto clean2 = description::Loop::make_from_msg(msg);
  REQUIRE(!clean2);
}

} // namespace rmf_task_ros2
