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

#include <rmf_utils/catch.hpp>

#include <Transport.hpp>
#include <std_msgs/msg/string.hpp>

TEST_CASE("publish subscribe loopback", "[Transport]")
{
  rclcpp::init(0, nullptr);
  auto transport = std::make_shared<Transport>("test_transport");
  transport->spin_background();
  auto publisher = transport->create_publisher<std_msgs::msg::String>("test_topic", 10);
  auto event_job = transport->create_event_job<std_msgs::msg::String>("test_topic", 10);

  std_msgs::msg::String msg{};
  msg.data = "hello";
  auto timer = transport->create_wall_timer(std::chrono::milliseconds(100), [&publisher, &msg]()
  {
    publisher->publish(msg);
  });

  bool received = false;
  auto j = make_job([&event_job, &received](const auto& s)
  {
    run_job<std_msgs::msg::String::SharedPtr>(event_job, [&received, s](const auto&)
    {
      received = true;
      s.on_completed();
    });
  });
  run_job_blocking<bool>(j);
  REQUIRE(received);
}