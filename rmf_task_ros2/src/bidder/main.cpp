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

#include <rmf_task_ros2/bidder/MinimalBidder.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace rmf_task_ros2;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::string node_name = "example_bidder" + std::string(argv[argc-1]);

  std::shared_ptr<rclcpp::Node> node = 
    rclcpp::Node::make_shared(node_name);

  RCLCPP_INFO(
    node->get_logger(),
    "Beginning example task bidder node");

  bidder::MinimalBidder bidder(node);
  bidder.add_bidder("dummybot", 
    [](const bidder::MinimalBidder::Itinerary& iti )
    {
      auto now = std::chrono::steady_clock::now();
      std::cout << "providing estimates" << std::endl;
      auto nominees = std::make_shared<Nomination::Nominees>();
      Nomination::Nominee nominee;
      nominee.robot_name = "dumb";
      nominee.end_time = rmf_traffic::time::apply_offset(now, 5);
      nominees->push_back(nominee);
      return nominees;
    },
    [](const DispatchConclusion& msg)
    {
      std::cout << "Execution!! " << msg.task_id << std::endl;
    }
  );

  rclcpp::spin(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Closing down task bidder node");

  rclcpp::shutdown();
}
