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

#include <rmf_task_ros2/bidding/MinimalBidder.hpp>

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

  bidding::MinimalBidder::Profile profile{
    "dummy_fleet",
    {TaskType::Station, TaskType::Charging, TaskType::Cleaning}
  };
  bidding::MinimalBidder bidder(profile, node);
  
  bidder.call_for_bid(
    [](const bidding::BidNotice& notice )
    {
      // Here user will provice the best robot as a bid submission
      std::cout << "providing best estimates" << std::endl;
      
      auto now = std::chrono::steady_clock::now();
      bidding::Submission best_robot_estimate;
      best_robot_estimate.robot_name = "dumb";
      best_robot_estimate.end_time = rmf_traffic::time::apply_offset(now, 5);     
      return best_robot_estimate;
    }
  );

  // RMF task action server (TODO)
  // task_action.ActionServer action_server(profile, node);
  // action_server.request_callback(
  //   [](const Task& task)
  //   {
  //     // action: initiate/cancel
  //   }
  // );

  rclcpp::spin(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Closing down task bidder node");

  rclcpp::shutdown();
}
