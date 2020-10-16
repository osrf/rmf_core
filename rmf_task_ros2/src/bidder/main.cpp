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

/// This is a testing bidder node script

#include <rmf_task_ros2/bidding/MinimalBidder.hpp>
#include <rmf_task_ros2/action/ActionInterface.hpp>

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
    { TaskType::Station, TaskType::Charging, TaskType::Delivery }
  };

  //============================================================================
  // Create Bidder instance

  std::shared_ptr<bidding::MinimalBidder> bidder =
    bidding::MinimalBidder::make(node, profile);

  bidder->call_for_bid(
    [](const bidding::BidNotice& notice)
    {
      // Here user will provice the best robot as a bid submission
      std::cout << "[Bidding] Providing best estimates" << std::endl;

      auto now = std::chrono::steady_clock::now();
      bidding::Submission best_robot_estimate;
      best_robot_estimate.robot_name = "dumb";
      best_robot_estimate.finish_time = rmf_traffic::time::apply_offset(now, 5);
      best_robot_estimate.prev_cost = 10.2;
      best_robot_estimate.new_cost = 13.5;
      return best_robot_estimate;
    }
  );

  //============================================================================
  // Create RMF task action server (TODO)

  std::shared_ptr<action::TaskActionServer> action_server =
    action::TaskActionServer::make(
    node, profile.fleet_name);

  action_server->register_callbacks(
    [&action_server](const TaskProfile& task_profile)
    {
      std::cout << "[Action] ~Start Queue Task: "
                << task_profile.task_id<<std::endl;

      // async on executing task
      // auto _ = std::async(std::launch::async,
      auto t = std::thread(
        [&action_server](auto profile)
        {
          TaskStatus status;
          status.task_profile = profile;
          std::cout << " [impl] Queued " << profile.task_id << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(2));
          std::cout << " [impl] Executing " << profile.task_id << std::endl;
          status.state = TaskStatus::State::Executing;
          action_server->update_status(status);

          std::this_thread::sleep_for(std::chrono::seconds(5));
          std::cout << " [impl] Completed " << profile.task_id << std::endl;
          status.state = TaskStatus::State::Completed;
          action_server->update_status(status);
        }, task_profile
      );
      t.detach();

      return true; //successs (send State::Queued)
    },
    [&action_server](const TaskProfile& task_profile)
    {
      std::cout << "[Action] ~Cancel Executing Task: "
                << task_profile.task_id<<std::endl;
      return true; //success , send State::Canceled
    }
  );

  rclcpp::spin(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Closing down task bidder node");

  rclcpp::shutdown();
}
