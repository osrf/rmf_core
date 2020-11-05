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
#include <rmf_task_ros2/action/ActionServer.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace rmf_task_ros2;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
    rclcpp::Node::make_shared("example_bidder");

  RCLCPP_INFO(
    node->get_logger(),
    "Beginning example task bidder node");

  bidding::MinimalBidder::Profile profile{
    "dummy_fleet",
    { TaskType::TYPE_CLEAN, TaskType::TYPE_DELIVERY }
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
      auto req_start_time =
      rmf_traffic_ros2::convert(notice.task_profile.start_time);

      bidding::Submission best_robot_estimate;
      best_robot_estimate.robot_name = "dumbot";
      best_robot_estimate.prev_cost = 10.2;
      best_robot_estimate.new_cost = 13.5;
      best_robot_estimate.finish_time =
      rmf_traffic::time::apply_offset(req_start_time, 7);
      return best_robot_estimate;
    }
  );

  //============================================================================
  // Create sample RMF task action server

  auto action_server = action::TaskActionServer<RequestMsg, StatusMsg>::make(
    node, profile.fleet_name);

  action_server->register_callbacks(
    [&action_server, &node](const TaskProfile& task_profile)
    {
      std::cout << "[Action] ~Start Queue Task: "
                << task_profile.task_id<<std::endl;

      // async on executing task
      // auto _ = std::async(std::launch::async,
      auto t = std::thread(
        [&action_server, &node](auto profile)
        {
          TaskStatus status;
          status.task_profile = profile;
          status.robot_name = "dumbot";
          status.start_time = rmf_traffic_ros2::convert(node->now());
          status.end_time =
          rmf_traffic::time::apply_offset(status.start_time, 7);

          // todo: should mock wait until start time is reached
          std::cout << " [impl] Queued " << profile.task_id << std::endl;
          action_server->update_status(status);

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
