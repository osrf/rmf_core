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
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
MinimalBidder::Profile bidder1_profile {
  "bidder1", { TaskType::Station, TaskType::Delivery }
};
MinimalBidder::Profile bidder2_profile {
  "bidder2", { TaskType::Delivery, TaskType::Cleaning }
};

rmf_traffic::Time default_time;
BidNotice bidding_task1;
TaskProfile task_profile1 { "bid1", default_time, TaskType::Station};

BidNotice bidding_task2;
TaskProfile task_profile2 { "bid2", default_time, TaskType::Delivery };

// set time window to 2s
auto timeout = rmf_traffic_ros2::convert(rmf_traffic::time::from_seconds(2.0));

//==============================================================================
SCENARIO("Auction with 2 Bids", "[TwoBids]")
{
  // Initializing bidding task
  bidding_task1.task_profile = convert(task_profile1);
  bidding_task1.time_window = timeout;
  bidding_task2.task_profile = convert(task_profile2);
  bidding_task2.time_window = timeout;

  // Creating 1 auctioneer and 1 bidder
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_selfbidding");
  auto auctioneer = Auctioneer::make(node);
  auto bidder1 = MinimalBidder::make(node, bidder1_profile);
  auto bidder2 = MinimalBidder::make(node, bidder2_profile);

  // test received msg
  rmf_utils::optional<TaskProfile> test_notice_bidder1;
  rmf_utils::optional<TaskProfile> test_notice_bidder2;
  std::string r_result_id = "";
  std::string r_result_winner = "";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  bidder1->call_for_bid(
    [&test_notice_bidder1](const BidNotice& notice)
    {
      Submission best_robot_estimate;
      test_notice_bidder1 = convert(notice.task_profile);
      return best_robot_estimate;
    }
  );
  bidder2->call_for_bid(
    [&test_notice_bidder2](const BidNotice& notice)
    {
      // TaskType should not be supported
      Submission best_robot_estimate;
      best_robot_estimate.new_cost = 2.3; // lower cost than bidder1
      test_notice_bidder2 = convert(notice.task_profile);
      return best_robot_estimate;
    }
  );
  auctioneer->receive_bidding_result(
    [&r_result_id, &r_result_winner](
      const TaskID& task_id, const rmf_utils::optional<Submission> winner)
    {
      if (!winner)
        return;
      r_result_id = task_id;
      r_result_winner = winner->fleet_name;
      return;
    }
  );

  // ROS Spin: forever incompleted future
  std::promise<void> ready_promise;
  std::shared_future<void> ready_future(ready_promise.get_future());
  // replacement of this method --->
  // std::this_thread::sleep_for (std::chrono::milliseconds(1000));
  // executor.spin_some();

  WHEN("First 'Station' Task Bid")
  {
    // start bidding
    bidding_task1.task_profile.submission_time = node->now();
    auctioneer->start_bidding(bidding_task1);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(1.0));

    // Check if bidder 1 & 2 receive BidNotice1
    REQUIRE(test_notice_bidder1);
    REQUIRE(*test_notice_bidder1 == convert(bidding_task1.task_profile));
    REQUIRE(!test_notice_bidder2); // bidder2 doesnt support tasktype

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(2.5));

    // Check if Auctioneer received Bid from bidder1
    REQUIRE(r_result_winner == "bidder1");
    REQUIRE(r_result_id == "bid1");
  }

  WHEN("Second 'Delivery' Task bid")
  {
    // start bidding
    bidding_task2.task_profile.submission_time = node->now();
    auctioneer->start_bidding(bidding_task2);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(1.0));

    // Check if bidder 1 & 2 receive BidNotice2
    auto task2_profile = convert(bidding_task2.task_profile);
    REQUIRE(*test_notice_bidder1 == task2_profile);
    REQUIRE(*test_notice_bidder2 == task2_profile);

    executor.spin_until_future_complete(ready_future,
      rmf_traffic::time::from_seconds(2.5));

    // Check if Auctioneer received Bid from bidder1
    REQUIRE(r_result_winner == "bidder2");
    REQUIRE(r_result_id == "bid2");
  }

  rclcpp::shutdown();
}

} // namespace bidding
} // namespace rmf_task_ros2
