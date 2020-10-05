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

using namespace rmf_task_ros2;
using namespace rmf_task_ros2::bidding;

//==============================================================================
MinimalBidder::Profile bidder1_profile {
  "bidder1",  { TaskType::Station, TaskType::Delivery }
};
MinimalBidder::Profile bidder2_profile {
  "bidder2",  { TaskType::Delivery, TaskType::Cleaning }
};

BiddingTask bidding_task1{
  "bid1", TaskType::Station, true, {}, {}, {"place1", "place2"}
};
BiddingTask bidding_task2{
  "bid2", TaskType::Delivery, true, {}, {}, {"place1", "place2"}
};

//==============================================================================
SCENARIO("Auction with 2 Bids", "[TwoBids]")
{
  // Creating 1 auctioneer and 1 bidder
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_selfbidding");
  auto auctioneer = Auctioneer::make(node);
  auto bidder1 = MinimalBidder::make(node, bidder1_profile);
  auto bidder2 = MinimalBidder::make(node, bidder2_profile);

  // received msg
  std::string r_notice_bidder1_id = "";
  std::string r_notice_bidder2_id = "";
  std::string r_result_id = "";
  std::string r_result_winner = "";
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  bidder1->call_for_bid(
    [&r_notice_bidder1_id](const BidNotice& notice)
    {
      Submission best_robot_estimate;
      r_notice_bidder1_id = notice.task_id;
      return best_robot_estimate;
    }
  );
  bidder2->call_for_bid(
    [&r_notice_bidder2_id](const BidNotice& notice)
    {
      // TaskType should not be supported
      Submission best_robot_estimate;
      best_robot_estimate.new_cost = 2.3; // lower cost than bidder1
      r_notice_bidder2_id = notice.task_id;
      return best_robot_estimate;
    }
  );
  auctioneer->receive_bidding_result(
    [&r_result_id, &r_result_winner](
      const TaskID& task_id, const rmf_utils::optional<Submission> winner)
    {
      if (!winner) return;
      r_result_id = task_id;
      r_result_winner = winner->fleet_name;
      return;
    }
  );

  WHEN("First 'Station' Task Bid")
  {
    // start bidding
    bidding_task1.submission_time = std::chrono::steady_clock::now();
    auctioneer->start_bidding(bidding_task1);

    // todo: use spin till timeout
    executor.spin_some();
    std::this_thread::sleep_for (std::chrono::milliseconds(500)); 
    executor.spin_some();

    // Check if bidder 1 & 2 receive BidNotice
    REQUIRE(r_notice_bidder1_id == "bid1"); // this is valid
    REQUIRE(r_notice_bidder2_id == ""); // bidder2 doesnt support tasktype

    // todo: wait till timeout is reached: default 2s
    std::this_thread::sleep_for (std::chrono::milliseconds(1000));
    executor.spin_some();
    std::this_thread::sleep_for (std::chrono::milliseconds(1000));
    executor.spin_some();

    // Check if Auctioneer received Bid from bidder1
    REQUIRE(r_result_winner == "bidder1");
    REQUIRE(r_result_id == "bid1");
  }
  
  WHEN("Second 'Delivery' Task bid")
  {
    // start bidding
    bidding_task2.submission_time = std::chrono::steady_clock::now();
    auctioneer->start_bidding(bidding_task2);

    // todo: use spin till timeout
    executor.spin_some();
    std::this_thread::sleep_for (std::chrono::milliseconds(500)); 
    executor.spin_some();

    // Check if bidder 1 & 2 receive BidNotice
    REQUIRE(r_notice_bidder1_id == "bid2"); // this is valid
    REQUIRE(r_notice_bidder2_id == "bid2"); // bidder2 doesnt support tasktype

    // todo: wait till timeout is reached: default 2s
    std::this_thread::sleep_for (std::chrono::milliseconds(1000));
    executor.spin_some();
    std::this_thread::sleep_for (std::chrono::milliseconds(1000));
    executor.spin_some();

    // Check if Auctioneer received Bid from bidder1
    REQUIRE(r_result_winner == "bidder2");
    REQUIRE(r_result_id == "bid2");
  }

  rclcpp::shutdown();
}
