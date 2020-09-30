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
MinimalBidder::Profile bidder_profile1 {
  "bidder1",  { TaskType::Station, TaskType::Delivery }
};
MinimalBidder::Profile bidder_profile2 {
  "bidder2",  { TaskType::Delivery, TaskType::Cleaning }
};

auto bid_now = std::chrono::steady_clock::now();

BiddingTask bidding_task1{
  "bid1", TaskType::Delivery, true, {}, {}, {"place1", "place2"}, bid_now
};

//==============================================================================
SCENARIO("Auction with 1 Bid", "[OneBid]")
{
  // Creating 1 auctioneer and 1 bidder
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_selfbidding");
  auto auctioneer = Auctioneer::make(node);
  auto bidder = MinimalBidder::make(node, bidder_profile1);
  
  // received msg
  std::string res_bid_notice = "";
  std::string res_bidder_taskid = "";
  std::string res_bidder_name = "";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  bidder->call_for_bid(
    [&res_bid_notice](const BidNotice& notice)
    {
      Submission best_robot_estimate;
      res_bid_notice = notice.task_id;
      return best_robot_estimate;
    }
  );
  auctioneer->receive_bidding_result(
    [&res_bidder_taskid, &res_bidder_name](
      const TaskID& task_id, const rmf_utils::optional<Submission> winner)
    {
      res_bidder_taskid = task_id;
      res_bidder_name = winner->fleet_name;
      return;
    }
  );

  auctioneer->start_bidding(bidding_task1);

  // todo: use spin till timeout
  executor.spin_some();
  std::this_thread::sleep_for (std::chrono::milliseconds(500)); 
  executor.spin_some();

  // Check if bidder received Bid notice
  REQUIRE( res_bid_notice == "bid1");

  // todo: wait till timeout is reached: default 2s
  std::this_thread::sleep_for (std::chrono::milliseconds(1000));
  executor.spin_some();
  std::this_thread::sleep_for (std::chrono::milliseconds(1000));
  executor.spin_some();

  // Check if Auctioneer received Bid from bidder
  REQUIRE( res_bidder_name == "bidder1");
  REQUIRE( res_bidder_taskid == "bid1");
}

//==============================================================================
SCENARIO("Auction with no Bid", "[NoBid]")
{
  REQUIRE(true); // reach timeout
}

//==============================================================================
SCENARIO("Auction with 2 Bids", "[TwoBids]")
{
  REQUIRE(true); // select the quickest robot
}
