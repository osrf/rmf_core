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

#ifndef RMF_TASK_ROS2__BIDDING_HPP
#define RMF_TASK_ROS2__BIDDING_HPP

#include <rmf_traffic/Time.hpp>
#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_msgs/msg/bid_notice.hpp>
#include <rmf_task_msgs/msg/bid_proposal.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
using BidNotice = rmf_task_msgs::msg::BidNotice;
using BidProposal = rmf_task_msgs::msg::BidProposal;
using TaskID = std::string;

//==============================================================================
struct Submission
{
  std::string fleet_name;
  std::string robot_name; // optional
  float prev_cost;
  float new_cost;
  rmf_traffic::Time start_time;
  rmf_traffic::Time end_time;
};

struct BiddingTask
{
  TaskID task_id;
  TaskType task_type;
  bool announce_all;
  std::vector<std::string> bidders;
  std::vector<bidding::Submission> submissions;
  std::vector<std::string> itinerary;
  rmf_traffic::Time submission_time;
  rmf_traffic::Duration time_window = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(2.0)); // 2s
};

//==============================================================================
static BidNotice convert(const BiddingTask& from)
{
  BidNotice notice_msg;
  notice_msg.task_id = from.task_id;
  notice_msg.type.value = static_cast<uint8_t>(from.task_type);
  notice_msg.announce_all = from.announce_all; 
  notice_msg.itinerary = from.itinerary;
  // todo: convert time;
  return notice_msg;
}

static BidProposal convert(const Submission& from)
{
  bidding::BidProposal proposal_msg;
  proposal_msg.fleet_name = from.fleet_name;
  proposal_msg.robot_name = from.robot_name;
  proposal_msg.prev_cost = from.prev_cost;
  proposal_msg.new_cost = from.new_cost;
  // todo, parse time from submission
  return proposal_msg;
}

static Submission convert(const BidProposal& from)
{
  Submission submission;
  submission.fleet_name = from.fleet_name;
  submission.robot_name = from.robot_name;
  submission.prev_cost = from.prev_cost;
  submission.new_cost = from.new_cost;
  // todo, time conversion
  return submission;
}

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__BIDDING_HPP
