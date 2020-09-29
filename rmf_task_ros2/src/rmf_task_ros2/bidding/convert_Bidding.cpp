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

#include <rmf_task_ros2/bidding/Bidding.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
BidNotice convert(const BiddingTask& from)
{
  bidding::BidNotice notice_msg;
  notice_msg.task_id = from.task_id;
  notice_msg.type.value = static_cast<uint8_t>(from.task_type);
  notice_msg.announce_all = from.announce_all; 
  notice_msg.itinerary = from.itinerary;
  notice_msg.submission_time = rmf_traffic_ros2::convert(from.submission_time);
  notice_msg.time_window = rmf_traffic_ros2::convert(from.time_window);
  return notice_msg;
}

BidProposal convert(const Submission& from)
{
  bidding::BidProposal proposal_msg;
  proposal_msg.fleet_name = from.fleet_name;
  proposal_msg.robot_name = from.robot_name;
  proposal_msg.prev_cost = from.prev_cost;
  proposal_msg.new_cost = from.new_cost;
  proposal_msg.start_time = rmf_traffic_ros2::convert(from.start_time);
  proposal_msg.end_time = rmf_traffic_ros2::convert(from.end_time);
  return proposal_msg;
}

Submission convert(const BidProposal& from)
{
  Submission submission;
  submission.fleet_name = from.fleet_name;
  submission.robot_name = from.robot_name;
  submission.prev_cost = from.prev_cost;
  submission.new_cost = from.new_cost;
  submission.start_time = rmf_traffic_ros2::convert(from.start_time);
  submission.end_time = rmf_traffic_ros2::convert(from.end_time);
  return submission;
}

} // namespace bidding
} // namespace rmf_task_ros2
