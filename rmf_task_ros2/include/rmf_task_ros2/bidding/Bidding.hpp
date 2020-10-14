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
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_task_ros2/TaskProfile.hpp>

#include <rmf_task_msgs/msg/bid_notice.hpp>
#include <rmf_task_msgs/msg/bid_proposal.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
using BidNotice = rmf_task_msgs::msg::BidNotice;
using BidProposal = rmf_task_msgs::msg::BidProposal;

//==============================================================================
struct Submission
{
  std::string fleet_name = "";
  std::string robot_name = ""; // optional
  float prev_cost = 0.0;
  float new_cost = std::numeric_limits<float>::max();
  rmf_traffic::Time finish_time;
};

//==============================================================================
struct BiddingTask
{
  TaskProfile task_profile;
  rmf_traffic::Duration time_window = rmf_traffic::time::from_seconds(2.0); // 2s
};

//==============================================================================
BidNotice convert(const BiddingTask& from);

//==============================================================================
BidProposal convert(const Submission& from);

//==============================================================================
Submission convert(const BidProposal& from);

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__BIDDING_HPP
