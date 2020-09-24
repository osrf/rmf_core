
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

#ifndef RMF_TASK_ROS2__DISPATCHER__NODE_HPP
#define RMF_TASK_ROS2__DISPATCHER__NODE_HPP

#include <rclcpp/node.hpp>
#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>

#include <rmf_task_msgs/msg/loop.hpp>
#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/station.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_task_ros2 {
namespace dispatcher {

//==============================================================================
class DispatcherNode 
    : public rclcpp::Node, 
      public std::enable_shared_from_this<DispatcherNode>
{
public:

  static std::shared_ptr<DispatcherNode> make_node();

  DispatcherNode();

  ~DispatcherNode(){};

private:

  using Loop = rmf_task_msgs::msg::Loop;
  using LoopSub = rclcpp::Subscription<Loop>;
  LoopSub::SharedPtr _loop_sub;

  using Delivery = rmf_task_msgs::msg::Delivery;
  using DeliverySub = rclcpp::Subscription<Delivery>;
  DeliverySub::SharedPtr _delivery_sub;

  using Station = rmf_task_msgs::msg::Station;
  using StationSub = rclcpp::Subscription<Station>;
  StationSub::SharedPtr _station_sub;

  bidding::Auctioneer _auctioneer;

};

//==============================================================================

} // namespace dispatcher
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__DISPATCHER__NODE_HPP
