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

#include <rmf_traffic_ros2/blockade/Node.hpp>

#include <rmf_traffic/blockade/Moderator.hpp>

#include <rmf_traffic_msgs/msg/blockade_cancel.hpp>
#include <rmf_traffic_msgs/msg/blockade_heartbeat.hpp>
#include <rmf_traffic_msgs/msg/blockade_reached.hpp>
#include <rmf_traffic_msgs/msg/blockade_ready.hpp>
#include <rmf_traffic_msgs/msg/blockade_set.hpp>

namespace rmf_traffic_ros2 {
namespace blockade {

//==============================================================================
class BlockadeNode : public rclcpp::Node
{
public:

  BlockadeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rmf_traffic_blockade_node", options),
      moderator(std::make_shared<rmf_traffic::blockade::Moderator>())
  {

  }

  using Set = rmf_traffic_msgs::msg::BlockadeSet;
  using Ready = rmf_traffic_msgs::msg::BlockadeReady;
  using Reached = rmf_traffic_msgs::msg::BlockadeReached;
  using Cancel = rmf_traffic_msgs::msg::BlockadeCancel;
  using Checkpoint = rmf_traffic_msgs::msg::BlockadeCheckpoint;

  std::shared_ptr<rmf_traffic::blockade::Moderator> moderator;
  std::size_t last_assignment_version = 0;
};

} // namespace blockade
} // namespace rmf_traffic_ros2
