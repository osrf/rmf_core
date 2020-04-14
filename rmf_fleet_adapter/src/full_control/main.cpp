/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "FleetAdapterNode.hpp"

#include <rmf_traffic/geometry/Circle.hpp>

#include <rclcpp/executors.hpp>

#include "../rmf_fleet_adapter/ParseArgs.hpp"

using namespace rmf_fleet_adapter;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto fleet_adapter_node = full_control::FleetAdapterNode::make();
  if (!fleet_adapter_node)
    return 1;

//  RCLCPP_INFO(fleet_adapter_node->get_logger(), "Starting Fleet Adapter");
  std::cout << "Starting [" << fleet_adapter_node->get_fleet_name() << "]" <<
    std::endl;
  rclcpp::spin(fleet_adapter_node);
  RCLCPP_INFO(fleet_adapter_node->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();
}
