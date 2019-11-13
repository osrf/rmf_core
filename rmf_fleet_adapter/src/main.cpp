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

#include <memory>
#include <iostream>

#include "FleetAdapterNode.hpp"
#include "FullControlFleetAdapter.hpp"
#include "StopControlFleetAdapter.hpp"
#include "NoControlFleetAdapter.hpp"

#include "utils/ParseGraph.hpp"

#include <rclcpp/rclcpp.hpp>


int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  using FleetControlLevel = rmf_fleet::adapter::FleetControlLevel;
  FleetControlLevel control_level = FleetControlLevel::NoControl;

  std::shared_ptr<rmf_fleet::adapter::FleetAdapterNode> fleet_adapter_node;

  if (control_level == FleetControlLevel::FullControl)
    fleet_adapter_node = rmf_fleet::adapter::FullControlFleetAdapter::make(
        "test_fleet");
  else if (control_level == FleetControlLevel::StopControl)
    std::cout << "not implemented yet!" << std::endl;
  else
    fleet_adapter_node = rmf_fleet::adapter::NoControlFleetAdapter::make(
        "test_fleet");

  RCLCPP_INFO(fleet_adapter_node->get_logger(), "Starting Fleet Adapter");

  rclcpp::spin(fleet_adapter_node);

  RCLCPP_INFO(fleet_adapter_node->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();
}
