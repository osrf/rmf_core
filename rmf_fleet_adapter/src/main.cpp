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
#include "ReadOnlyFleetAdapter.hpp"

#include "utils/ParseGraph.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic/geometry/Circle.hpp>


int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  // --------------------------------------------------------------------------
  // seed data, will be replaced with arg parser soon
  std::string fleet_name = "fake_fleet";
  std::string graph_file_path = "/home/aaron/Desktop/chart_nav_4.yaml";
  double scaling = 1.0;
  auto profile = rmf_traffic::Trajectory::Profile::make_guided(
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(1.0));
  rmf_traffic::agv::VehicleTraits traits{
    {0.7*scaling, 0.5*scaling},
    {0.3*scaling, 1.5*scaling},
    profile
  };

  using FleetControlLevel = rmf_fleet::adapter::FleetControlLevel;
  FleetControlLevel control_level = FleetControlLevel::ReadOnly;

  // --------------------------------------------------------------------------

  std::shared_ptr<rmf_fleet::adapter::FleetAdapterNode> fleet_adapter_node;

  if (control_level == FleetControlLevel::FullControl)
    std::cout << "not implemented yet!" << std::endl;
  else if (control_level == FleetControlLevel::StopControl)
    std::cout << "not going to be implemented before DP2!" << std::endl;
  else
    fleet_adapter_node = rmf_fleet::adapter::ReadOnlyFleetAdapter::make(
        fleet_name, graph_file_path, traits);

  RCLCPP_INFO(fleet_adapter_node->get_logger(), "Starting Fleet Adapter");

  rclcpp::spin(fleet_adapter_node);

  RCLCPP_INFO(fleet_adapter_node->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();
}
