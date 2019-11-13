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
#include "ParseGraph.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_traffic_msgs/msg/test_task_request.hpp>
#include <rmf_msgs/msg/robot_state.hpp>

bool get_arg(
    const std::vector<std::string>& args,
    const std::string& key,
    std::string& value,
    const std::string& desc,
    const bool mandatory = true)
{
  const auto key_arg = std::find(args.begin(), args.end(), key);
  if(key_arg == args.end())
  {
    // TODO(MXG): See if there's a way to use RCLCPP_ERROR here without first
    // constructing a node. If not, we could consider constructing the
    // FleetAdapterNode in two parts.
    if(mandatory)
    {
      std::cerr << "You must specify a " << desc <<" using the " << key
                << " argument!" << std::endl;
    }
    return false;
  }
  else if(key_arg+1 == args.end())
  {
    std::cerr << "The " << key << " argument must be followed by a " << desc
              << "!" << std::endl;
    return false;
  }

  value = *(key_arg+1);
  return true;
}

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string graph_file;
  if(!get_arg(args, "-g", graph_file, "graph file name"))
    return 1;

  std::string fleet_name;
  if(!get_arg(args, "-f", fleet_name, "fleet name"))
    return 1;

  std::string v_arg;
  get_arg(args, "-v", v_arg, "nominal velocity", false);
  const double v_nom = v_arg.empty()? 0.7 : std::stod(v_arg);

  std::string a_arg;
  get_arg(args, "-a", a_arg, "nominal acceleration", false);
  const double a_nom = a_arg.empty()? 0.5 : std::stod(a_arg);

  std::string r_arg;
  get_arg(args, "-r", r_arg, "tire radius", false);
  const double scaling = r_arg.empty()? 1.0 : std::stod(r_arg)/0.1;


  std::string start_wp;
  const bool has_start = get_arg(args, "-s", start_wp, "start waypoint", false);

  std::string initial_wp = start_wp;
  get_arg(args, "-i", initial_wp, "initial waypoint", false);

  std::string end_wp;
  const bool has_end = get_arg(args, "-e", end_wp, "end waypoint", false);

  if(has_start && !has_end)
  {
    std::cerr << "If you specify a start waypoint, you must also specify an "
              << "end waypoint!" << std::endl;
    return 1;
  }

  if(has_end && !has_start)
  {
    std::cerr << "If you specify an end waypoint, you must also specify a "
              << "start waypoint!";
    return 1;
  }

  // TODO(MXG): Parse arguments for specifying vehicle traits and profile
  // properties, or allow the user to pass a yaml file describing the properties
  auto profile = rmf_traffic::Trajectory::Profile::make_guided(
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(1.0));

  std::cout << "Vehicle traits: v: " << v_nom << " | a: " << a_nom << std::endl;
  rmf_traffic::agv::VehicleTraits traits{
    {v_nom*scaling, a_nom*scaling},
    {0.3*scaling, 1.5*scaling},
    profile
  };

  const auto fleet_adapter_node =
    rmf_fleet::adapter::FleetAdapterNode::make(fleet_name, graph_file, traits);

  if(!fleet_adapter_node)
  {
    std::cerr << "Failed to initialize the fleet adapter node" << std::endl;
    return 1;
  }

  RCLCPP_INFO(
        fleet_adapter_node->get_logger(),
        "Beginning loop");

  rclcpp::spin(fleet_adapter_node);

  RCLCPP_INFO(
        fleet_adapter_node->get_logger(),
        "Closing down");

  rclcpp::shutdown();
}
