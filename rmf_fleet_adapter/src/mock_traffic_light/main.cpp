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

// Internal implementation-specific headers
#include "../rmf_fleet_adapter/ParseArgs.hpp"
#include "../rmf_fleet_adapter/load_param.hpp"

// Public rmf_fleet_adapter API headers
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

// Standard topic names for communicating with fleet drivers
#include <rmf_fleet_adapter/StandardNames.hpp>

// Fleet driver state/command messages
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

// Utility functions for estimating where a robot is on the graph based on
// the information provided by fleet drivers.
#include "../rmf_fleet_adapter/estimation.hpp"

//==============================================================================
class MockTrafficLightCommandHandle
    : public rmf_fleet_adapter::agv::TrafficLight::CommandHandle
{
public:

  using PathRequestPub =
      rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr;

  using ModeRequestPub =
      rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr;

  MockTrafficLightCommandHandle(
      rclcpp::Node& node,
      std::string fleet_name,
      std::string robot_name,
      std::shared_ptr<const rmf_traffic::agv::Graph> graph,
      std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
      PathRequestPub path_request_pub)
    : _node(&node),
      _path_request_pub(std::move(path_request_pub))
  {
    _current_path_request.fleet_name = fleet_name;
    _current_path_request.robot_name = robot_name;

    _travel_info.graph = std::move(graph);
    _travel_info.traits = std::move(traits);
    _travel_info.fleet_name = std::move(fleet_name);
    _travel_info.robot_name = std::move(robot_name);
  }

  void receive_path_timing(
    const std::size_t version,
    const std::vector<rclcpp::Time>& departure_timing,
    ProgressCallback progress_updater) final
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_path_version != version)
      return;

    _progress_updater = std::move(progress_updater);
    if(_current_path_request.path.size() != departure_timing.size())
    {
      throw std::runtime_error(
            "Mismatch between path request size ["
            + std::to_string(_current_path_request.path.size())
            + "] and timing [" + std::to_string(departure_timing.size()) + "]");
    }

    for (std::size_t i=0; i < departure_timing.size(); ++i)
      _current_path_request.path[i].t = departure_timing[i];

    _current_path_request.task_id = std::to_string(++_command_version);
    _path_request_pub->publish(_current_path_request);
  }

  void deadlock() final
  {
    RCLCPP_ERROR(_node->get_logger(), "Deadlock detected!");
  }

  void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _current_path_request.path.clear();

    const auto& graph = _travel_info.graph;
    std::string first_level;
    for (const auto& wp : waypoints)
    {
      if (wp.graph_index())
      {
        first_level = graph->get_waypoint(*wp.graph_index()).get_map_name();
        break;
      }
    }

    const auto get_map_name = [&](rmf_utils::optional<std::size_t> index)
    {
      if (index)
        return graph->get_waypoint(*index).get_map_name();

      return first_level;
    };

    std::vector<rmf_fleet_adapter::agv::Waypoint> path;
    path.reserve(waypoints.size());

    for (std::size_t i=0; i < waypoints.size(); ++i)
    {
      const auto& wp = waypoints[i];
      if (i > 0)
      {
        const auto last_x = _current_path_request.path.back().x;
        const auto last_y = _current_path_request.path.back().y;
        const auto delta =
            (wp.position().block<2,1>(0,0)
             - Eigen::Vector2d(last_x, last_y)).norm();

        if (delta < 0.01)
          continue;
      }

      const auto p = wp.position();
      const auto map_name = get_map_name(wp.graph_index());

      rmf_fleet_msgs::msg::Location location;
      location.x = p[0];
      location.y = p[1];
      location.yaw = p[2];

      path.emplace_back(map_name, p);
    }

    _path_version = _path_updater->update_path(path);
  }

private:

  rclcpp::Node* _node;
  PathRequestPub _path_request_pub;
  rmf_fleet_msgs::msg::PathRequest _current_path_request;
  std::chrono::steady_clock::time_point _path_requested_time;
  TravelInfo _travel_info;

  std::size_t _path_version = 0;
  std::size_t _command_version = 0;

  ProgressCallback _progress_updater;

  std::mutex _mutex;

  rmf_fleet_adapter::agv::TrafficLight::UpdateHandlePtr _path_updater;
};

//==============================================================================
/// This is an RAII class that keeps the connections to the fleet driver alive.
struct Connections : public std::enable_shared_from_this<Connections>
{
  /// The API for adding new robots to the adapter
  rmf_fleet_adapter::agv::FleetUpdateHandlePtr fleet;

  /// The API for running the fleet adapter
  rmf_fleet_adapter::agv::AdapterPtr adapter;

  /// The navigation graph for the robot
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  /// The traits of the vehicles
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;

  /// The topic subscription for responding to new fleet states
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
  fleet_state_sub;

  /// The publisher for sending out path requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr
  path_request_pub;

  /// The publisher for sending out mode requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
  mode_request_pub;

  /// The container for robot update handles
  std::unordered_map<std::string, FleetDriverRobotCommandHandlePtr>
  robots;

  std::mutex mutex;

  void add_robot(
      const std::string& fleet_name,
      const rmf_fleet_msgs::msg::RobotState& state)
  {
    const auto& robot_name = state.name;
    const auto command = std::make_shared<FleetDriverRobotCommandHandle>(
          *adapter->node(), fleet_name, robot_name, graph, traits,
          path_request_pub, mode_request_pub);

    const auto& l = state.location;
    fleet->(
          command, robot_name, traits->profile(),
          rmf_traffic::agv::compute_plan_starts(
            *graph, state.location.level_name, {l.x, l.y, l.yaw},
            rmf_traffic_ros2::convert(adapter->node()->now())),
          [c = weak_from_this(), command, robot_name = std::move(robot_name)](
          const rmf_fleet_adapter::agv::RobotUpdateHandlePtr& updater)
    {
      const auto connections = c.lock();
      if (!connections)
        return;

      std::lock_guard<std::mutex> lock(connections->mutex);

      command->set_updater(updater);
      connections->robots[robot_name] = command;
    });
  }

  void add_traffic_light(
      const std::string& fleet_name,
      const rmf_fleet_msgs::msg::RobotState& state)
  {
    adapter->add_traffic_light(
          )
  }
};

//==============================================================================
std::shared_ptr<Connections> make_fleet(
    const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
  const auto& node = adapter->node();
  std::shared_ptr<Connections> connections = std::make_shared<Connections>();
  connections->adapter = adapter;

  const std::string fleet_name_param_name = "fleet_name";
  const std::string fleet_name = node->declare_parameter(
        "fleet_name", std::string());
  if (fleet_name.empty())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Missing [%s] parameter", fleet_name_param_name.c_str());

    return nullptr;
  }

  connections->traits = std::make_shared<rmf_traffic::agv::VehicleTraits>(
        rmf_fleet_adapter::get_traits_or_default(
        *node, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5));

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string graph_file =
      node->declare_parameter(nav_graph_param_name, std::string());
  if (graph_file.empty())
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Missing [%s] parameter", nav_graph_param_name.c_str());

    return nullptr;
  }

  connections->graph =
      std::make_shared<rmf_traffic::agv::Graph>(
        rmf_fleet_adapter::agv::parse_graph(graph_file, *connections->traits));

  std::cout << "The fleet [" << fleet_name
            << "] has the following named waypoints:\n";
  for (const auto& key : connections->graph->keys())
    std::cout << " -- " << key.first << std::endl;

  connections->fleet = adapter->add_fleet(
        fleet_name, *connections->traits, *connections->graph);

  // If the perform_deliveries parameter is true, then we just blindly accept
  // all delivery requests.
  if (node->declare_parameter<bool>("perform_deliveries", false))
  {
    connections->fleet->accept_delivery_requests(
          [](const rmf_task_msgs::msg::Delivery&){ return true; });
  }

  if (node->declare_parameter<bool>("disable_delay_threshold", false))
  {
    connections->fleet->default_maximum_delay(rmf_utils::nullopt);
  }
  else
  {
    connections->fleet->default_maximum_delay(
          rmf_fleet_adapter::get_parameter_or_default_time(
            *node, "delay_threshold", 10.0));
  }

  connections->path_request_pub = node->create_publisher<
      rmf_fleet_msgs::msg::PathRequest>(
        rmf_fleet_adapter::PathRequestTopicName, rclcpp::SystemDefaultsQoS());

  connections->mode_request_pub = node->create_publisher<
      rmf_fleet_msgs::msg::ModeRequest>(
        rmf_fleet_adapter::ModeRequestTopicName, rclcpp::SystemDefaultsQoS());

  connections->fleet_state_sub = node->create_subscription<
      rmf_fleet_msgs::msg::FleetState>(
        rmf_fleet_adapter::FleetStateTopicName,
        rclcpp::SystemDefaultsQoS(),
        [c = std::weak_ptr<Connections>(connections), fleet_name](
        const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
  {
    if (msg->name != fleet_name)
      return;

    const auto connections = c.lock();
    if (!connections)
      return;

    for (const auto& state : msg->robots)
    {
      const auto insertion = connections->robots.insert({state.name, nullptr});
      const bool new_robot = insertion.second;
      if (new_robot)
      {
        // We have not seen this robot before, so let's add it to the fleet.
        connections->add_robot(fleet_name, state);
      }

      const auto& command = insertion.first->second;
      if (command)
      {
        // We are ready to command this robot, so let's update its state
        command->update_state(state);
      }
    }
  });

  return connections;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
    return 1;


}
