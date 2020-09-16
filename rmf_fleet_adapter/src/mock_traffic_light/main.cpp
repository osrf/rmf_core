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

// Loop request message
#include <rmf_task_msgs/msg/loop.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

// Utility functions for estimating where a robot is on the graph based on
// the information provided by fleet drivers.
#include "../rmf_fleet_adapter/estimation.hpp"

#include <deque>

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
      std::shared_ptr<const rmf_traffic::agv::Planner> planner,
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

    _planner = planner;
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
    if(_path_size != departure_timing.size())
    {
      throw std::runtime_error(
            "Mismatch between path request size ["
            + std::to_string(_path_size) + "] and timing ["
            + std::to_string(departure_timing.size()) + "]");
    }

    // Having _last_target == 0 causes problems with the index conversions below
    // so we'll switch its value to 1 to deal with that edge case
    if (_last_target == 0)
      _last_target = 1;

    const std::size_t N_command = _current_path_request.path.size();
    const std::size_t N_command_desired = _path_size - _last_target + 1;
    if (N_command_desired < N_command)
    {
      _current_path_request.path.erase(
            _current_path_request.path.begin() +
            N_command - N_command_desired);
    }
    _current_path_request.path.front() = _last_state.location;

    for (std::size_t i=_last_target-1; i < _path_size; ++i)
      _current_path_request.path[i+1-_last_target].t = departure_timing[i];

    std::cout << " >>> Issuing timing commands: " << std::endl;
    for (std::size_t i=0; i < _current_path_request.path.size(); ++i)
      std::cout << " -- " << i << ": "
                << rclcpp::Time(_current_path_request.path[i].t).seconds()
                << std::endl;

    _moving = true;
    _current_path_request.task_id = std::to_string(++_command_version);
    _path_request_pub->publish(_current_path_request);
  }

  void deadlock() final
  {
    RCLCPP_ERROR(_node->get_logger(), "Deadlock detected!");
  }

  void set_updater(
      rmf_fleet_adapter::agv::TrafficLight::UpdateHandlePtr updater)
  {
    _path_updater = std::move(updater);
  }

  void update_state(const rmf_fleet_msgs::msg::RobotState& state)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _last_state = state;

    if (!_moving)
      return;

    if (_current_path_request.task_id != state.task_id)
    {
      _path_request_pub->publish(_current_path_request);
      return;
    }

    if (state.path.empty())
    {
      _moving = false;
      _queue.pop_front();
      _start_next_waypoint();
      return;
    }

    assert(state.path.size() <= _path_size);
    _last_target = _path_size - state.path.size();

    const auto& l = state.location;
    const Eigen::Vector3d p(l.x, l.y, l.yaw);
    _progress_updater(_last_target, p);
  }

  std::size_t queue_size() const
  {
    return _queue.size();
  }

  void add_to_queue(const std::vector<std::string>& new_waypoints)
  {
    std::unique_lock<std::mutex> lock(_mutex);

    const bool empty_queue = _queue.empty();

    for (const auto& wp : new_waypoints)
      _queue.push_back(wp);

    if (!_moving && empty_queue)
      _start_next_waypoint();
  }

private:

  void _start_next_waypoint()
  {
    if (_queue.empty())
      return;

    const std::string next = std::move(_queue.front());

    const auto& l = _last_state.location;
    const auto starts = rmf_traffic::agv::compute_plan_starts(
          _planner->get_configuration().graph(),
          l.level_name,
          {l.x, l.y, l.yaw},
          rmf_traffic_ros2::convert(_node->now()));

    if (starts.empty())
    {
      RCLCPP_ERROR(
            _node->get_logger(),
            "Robot [%s] is lost!",
            _travel_info.robot_name.c_str());
      return;
    }

    const auto goal = _planner->get_configuration().graph().find_waypoint(next);
    if (!goal)
    {
      RCLCPP_ERROR(
            _node->get_logger(),
            "Could not find a goal named [%s] for robot [%s]",
            next.c_str(), _travel_info.robot_name.c_str());
      return;
    }

    const auto result = _planner->plan(starts, goal->index());
    if (!result)
    {
      RCLCPP_ERROR(
            _node->get_logger(),
            "Could not find a plan to get robot [%s] to waypoint [%s]",
            _travel_info.robot_name.c_str(), next.c_str());
      return;
    }

    _follow_new_path(result->get_waypoints());
  }

  void _follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
  {
    _current_path_request.path.clear();

    _moving = false;
    _last_target = 0;
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
      _current_path_request.path.push_back(location);

      path.emplace_back(map_name, p);
    }

    std::cout << "Submitting path size: " << path.size() << std::endl;
    _path_version = _path_updater->update_path(path);
    _path_size = path.size();
  }

  std::deque<std::string> _queue;

  rclcpp::Node* _node;
  PathRequestPub _path_request_pub;
  rmf_fleet_msgs::msg::PathRequest _current_path_request;
  std::chrono::steady_clock::time_point _path_requested_time;
  TravelInfo _travel_info;
  std::shared_ptr<const rmf_traffic::agv::Planner> _planner;
  rmf_fleet_msgs::msg::RobotState _last_state;

  std::size_t _path_version = 0;
  std::size_t _command_version = 0;
  std::size_t _last_target = 0;
  bool _moving = false;
  std::size_t _path_size;

  ProgressCallback _progress_updater;

  std::mutex _mutex;

  rmf_fleet_adapter::agv::TrafficLight::UpdateHandlePtr _path_updater;
};

using MockTrafficLightCommandHandlePtr =
std::shared_ptr<MockTrafficLightCommandHandle>;

//==============================================================================
/// This is an RAII class that keeps the connections to the fleet driver alive.
struct Connections : public std::enable_shared_from_this<Connections>
{
  /// The API for running the fleet adapter
  rmf_fleet_adapter::agv::AdapterPtr adapter;

  /// The navigation graph for the robot
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  std::shared_ptr<const rmf_traffic::agv::Planner> planner;

  /// The traits of the vehicles
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;

  /// The topic subscription for responding to new fleet states
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
  fleet_state_sub;

  /// The topic subscription for receiving loop requests
  rclcpp::Subscription<rmf_task_msgs::msg::Loop>::SharedPtr
  loop_sub;

  /// The publisher for sending out path requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr
  path_request_pub;

  /// The publisher for sending out mode requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
  mode_request_pub;

  std::unordered_map<std::string, MockTrafficLightCommandHandlePtr> robots;

  std::mutex mutex;

  void add_traffic_light(
      const std::string& fleet_name,
      const rmf_fleet_msgs::msg::RobotState& state)
  {
    const std::string& robot_name = state.name;
    const auto command = std::make_shared<MockTrafficLightCommandHandle>(
          *adapter->node(), fleet_name, robot_name, graph, traits, planner,
          path_request_pub);

    adapter->add_traffic_light(
          command, fleet_name, robot_name, *traits,
          [c = weak_from_this(), command, robot_name = robot_name](
          rmf_fleet_adapter::agv::TrafficLight::UpdateHandlePtr updater)
    {
      const auto connections = c.lock();
      command->set_updater(std::move(updater));
      connections->robots[robot_name] = command;
    });
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

  connections->planner =
      std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Planner::Configuration(
          *connections->graph,
          *connections->traits),
        rmf_traffic::agv::Planner::Options(nullptr));

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
        RCLCPP_INFO(
          connections->adapter->node()->get_logger(),
          "Adding robot [%s] to the fleet",
          state.name.c_str());

        // We have not seen this robot before, so let's add it to the fleet.
        connections->add_traffic_light(fleet_name, state);
      }

      const auto& command = insertion.first->second;
      if (command)
      {
        // We are commanding this robot, so let's update its state
        command->update_state(state);
      }
    }
  });

  connections->loop_sub = node->create_subscription<
      rmf_task_msgs::msg::Loop>(
        rmf_fleet_adapter::LoopRequestTopicName,
        rclcpp::SystemDefaultsQoS(),
        [c = std::weak_ptr<Connections>(connections), fleet_name](
        const rmf_task_msgs::msg::Loop::SharedPtr msg)
  {
    if (msg->robot_type != fleet_name)
      return;

    const auto connections = c.lock();
    if (!connections)
      return;

    // This is a very dumb approach to assigning the tasks, but this adapter is
    // only meant for testing purposes anyway.
    MockTrafficLightCommandHandlePtr best = nullptr;
    for (const auto& r : connections->robots)
    {
      const auto& candidate = r.second;
      if (!best || candidate->queue_size() < best->queue_size())
        best = candidate;
    }

    if (!best)
    {
      RCLCPP_ERROR(
        connections->adapter->node()->get_logger(),
        "No robot is available to fulfill the loop task request [%s]",
        msg->task_id.c_str());
      return;
    }

    std::vector<std::string> new_waypoints;
    new_waypoints.reserve(2*msg->num_loops);
    for (std::size_t i=0; i < msg->num_loops; ++i)
    {
      new_waypoints.push_back(msg->start_name);
      new_waypoints.push_back(msg->finish_name);
    }

    best->add_to_queue(new_waypoints);
  });

  return connections;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
    return 1;

  const auto fleet_connections = make_fleet(adapter);
  if (!fleet_connections)
    return 1;

  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Mock Traffic Light");

  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Mock Traffic Light");

  rclcpp::shutdown();
}
