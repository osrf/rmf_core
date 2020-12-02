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
#include <rmf_fleet_msgs/msg/location.hpp>
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

namespace {
//==============================================================================
std::string time_to_str(const rclcpp::Time& t)
{
  double s = t.seconds();
  int hours = static_cast<int>(s)/3600;
  s = s - 3600*hours;
  int minutes = static_cast<int>(s)/60;
  s = s - 60*minutes;

  std::stringstream ss;
  ss << hours << ":" << minutes << ":" << s;
  return ss.str();
}

//==============================================================================
rmf_fleet_msgs::msg::Location make_location(
    rclcpp::Time t,
    const Eigen::Vector3d& p,
    const std::string& map_name,
    const std::size_t index)
{
  return rmf_fleet_msgs::build<rmf_fleet_msgs::msg::Location>()
      .t(t)
      .x(p[0]).y(p[1]).yaw(p[2])
      .level_name(map_name)
      .index(index);
}

} // anonymous namespace

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

  const std::string& name() const
  {
    return _travel_info.robot_name;
  }

  void receive_checkpoints(
    const std::size_t version,
    std::vector<Checkpoint> checkpoints,
    std::size_t standby_at,
    OnStandby on_standby,
    Reject reject) final
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _internal_receive_checkpoints(
          version,
          std::move(checkpoints),
          standby_at,
          std::move(on_standby),
          std::move(reject));
  }

  void immediately_stop_until(
      std::size_t version,
      rclcpp::Time time,
      StoppedAt stopped,
      Departed departed) final
  {
    if (version != _path_version)
      return;

    std::cout << name() << " !!! BEING TOLD TO STOP UNTIL " << time_to_str(time)
              << " | current target: " << _last_target << std::endl;
    _depart_resuming = departed;

    const auto& p = _last_state.location;
    stopped({p.x, p.y, p.yaw});

    _current_path_request.path.clear();
    _update_location.clear();

    // We set this index value before adding this location to the path so that
    // we definitively embed its target checkpoint index into the path that we
    // will read back from the robot.
    _last_state.location.index = _last_target;

    // The initial confirmation waypoint
    _current_path_request.path.push_back(_last_state.location);
    _update_location.push_back(
          [target = _last_target](Eigen::Vector3d){ return target; });

    // The waiting waypoint
    _current_path_request.path.push_back(_last_state.location);
    _current_path_request.path.back().t = time;
    _update_location.push_back(
          [target = _last_target](Eigen::Vector3d){ return target; });

    if (_last_target > 0)
    {
      // The target waypoint
      const auto& wp = _current_path.at(_last_target);
      auto location = make_location(
            time, wp.position(), wp.map_name(), _last_target);

      _current_path_request.path.emplace_back(location);
      _update_location.push_back(
            [target = _last_target, departed](
            Eigen::Vector3d location){ departed(location); return target; });
    }

    for (auto& c : _checkpoints)
      c.reset();

    _current_path_request.task_id = std::to_string(++_command_version);
    _path_request_pub->publish(_current_path_request);
    _moving = true;
    _finishing = (_last_target == _checkpoints.size());

    // receive_checkpoints should give us a new standby callback soon
    _on_standby = nullptr;
  }

  void resume(std::size_t) final
  {
    std::cout << name() << " !!! being told to resume" << std::endl;
  }

  void deadlock(std::vector<Blocker>) final
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

    if (state.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR)
    {
      std::cout << name() << " > Adapter Error!! Resending checkpoints! Version: "
                << _path_version << std::endl;
      if (state.path.empty())
        _last_target = _last_accepted_goal.value();
      else
        _last_target = state.path.front().index;

      _last_state.location.index = _last_target;
      _internal_receive_checkpoints(
            _last_received_version,
            _last_received_checkpoints,
            _last_received_standby_at,
            _on_standby,
            _last_received_reject);
      return;
    }

    if (_queue.empty())
    {
      std::cout << name() << " ??? MOVING WHILE QUEUE IS EMPTY?? " << name()
                << " | task_id: " << _current_path_request.task_id << std::endl;
    }

    if (state.path.empty())
    {
      if (_on_standby)
      {
        std::cout << name() << " > Triggering _on_standby for " << _last_target << std::endl;
        _last_target = _last_received_standby_at;
        _on_standby();
        _on_standby = nullptr;
      }

      assert(!_checkpoints.empty());
      if (_finishing)
      {
        _moving = false;
        _finishing = false;
        assert(!_queue.empty());
        std::cout << "Remaining queue size: " << _queue.size() << " - 1" << std::endl;

        std::cout << __LINE__ << ": popping " << name() << " " << _queue.size();
        _queue.pop_front();
        std::cout << " -> " << _queue.size() << " | task_id: " << _current_path_request.task_id << std::endl;

        _go_to_next_waypoint();
        return;
      }

      return;
    }

    _last_accepted_goal = state.path.back().index;

    if (state.path.size() < _update_location.size())
    {
      const std::size_t remove_N = _update_location.size() - state.path.size();
      _update_location.erase(
            _update_location.begin(),
            _update_location.begin() + remove_N);
    }

    assert(!_update_location.empty());
    if (_update_location.empty())
    {
      std::cout << " !!!! Empty _update_location for " << name() << std::endl;
    }
    const auto& l = state.location;
    _last_target = _update_location.front()({l.x, l.y, l.yaw});
    _last_state.location.index = _last_target;
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
      _go_to_next_waypoint();
  }

  rmf_utils::optional<std::string> start_waypoint() const
  {
    if (_moving || !_queue.empty())
      return rmf_utils::nullopt;

    const auto& l = _last_state.location;
    const auto starts = rmf_traffic::agv::compute_plan_starts(
          _planner->get_configuration().graph(),
          l.level_name,
          {l.x, l.y, l.yaw},
          rmf_traffic_ros2::convert(_node->now()));

    if (starts.empty())
      return rmf_utils::nullopt;

    const auto* name = _planner->get_configuration().graph().get_waypoint(
          starts.front().waypoint()).name();

    if (name)
      return *name;

    return rmf_utils::nullopt;
  }

private:

  void _go_to_next_waypoint()
  {
    while (!_queue.empty())
    {
      const std::string next = _queue.front();

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

        std::cout << __LINE__ << ": popping " << name() << " " << _queue.size();
        _queue.pop_front();
        std::cout << " -> " << _queue.size() << std::endl;

        continue;
      }

      const auto result = _planner->plan(starts, goal->index());
      if (!result)
      {
        RCLCPP_WARN(
              _node->get_logger(),
              "Could not find a plan to get robot [%s] to waypoint [%s]",
              _travel_info.robot_name.c_str(), next.c_str());

        std::cout << __LINE__ << ": popping " << name() << " " << _queue.size();
        _queue.pop_front();
        std::cout << " -> " << _queue.size() << std::endl;
        continue;
      }

      if (result->get_waypoints().size() < 2)
      {
        std::cout << __LINE__ << ": popping " << name() << " " << _queue.size();
        _queue.pop_front();
        std::cout << " -> " << _queue.size() << std::endl;
        continue;
      }

      std::cout << "Moving " << name() << " towards " << _queue.front()
                << " with " << result->get_waypoints().size() << " checkpoints" << std::endl;

      _follow_new_path(result->get_waypoints());
      return;
    }
  }

  void _follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
  {
    _current_path_request.path.clear();

    _moving = false;
    _last_target = 0;

    // This deals with the edge case where an ADAPTER_ERROR happens just after
    // the robot finishes a path and has not responded positively to the next
    // path. In that situation, we want _last_accepted_goal to have a value of 0
    // so that _last_target will be given a value of 0.
    _last_accepted_goal = 0;

    const auto& graph = _travel_info.graph;
    std::string first_level;
    for (const auto& wp : waypoints)
    {
      if (wp.graph_index())
      {
        first_level = graph->get_waypoint(wp.graph_index().value()).get_map_name();
        break;
      }
    }

    const auto get_map_name = [&](rmf_utils::optional<std::size_t> index)
    {
      if (index)
        return graph->get_waypoint(index.value()).get_map_name();

      return first_level;
    };

    _current_path.clear();
    _current_path.reserve(waypoints.size());

    for (std::size_t i=0; i < waypoints.size(); ++i)
    {
      const auto& wp = waypoints[i];
      if (i > 0)
      {
        const auto last_x = _current_path.back().position().x();
        const auto last_y = _current_path.back().position().y();
        const auto delta =
            (wp.position().block<2,1>(0,0)
             - Eigen::Vector2d(last_x, last_y)).norm();

        if (delta < 0.01)
          continue;
      }

      const auto p = wp.position();
      const auto map_name = get_map_name(wp.graph_index());
      _current_path.emplace_back(map_name, p);
    }

    _checkpoints.clear();
    _checkpoints.resize(_current_path.size()-1);
    _path_version = _path_updater->follow_new_path(_current_path);
  }

  void _internal_receive_checkpoints(
      const std::size_t version,
      std::vector<Checkpoint> checkpoints,
      const std::size_t standby_at,
      OnStandby on_standby,
      Reject reject)
  {
    if (_path_version != version)
      return;

    _last_received_version = version;
    _last_received_checkpoints = checkpoints;
    _last_received_standby_at = standby_at;
    _last_received_reject = reject;

    std::cout << name() << " > new checkpoints:";
    for (const auto& c : checkpoints)
      std::cout << " " << c.waypoint_index;

    std::cout << " | standby_at: " << standby_at << " | _last_target: " << _last_target;
    std::cout << std::endl;

    if (standby_at < _last_target)
    {
      // The last checkpoint is earlier than the current target of the robot.
      // That means we are already violating the expectations of this plan.
      // We must reject it to get a new plan.
      RCLCPP_INFO(
            _node->get_logger(),
            " !!! REJECTING NEW CHECKPOINTS FOR [%s]. Last target: %u",
            name().c_str(),
            _last_target);

      _last_state.location.index = _last_target;

      // This effectively tells the robot to stop in place
      _current_path_request.path.push_back(_last_state.location);
      _update_location.push_back(
          [target = _last_target](Eigen::Vector3d){ return target; });

      const auto& p = _last_state.location;
      reject(_last_target-1, {p.x, p.y, p.yaw});
    }

    _on_standby = std::move(on_standby);

    if (checkpoints.empty())
    {
      // This should only happen if immediately_stop_until was called
      // beforehand. In that case, we shouldn't need to send any new command
      // to the robot
      return;
    }

    for (const auto& c : checkpoints)
      _checkpoints.at(c.waypoint_index) = c;

    const auto last_ready_checkpoint = checkpoints.back().waypoint_index;
    for (std::size_t c = last_ready_checkpoint+1; c < _checkpoints.size(); ++c)
      _checkpoints.at(c).reset();

    _current_path_request.path.clear();
    _current_path_request.path.reserve(_checkpoints.size());

    _update_location.clear();
    _update_location.reserve(_checkpoints.size());

    _last_state.location.index = _last_target;
    _current_path_request.path.push_back(_last_state.location);
    _update_location.push_back(
          [target = _last_target](Eigen::Vector3d){ return target; });

    _finishing = standby_at == _checkpoints.size();

    const auto push_back = [&](
        const std::size_t index,
        rclcpp::Time t,
        std::function<std::size_t(Eigen::Vector3d)> update)
    {
      assert(index < _current_path.size());
      const auto& p = _current_path.at(index).position();
      auto location = make_location(
            t, p, _current_path.at(index).map_name(), index);

      _current_path_request.path.emplace_back(std::move(location));
      _update_location.emplace_back(std::move(update));
    };

    for (std::size_t i=_last_target; i <= _checkpoints.size(); ++i)
    {
      auto departure_time = [&]()
      {
        if (i == _checkpoints.size())
        {
          // The last checkpoint won't receive a departure time, but the
          // rmf_fleet_msgs API demands one. So we'll just use the previous
          // checkpoint's departure time since the value doesn't matter
          // anyway.
          return _checkpoints.at(i-1).value().departure_time;
        }

        const auto& c = _checkpoints.at(i);
        if (c.has_value())
          return c->departure_time;

        // Similar to the last checkpoint case, if we've made it past the last
        // checkpoint that has been assigned a departure time, we will use the
        // departure time of the previous checkpoint. We only ever iterate one
        // past the last departure checkpoint, so this should be okay.
        assert(i > 0);
        return _checkpoints.at(i-1).value().departure_time;
      }();

      auto updater = [&]() -> std::function<std::size_t(Eigen::Vector3d)>
      {
        if (i == 0)
          return [](Eigen::Vector3d){ return 0; };

        if (_checkpoints.at(i-1).has_value())
        {
          return [i, departed = _checkpoints.at(i-1).value().departed](
              Eigen::Vector3d location){ departed(location); return i; };
        }

        // If the previous checkpoint does not have a departed callback, then
        // we should assume that we were given an immediate stop command, and
        // we'll use the _depart_resuming callback.
        if (!_depart_resuming)
        {
          throw std::runtime_error(
              "[MockTrafficLightCommandHandle::receive_checkpoints] Missing "
              "_depart_resuming for robot " + name());
        }

        return [i, departed = _depart_resuming](
            Eigen::Vector3d location){ departed(location); return i; };
      }();

      push_back(i, departure_time, std::move(updater));

      if (i < _checkpoints.size() && !_checkpoints.at(i).has_value())
        break;
    }

    std::cout << name() << " > issuing path request:";
    for (const auto& l : _current_path_request.path)
      std::cout << " " << l.index;
    std::cout << std::endl;

    _current_path_request.task_id = std::to_string(++_command_version);
    _path_request_pub->publish(_current_path_request);
    _moving = true;
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
  bool _finishing = false;

  std::vector<rmf_fleet_adapter::agv::Waypoint> _current_path;
  std::vector<std::optional<Checkpoint>> _checkpoints;
  std::vector<std::function<std::size_t(Eigen::Vector3d)>> _update_location;

  std::size_t _last_received_version;
  std::vector<Checkpoint> _last_received_checkpoints;
  std::size_t _last_received_standby_at;
  Reject _last_received_reject;
  OnStandby _on_standby;
  Departed _depart_resuming;

  std::optional<std::size_t> _last_accepted_goal;

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

  std::unordered_set<std::string> received_task_ids;

  void add_traffic_light(
      const std::string& fleet_name,
      const rmf_fleet_msgs::msg::RobotState& state)
  {
    const std::string robot_name = state.name;
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

  auto graph =
      std::make_shared<rmf_traffic::agv::Graph>(
        rmf_fleet_adapter::agv::parse_graph(graph_file, *connections->traits));

  // We add pseudo-events on every lane to force the planner to include every
  // intermediate waypoint in its plan.
  for (std::size_t i=0; i < graph->num_lanes(); ++i)
  {
    graph->get_lane(i).exit().event(
          rmf_traffic::agv::Graph::Lane::Event::make(
            rmf_traffic::agv::Graph::Lane::Wait(std::chrono::seconds(0))));
  }

  connections->graph = std::move(graph);
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

    if (!connections->received_task_ids.insert(msg->task_id).second)
      return;

    MockTrafficLightCommandHandlePtr best = nullptr;
    for (const auto& r : connections->robots)
    {
      const auto start = r.second->start_waypoint();
      if (!start)
        continue;

      if (*start == msg->start_name)
      {
        best = r.second;
        break;
      }
    }

    if (!best)
    {
      // This is a very dumb approach to assigning the tasks, but this adapter is
      // only meant for testing purposes anyway.
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
    }

    std::vector<std::string> new_waypoints;
    new_waypoints.reserve(2*msg->num_loops);
    for (std::size_t i=0; i < msg->num_loops; ++i)
    {
      new_waypoints.push_back(msg->start_name);
      new_waypoints.push_back(msg->finish_name);
    }

    RCLCPP_INFO(
      connections->adapter->node()->get_logger(),
      "Assigning task [%s] to robot [%s]",
      msg->task_id.c_str(),
      best->name().c_str());

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
