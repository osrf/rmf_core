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

#include "../rmf_fleet_adapter/ParseGraph.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rclcpp/executors.hpp>

#include "Actions.hpp"

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make(
    const std::string& fleet_name,
    const std::string& graph_file,
    rmf_traffic::agv::VehicleTraits traits,
    const rmf_traffic::Duration plan_time,
    const rmf_traffic::Duration wait_time)
{
  const auto node = std::shared_ptr<FleetAdapterNode>(
        new FleetAdapterNode(fleet_name, plan_time));

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
        *node, rmf_traffic::schedule::query_everything().spacetime());

  auto submit_trajectories = node->create_client<SubmitTrajectories>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName);

  auto delay_trajectories = node->create_client<DelayTrajectories>(
        rmf_traffic_ros2::DelayTrajectoriesSrvName);

  auto replace_trajectories = node->create_client<ReplaceTrajectories>(
        rmf_traffic_ros2::ReplaceTrajectoriesSrvName);

  rmf_traffic::agv::Graph graph;
  std::unordered_map<std::string, std::size_t> waypoint_keys;
  if (!parse_graph(graph_file, traits, *node, graph, waypoint_keys))
    return nullptr;

  using namespace std::chrono_literals;

  const auto stop_time = std::chrono::steady_clock::now() + wait_time;
  while(rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    bool ready = true;
    ready &= submit_trajectories->service_is_ready();
    ready &= delay_trajectories->service_is_ready();
    ready &= replace_trajectories->service_is_ready();
    ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

    if (ready)
    {
      node->start(
            Fields{
              std::move(graph),
              std::move(traits),
              mirror_future.get(),
              std::move(submit_trajectories),
              std::move(delay_trajectories),
              std::move(replace_trajectories)
            });

      return node;
    }
  }

  RCLCPP_ERROR(
        node->get_logger(),
        "Timeout after waiting ["
        + std::to_string(rmf_traffic::time::to_seconds(wait_time))
        + "] to connect to the schedule");

  return nullptr;
}

//==============================================================================
FleetAdapterNode::Task::Task(
    FleetAdapterNode* node,
    FleetAdapterNode::RobotContext* state,
    const Delivery& request,
    const std::size_t pickup_wp,
    const std::size_t dropoff_wp)
: _delivery(request),
  _node(node),
  _state_ptr(state)
{
  _action_queue.push(
        make_move(node, state, pickup_wp, node->get_fallback_wps()));


}

//==============================================================================
void FleetAdapterNode::Task::next()
{
  if (!_start_time)
    _start_time = _node->get_clock()->now();

  if (_action_queue.empty())
  {
    return _state_ptr->next_task();
  }

  _action = std::move(_action_queue.front());
  _action_queue.pop();
  _action->execute();
}

//==============================================================================
const std::string& FleetAdapterNode::get_fleet_name() const
{
  return _fleet_name;
}

//==============================================================================
rmf_traffic::Duration FleetAdapterNode::get_plan_time() const
{
  return _plan_time;
}

//==============================================================================
const rmf_traffic::agv::Planner& FleetAdapterNode::get_planner() const
{
  return _field->planner;
}

//==============================================================================
const rmf_traffic::agv::Graph& FleetAdapterNode::get_graph() const
{
  return _field->graph;
}

//==============================================================================
std::vector<rmf_traffic::agv::Plan::Start>
FleetAdapterNode::compute_plan_starts(const Location& location)
{
  // Add 3 seconds to the current time to give us some buffer
  // TODO(MXG): Make this configurable
  const auto now = rmf_traffic_ros2::convert(get_clock()->now())
      + std::chrono::seconds(3);

  const Eigen::Vector2d p_location = {location.x, location.y};
  const double start_yaw = static_cast<double>(location.yaw);

  const auto& graph = _field->graph;

  for (std::size_t i=0; i < graph.num_waypoints(); ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    const Eigen::Vector2d wp_location = wp.get_location();

    if ( (p_location - wp_location).norm() < 0.05 )
    {
      // This waypoint is very close to the real location, so we will assume
      // that the robot is located here.
      return {rmf_traffic::agv::Plan::Start(now, wp.index(), start_yaw)};
    }
  }

  std::vector<rmf_traffic::agv::Plan::Start> starts;

  double closest_lane_dist = std::numeric_limits<double>::infinity();
  std::size_t closest_lane = std::numeric_limits<std::size_t>::max();
  for (std::size_t i=0; i < graph.num_lanes(); ++i)
  {
    const auto& lane = graph.get_lane(i);
    const Eigen::Vector2d p0 = graph.get_waypoint(
          lane.entry().waypoint_index()).get_location();
    const Eigen::Vector2d p1 = graph.get_waypoint(
          lane.exit().waypoint_index()).get_location();

    const double lane_length = (p1 - p0).norm();
    const Eigen::Vector2d pn = (p1 - p0)/lane_length;

    const Eigen::Vector2d p_l = p_location - p0;
    const double p_l_projection = p_l.dot(pn);

    double lane_dist = std::numeric_limits<double>::infinity();
    if (p_l_projection < 0.0)
    {
      // If it's negative then its closest point on the lane is the entry point
      lane_dist = p_l.norm();
    }
    else if (p_l_projection > lane_length)
    {
      // If it's larger than the lane length, then its closest point on the lane
      // is the exit point.
      lane_dist = (p_location - p1).norm();
    }
    else
    {
      // If it's between the entry and the exit points, then we should compute
      // its distance away from the lane line.
      lane_dist = (p_l - p_l_projection*pn).norm();
    }

    if (lane_dist < 0.25)
    {
      // If the robot is within 25cm of the lane, go ahead and add it as one of
      // the options to consider.
      starts.emplace_back(
            rmf_traffic::agv::Plan::Start(
              now, lane.exit().waypoint_index(), start_yaw, p_location, i));
    }
  }

  if (starts.empty())
  {
    // None of the lanes were very close, so we'll go ahead and use the one that
    // seems closest.
    if (closest_lane_dist > 1.0)
    {
      RCLCPP_WARN(
            get_logger(),
            "The robot appears to be [" + std::to_string(closest_lane_dist)
            + "] meters away from its closest lane on the graph ["
            + std::to_string(closest_lane) + "]!");
    }

    starts.emplace_back(
          rmf_traffic::agv::Plan::Start(
            now, graph.get_lane(closest_lane).exit().waypoint_index(),
            start_yaw, p_location, closest_lane));
  }

  return starts;
}

//==============================================================================
auto FleetAdapterNode::get_waypoint_keys() const -> const WaypointKeys&
{
  return _field->waypoint_keys;
}

//==============================================================================
const std::vector<std::size_t>& FleetAdapterNode::get_fallback_wps() const
{
  return _field->fallback_waypoints;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode(
    const std::string& fleet_name,
    rmf_traffic::Duration plan_time)
: rclcpp::Node(fleet_name + "__full_control_fleet_adapter"),
  _fleet_name(fleet_name),
  _plan_time(plan_time)
{
  // Do nothing
}

//==============================================================================
void FleetAdapterNode::start(Fields fields)
{
  _field = std::move(fields);
  _field->mirror.update();

  _delivery_sub = create_subscription<Delivery>(
        DeliveryTopicName, rclcpp::SystemDefaultsQoS(),
        [&](Delivery::UniquePtr msg)
  {
    this->delivery_request(std::move(msg));
  });

  _dispenser_result_sub = create_subscription<DispenserResult>(
        DispenserResultTopicName, rclcpp::SystemDefaultsQoS(),
        [&](DispenserResult::UniquePtr msg)
  {
    this->dispenser_result_update(std::move(msg));
  });

  _dispenser_state_sub = create_subscription<DispenserState>(
        DispenserStateTopicName, rclcpp::SystemDefaultsQoS(),
        [&](DispenserState::UniquePtr msg)
  {
    this->dispenser_state_update(std::move(msg));
  });

  _fleet_state_sub = create_subscription<FleetState>(
        FleetStateTopicName, rclcpp::SystemDefaultsQoS(),
        [&](FleetState::UniquePtr msg)
  {
    this->fleet_state_update(std::move(msg));
  });

  _door_state_sub = create_subscription<DoorState>(
        DoorStateTopicName, rclcpp::SystemDefaultsQoS(),
        [&](DoorState::UniquePtr msg)
  {
    this->door_state_update(std::move(msg));
  });

  path_request_publisher = create_publisher<PathRequest>(
        PathRequestTopicName, rclcpp::SystemDefaultsQoS());

  door_request_publisher = create_publisher<DoorRequest>(
        DoorRequestTopicName, rclcpp::SystemDefaultsQoS());

  task_summary_publisher = create_publisher<TaskSummary>(
        TaskSummaryTopicName, rclcpp::SystemDefaultsQoS());
}

//==============================================================================
void FleetAdapterNode::delivery_request(Delivery::UniquePtr msg)
{
  const auto& waypoint_keys = _field->waypoint_keys;
  const auto& delivery = *msg;
  const auto pickup = waypoint_keys.find(delivery.pickup_place_name);
  if (pickup == waypoint_keys.end())
  {
    RCLCPP_ERROR(
          get_logger(),
          "Unknown pickup location [" + delivery.pickup_place_name + "] in "
          + "delivery request [" + delivery.task_id + "]");
    return;
  }

  const auto dropoff = waypoint_keys.find(delivery.dropoff_place_name);
  if (dropoff == waypoint_keys.end())
  {
    RCLCPP_ERROR(
          get_logger(),
          "Uknown dropoff location [" + delivery.dropoff_place_name + "] in "
          + "delivery request [" + delivery.task_id + "]");
    return;
  }

  if (_contexts.empty())
  {
    RCLCPP_ERROR(
          get_logger(),
          "No robots appear to be online!");
    return;
  }

  auto context = _contexts.begin()->second.get();

  auto task = std::make_unique<Task>(
        this, context, delivery, pickup->second, dropoff->second);

  if (!context->task)
  {
    // No task is currently active, so set this new task as the active one and
    // begin executing it.
    context->task = std::move(task);
    context->task->next();
  }
  else
  {
    // A task is currently active, so add this task to the queue. We're going to
    // do first-come first-serve for task requests for now.
    context->task_queue.emplace(std::move(task));
  }
}

//==============================================================================
void FleetAdapterNode::dispenser_result_update(DispenserResult::UniquePtr msg)
{
  for (auto* listener : dispenser_result_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::dispenser_state_update(DispenserState::UniquePtr msg)
{
  for (auto* listener : dispenser_state_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::fleet_state_update(FleetState::UniquePtr msg)
{
  const auto& fleet_state = *msg;
  if (fleet_state.name != _fleet_name)
    return;

  for (const auto& robot : fleet_state.robots)
  {
    const auto insertion = _contexts.insert(
          std::make_pair(robot.name, nullptr));
    const bool inserted = insertion.second;
    const auto it = insertion.first;

    if (inserted)
    {
      it->second = std::make_unique<RobotContext>(
            RobotContext{robot.location, nullptr, {}, {}});
    }
    else
    {
      it->second->location = robot.location;
    }

    for (auto* listener : it->second->listeners)
      listener->receive(robot);
  }
}

//==============================================================================
void FleetAdapterNode::door_state_update(DoorState::UniquePtr msg)
{
  for (const auto& listener : door_state_listeners)
    listener->receive(*msg);
}

//==============================================================================
void FleetAdapterNode::lift_state_update(LiftState::UniquePtr msg)
{
  for (const auto& listener : lift_state_listeners)
    listener->receive(*msg);
}

} // namespace full_control
} // namespace rmf_fleet_adapter
