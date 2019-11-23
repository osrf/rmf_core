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
    FleetAdapterNode::RobotState* state,
    const Delivery& request)
: _delivery(request),
  _state_ptr(state)
{
  const auto& waypoint_keys = node->get_waypoint_keys();
  const auto pickup = waypoint_keys.find(_delivery.pickup_place_name);
  if (pickup == waypoint_keys.end())
  {
    _action_queue.push(
          make_failure(
            "Unknown pickup location [" + _delivery.pickup_place_name + "]"));
    return;
  }

  const auto dropoff = waypoint_keys.find(_delivery.dropoff_place_name);
  if (dropoff == waypoint_keys.end())
  {
    _action_queue.push(
          make_failure(
            "Uknown dropoff location [" + _delivery.dropoff_place_name + "]"));
    return;
  }

  _action_queue.push(make_move(
        node, state, pickup->second, node->get_fallback_wps()));

}

//==============================================================================
void FleetAdapterNode::Task::next()
{
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
}

//==============================================================================


} // namespace full_control
} // namespace rmf_fleet_adapter
