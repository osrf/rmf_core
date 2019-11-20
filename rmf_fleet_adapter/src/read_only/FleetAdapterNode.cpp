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

#include <rmf_traffic/agv/Interpolate.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rclcpp/macros.hpp>

namespace rmf_fleet_adapter {
namespace read_only {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make(
    const std::string& fleet_name,
    rmf_traffic::agv::VehicleTraits traits,
    rmf_traffic::Duration wait_time)
{
  const auto stop_waiting = std::chrono::steady_clock::now() + wait_time;

  auto node = std::make_shared<FleetAdapterNode>(fleet_name, std::move(traits));
  if(node->wait_until(stop_waiting))
    return node;

  RCLCPP_INFO(
        node->get_logger(),
        "Timeout while trying to connect to traffic schedule");
  return nullptr;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode(
    const std::string& fleet_name,
    rmf_traffic::agv::VehicleTraits traits)
: rclcpp::Node(fleet_name + "__read_only_fleet_adapter"),
  _fleet_name(fleet_name),
  _traits(std::move(traits))
{
  _client_submit_trajectories =
      create_client<SubmitTrajectories>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName);

  _client_delay_trajectories =
      create_client<DelayTrajectories>(
        rmf_traffic_ros2::DelayTrajectoriesSrvName);

  _client_replace_trajectories =
      create_client<ReplaceTrajectories>(
        rmf_traffic_ros2::ReplaceTrajectoriesSrvName);

  _fleet_state_subscription =
      create_subscription<FleetState>(
        FleetStateTopicName, rclcpp::SystemDefaultsQoS(),
        [&](FleetState::UniquePtr msg)
  {
    this->fleet_state_update(std::move(msg));
  });


}

//==============================================================================
bool FleetAdapterNode::wait_until(const rmf_traffic::Time stop_waiting) const
{
  if(!_client_submit_trajectories->wait_for_service(
        stop_waiting - std::chrono::steady_clock::now()))
    return false;

  if(!_client_delay_trajectories->wait_for_service(
        stop_waiting - std::chrono::steady_clock::now()))
    return false;

  if(!_client_replace_trajectories->wait_for_service(
        stop_waiting - std::chrono::steady_clock::now()))
    return false;

  return true;
}

//==============================================================================
void FleetAdapterNode::fleet_state_update(FleetState::UniquePtr state)
{
  for(const auto& robot : state->robots)
  {
    const auto it = _schedule_entries.find(robot.name);
    if (it == _schedule_entries.end())
      submit_robot(robot);
    else
      update_robot(robot, it);
  }
}

//==============================================================================
void FleetAdapterNode::submit_robot(const RobotState& state)
{
  const rmf_traffic::Trajectory trajectory = make_trajectory(state);
}

//==============================================================================
void FleetAdapterNode::update_robot(
    const RobotState& state,
    const ScheduleEntries::iterator& it)
{
  if(handle_delay(state, it))
    return;

  const rmf_traffic::Trajectory trajectory = make_trajectory(state);

  ReplaceTrajectories::Request msg;
  msg.replace_ids.push_back(it->second.schedule_id);
  msg.trajectories.emplace_back(rmf_traffic_ros2::convert(trajectory));

  using ReplaceTrajectoriesFuture =
      rclcpp::Client<ReplaceTrajectories>::SharedFuture;
  using namespace std::chrono_literals;

  _client_replace_trajectories->async_send_request(
        std::make_shared<ReplaceTrajectories::Request>(std::move(msg)),
        [=](const ReplaceTrajectoriesFuture future)
  {
    if (!future.valid() || std::future_status::ready != future.wait_for(0s))
    {
      RCLCPP_ERROR(
            this->get_logger(),
            "Failed to get response from scheduler");
      return;
    }

    const auto response = *future.get();
    if (!response.error.empty())
    {
      RCLCPP_ERROR(
            this->get_logger(),
            "Error response from scheduler: " + response.error);
    }

    const std::string robot_name = state.name;
    this->update_id(robot_name, response.current_version);
  });
}

//==============================================================================
rmf_traffic::Trajectory FleetAdapterNode::make_trajectory(
    const RobotState& state) const
{
  // TODO(MXG): Account for the multi-floor use case
  const std::string& map_name = state.location.level_name;

  std::vector<Eigen::Vector3d> positions;
  positions.push_back({state.location.x, state.location.y, state.location.yaw});
  for (const auto& location : state.path)
    positions.push_back({location.x, location.y, location.yaw});

  const auto trajectory = rmf_traffic::agv::Interpolate::positions(
        map_name, _traits, )

  if(state.path.size() < 1)
  {

  } // Put a compilation error here so I remember where I left off
}

//==============================================================================
void FleetAdapterNode::update_id(const std::string& name, uint64_t id)
{
  const auto it = _schedule_entries.find(name);
  if (it == _schedule_entries.end())
  {
    RCLCPP_INFO(
          get_logger(),
          "[read_only::FleetAdapterNode] Updating unrecognized robot: " + name);
    return;
  }

  it->second.schedule_id = id;
}

} // namespace read_only
} // namespace rmf_fleet_adapter
