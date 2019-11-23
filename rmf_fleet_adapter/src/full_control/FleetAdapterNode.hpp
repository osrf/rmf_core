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

#ifndef SRC__FULL_CONTROL__FLEETADAPTERNODE_HPP
#define SRC__FULL_CONTROL__FLEETADAPTERNODE_HPP

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>

#include <rmf_traffic_msgs/srv/submit_trajectories.hpp>
#include <rmf_traffic_msgs/srv/delay_trajectories.hpp>
#include <rmf_traffic_msgs/srv/replace_trajectories.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rclcpp/node.hpp>

#include <rmf_utils/optional.hpp>

#include <unordered_map>
#include <vector>
#include <queue>

#include "Action.hpp"

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
class FleetAdapterNode : public rclcpp::Node
{
public:

  static std::shared_ptr<FleetAdapterNode> make(
      const std::string& fleet_name,
      const std::string& graph_file,
      rmf_traffic::agv::VehicleTraits traits,
      rmf_traffic::Duration get_plan_time,
      rmf_traffic::Duration wait_time = std::chrono::seconds(10));

  using WaypointKeys = std::unordered_map<std::string, std::size_t>;

  using Delivery = rmf_task_msgs::msg::Delivery;
  using Location = rmf_fleet_msgs::msg::Location;

  class Task;
  struct RobotState
  {
    Location location;
    std::unique_ptr<Task> task;
    std::queue<Delivery> request_queue;

    void next_task();
  };

  class Task
  {
  public:

    Task(const FleetAdapterNode* node,
         RobotState* state_ptr,
         const Delivery& request);

    void next();

    void interrupt();

    void resume();

    void report_status();

    void critical_failure(const std::string& error);

  private:

    const Delivery _delivery;
    RobotState* const _state_ptr;
    std::unique_ptr<Action> _action;
    std::queue<std::unique_ptr<Action>> _action_queue;


  };

  const std::string& get_fleet_name() const;

  rmf_traffic::Duration get_plan_time() const;

  const rmf_traffic::agv::Planner& get_planner() const;

  const rmf_traffic::agv::Graph& get_graph() const;

  std::size_t compute_closest_wp(const Location& location) const;

  const WaypointKeys& get_waypoint_keys() const;

  const std::vector<std::size_t>& get_fallback_wps() const;


  using SubmitTrajectories = rmf_traffic_msgs::srv::SubmitTrajectories;
  using SubmitTrajectoriesClient = rclcpp::Client<SubmitTrajectories>;
  using SubmitTrajectoriesPtr = SubmitTrajectoriesClient::SharedPtr;

  using DelayTrajectories = rmf_traffic_msgs::srv::DelayTrajectories;
  using DelayTrajectoriesClient = rclcpp::Client<DelayTrajectories>;
  using DelayTrajectoriesPtr = DelayTrajectoriesClient::SharedPtr;

  using ReplaceTrajectories = rmf_traffic_msgs::srv::ReplaceTrajectories;
  using ReplaceTrajectoriesClient = rclcpp::Client<ReplaceTrajectories>;
  using ReplaceTrajectoriesPtr = ReplaceTrajectoriesClient::SharedPtr;

  struct Fields
  {
    rmf_traffic_ros2::schedule::MirrorManager mirror;

    SubmitTrajectoriesPtr submit_trajectories;
    DelayTrajectoriesPtr delay_trajectories;
    ReplaceTrajectoriesPtr replace_trajectories;

    rmf_traffic::agv::Graph graph;
    rmf_traffic::agv::VehicleTraits traits;
    rmf_traffic::agv::Planner planner;
    WaypointKeys waypoint_keys;
    std::vector<std::size_t> fallback_waypoints;

    Fields(
        rmf_traffic::agv::Graph graph_,
        rmf_traffic::agv::VehicleTraits traits_,
        rmf_traffic_ros2::schedule::MirrorManager mirror_,
        SubmitTrajectoriesPtr submit_trajectories_,
        DelayTrajectoriesPtr delay_trajectories_,
        ReplaceTrajectoriesPtr replace_trajectories_)
    : mirror(std::move(mirror_)),
      submit_trajectories(std::move(submit_trajectories_)),
      delay_trajectories(std::move(delay_trajectories_)),
      replace_trajectories(std::move(replace_trajectories_)),
      graph(std::move(graph_)),
      traits(std::move(traits_)),
      planner(
        rmf_traffic::agv::Planner::Configuration(graph, traits),
        rmf_traffic::agv::Planner::Options(mirror.viewer()))
    {
      // Do nothing
    }
  };

  const Fields& get_fields() const;

private:

  FleetAdapterNode(
      const std::string& fleet_name,
      rmf_traffic::Duration get_plan_time);

  std::string _fleet_name;

  rmf_traffic::Duration _plan_time;

  void start(Fields fields);

  rmf_utils::optional<Fields> _field;

  using DeliverySub = rclcpp::Subscription<Delivery>;
  using DeliveryPtr = DeliverySub::SharedPtr;
  DeliveryPtr _delivery_sub;
  void delivery_request(Delivery::UniquePtr msg);


  using States =
      std::unordered_map<std::string, std::unique_ptr<RobotState>>;

};

} // namespace full_control
} // namespace rmf_fleet_adapter

#endif // SRC__FULL_CONTROL__FLEETADAPTERNODE_HPP
