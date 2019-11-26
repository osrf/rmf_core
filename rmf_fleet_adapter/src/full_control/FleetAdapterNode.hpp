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
#include <rmf_traffic_msgs/srv/erase_trajectories.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>

#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/task_summary.hpp>

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
#include "../rmf_fleet_adapter/ParseGraph.hpp"

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
template<typename T>
class Listener
{
public:

  virtual void receive(const T& msg) = 0;

  virtual ~Listener() = default;
};

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
  using WaypointNames = std::unordered_map<std::size_t, std::string>;

  using Delivery = rmf_task_msgs::msg::Delivery;
  using Location = rmf_fleet_msgs::msg::Location;

  using RobotState = rmf_fleet_msgs::msg::RobotState;
  using RobotStateListeners = std::unordered_set<Listener<RobotState>*>;

  class Task;
  struct RobotContext
  {
    Location location;
    std::unique_ptr<Task> task;
    std::queue<std::unique_ptr<Task>> task_queue;

    RobotStateListeners listeners;

    void next_task();
  };

  class Task
  {
  public:

    Task(FleetAdapterNode* node,
         RobotContext* state_ptr,
         const Delivery& request,
         std::size_t pickup_wp,
         std::size_t dropoff_wp);

    void next();

    void interrupt();

    void resume();

    void report_status();

    void critical_failure(const std::string& error);

    const std::string& id() const;

    const rclcpp::Time& start_time() const;

  private:

    const Delivery _delivery;
    FleetAdapterNode* const _node;
    RobotContext* const _context;
    std::unique_ptr<Action> _action;
    std::queue<std::unique_ptr<Action>> _action_queue;
    rmf_utils::optional<rclcpp::Time> _start_time;

  };

  const std::string& get_fleet_name() const;

  rmf_traffic::Duration get_plan_time() const;

  const rmf_traffic::agv::Planner& get_planner() const;

  const rmf_traffic::agv::Graph& get_graph() const;

  std::vector<rmf_traffic::agv::Plan::Start> compute_plan_starts(
      const Location& location);

  const WaypointKeys& get_waypoint_keys() const;

  const WaypointNames& get_waypoint_names() const;

  const std::vector<std::size_t>& get_parking_spots() const;


  using SubmitTrajectories = rmf_traffic_msgs::srv::SubmitTrajectories;
  using SubmitTrajectoriesClient = rclcpp::Client<SubmitTrajectories>;
  using SubmitTrajectoriesPtr = SubmitTrajectoriesClient::SharedPtr;

  using DelayTrajectories = rmf_traffic_msgs::srv::DelayTrajectories;
  using DelayTrajectoriesClient = rclcpp::Client<DelayTrajectories>;
  using DelayTrajectoriesPtr = DelayTrajectoriesClient::SharedPtr;

  using ReplaceTrajectories = rmf_traffic_msgs::srv::ReplaceTrajectories;
  using ReplaceTrajectoriesClient = rclcpp::Client<ReplaceTrajectories>;
  using ReplaceTrajectoriesPtr = ReplaceTrajectoriesClient::SharedPtr;

  using EraseTrajectories = rmf_traffic_msgs::srv::EraseTrajectories;
  using EraseTrajectoriesClient = rclcpp::Client<EraseTrajectories>;
  using EraseTrajectoriesPtr = EraseTrajectoriesClient::SharedPtr;

  struct Fields
  {
    rmf_traffic_ros2::schedule::MirrorManager mirror;

    SubmitTrajectoriesPtr submit_trajectories;
    DelayTrajectoriesPtr delay_trajectories;
    ReplaceTrajectoriesPtr replace_trajectories;
    EraseTrajectoriesPtr erase_trajectories;

    GraphInfo graph_info;
    rmf_traffic::agv::VehicleTraits traits;
    rmf_traffic::agv::Planner planner;

    Fields(
        GraphInfo graph_info_,
        rmf_traffic::agv::VehicleTraits traits_,
        rmf_traffic_ros2::schedule::MirrorManager mirror_,
        SubmitTrajectoriesPtr submit_trajectories_,
        DelayTrajectoriesPtr delay_trajectories_,
        ReplaceTrajectoriesPtr replace_trajectories_,
        EraseTrajectoriesPtr erase_trajectories_)
    : mirror(std::move(mirror_)),
      submit_trajectories(std::move(submit_trajectories_)),
      delay_trajectories(std::move(delay_trajectories_)),
      replace_trajectories(std::move(replace_trajectories_)),
      erase_trajectories(std::move(erase_trajectories_)),
      graph_info(std::move(graph_info_)),
      traits(std::move(traits_)),
      planner(
        rmf_traffic::agv::Planner::Configuration(graph_info.graph, traits),
        rmf_traffic::agv::Planner::Options(mirror.viewer()))
    {
      // Do nothing
    }
  };

  const Fields& get_fields() const;

  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorStateListeners = std::unordered_set<Listener<DoorState>*>;
  DoorStateListeners door_state_listeners;

  using LiftState = rmf_lift_msgs::msg::LiftState;
  using LiftStateListeners = std::unordered_set<Listener<LiftState>*>;
  LiftStateListeners lift_state_listeners;

  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;

  using DispenserResultListeners =
      std::unordered_set<Listener<DispenserResult>*>;
  DispenserResultListeners dispenser_result_listeners;

  using DispenserStateListeners =
      std::unordered_set<Listener<DispenserState>*>;
  DispenserStateListeners dispenser_state_listeners;

  using PathRequest = rmf_fleet_msgs::msg::PathRequest;
  using PathRequestPub = rclcpp::Publisher<PathRequest>;
  PathRequestPub::SharedPtr path_request_publisher;

  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorRequestPub = rclcpp::Publisher<DoorRequest>;
  DoorRequestPub::SharedPtr door_request_publisher;

  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using LiftRequestPub = rclcpp::Publisher<LiftRequest>;
  LiftRequestPub::SharedPtr lift_request_publisher;

  using TaskSummary = rmf_task_msgs::msg::TaskSummary;
  using TaskSummaryPub = rclcpp::Publisher<TaskSummary>;
  TaskSummaryPub::SharedPtr task_summary_publisher;

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

  using DispenserResultSub = rclcpp::Subscription<DispenserResult>;
  DispenserResultSub::SharedPtr _dispenser_result_sub;
  void dispenser_result_update(DispenserResult::UniquePtr msg);

  using DispenserStateSub = rclcpp::Subscription<DispenserState>;
  DispenserStateSub::SharedPtr _dispenser_state_sub;
  void dispenser_state_update(DispenserState::UniquePtr msg);

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateSub = rclcpp::Subscription<FleetState>;
  FleetStateSub::SharedPtr _fleet_state_sub;
  void fleet_state_update(FleetState::UniquePtr msg);

  using DoorStateSub = rclcpp::Subscription<DoorState>;
  DoorStateSub::SharedPtr _door_state_sub;
  void door_state_update(DoorState::UniquePtr msg);

  using LiftStateSub = rclcpp::Subscription<LiftState>;
  LiftStateSub::SharedPtr _lift_state_sub;
  void lift_state_update(LiftState::UniquePtr msg);

  using Context =
      std::unordered_map<std::string, std::unique_ptr<RobotContext>>;
  Context _contexts;

};

} // namespace full_control
} // namespace rmf_fleet_adapter

#endif // SRC__FULL_CONTROL__FLEETADAPTERNODE_HPP
