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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP

#include <rmf_task_msgs/msg/loop.hpp>

#include <rmf_task_msgs/msg/bid_proposal.hpp>
#include <rmf_task_msgs/msg/bid_notice.hpp>
#include <rmf_task_msgs/msg/dispatch_request.hpp>

#include <rmf_task/agv/TaskPlanner.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/requests/Clean.hpp>

#include <rmf_fleet_msgs/msg/dock_summary.hpp>

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include "Node.hpp"
#include "RobotContext.hpp"
#include "../TaskManager.hpp"

#include <rmf_traffic/schedule/Snapshot.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/Trajectory.hpp>

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <iostream>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// This abstract interface class allows us to use the same implementation of
/// FleetUpdateHandle whether we are running it in a distributed system or in a
/// single-process testing environment.
class ParticipantFactory
{
public:

  using ReadyCallback = std::function<void(rmf_traffic::schedule::Participant)>;

  virtual void async_make_participant(
      rmf_traffic::schedule::ParticipantDescription description,
      ReadyCallback ready_callback) = 0;

  virtual ~ParticipantFactory() = default;
};

//==============================================================================
class SimpleParticipantFactory : public ParticipantFactory
{
public:

  SimpleParticipantFactory(
      std::shared_ptr<rmf_traffic::schedule::Writer> writer)
    : _writer{std::move(writer)}
  {
    // Do nothing
  }

  void async_make_participant(
      rmf_traffic::schedule::ParticipantDescription description,
      ReadyCallback ready_callback) final
  {
    ready_callback(
        rmf_traffic::schedule::make_participant(
          std::move(description),
          _writer,
          nullptr)
    );
  }

private:
  std::shared_ptr<rmf_traffic::schedule::Writer> _writer;
};

//==============================================================================
class ParticipantFactoryRos2 : public ParticipantFactory
{
public:

  ParticipantFactoryRos2(
      rmf_traffic_ros2::schedule::WriterPtr writer)
    : _writer{std::move(writer)}
  {
    // Do nothing
  }

  void async_make_participant(
      rmf_traffic::schedule::ParticipantDescription description,
      ReadyCallback ready_callback) final
  {
    _writer->async_make_participant(
          std::move(description),
          std::move(ready_callback));
  }

private:
  rmf_traffic_ros2::schedule::WriterPtr _writer;
};

//==============================================================================
class FleetUpdateHandle::Implementation
{
public:

  std::string name;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  std::shared_ptr<Node> node;
  rxcpp::schedulers::worker worker;
  std::shared_ptr<ParticipantFactory> writer;
  std::shared_ptr<rmf_traffic::schedule::Snappable> snappable;
  std::shared_ptr<rmf_traffic_ros2::schedule::Negotiation> negotiation;

  // Task planner params
  std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system = nullptr;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink = nullptr;
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink = nullptr;
  std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink = nullptr;
  const bool drain_battery = true;
  std::shared_ptr<rmf_task::agv::TaskPlanner> task_planner = nullptr;
  bool initialized_task_planner = false;

  rmf_utils::optional<rmf_traffic::Duration> default_maximum_delay =
      std::chrono::nanoseconds(std::chrono::seconds(10));

  AcceptDeliveryRequest accept_delivery = nullptr;
  std::unordered_map<RobotContextPtr, std::shared_ptr<TaskManager>> task_managers = {};

  // Map task id to pair of <RequestPtr, Assignments>
  using Assignments = rmf_task::agv::TaskPlanner::Assignments;
  std::unordered_map<std::string,
    std::pair<rmf_task::RequestPtr, Assignments>> task_map = {};

  // Map of dock name to dock parameters
  std::unordered_map<std::string,
    rmf_fleet_msgs::msg::DockParameter> dock_param_map = {};

  AcceptTaskRequest accept_task = nullptr;

  using BidNotice = rmf_task_msgs::msg::BidNotice;
  using BidNoticeSub = rclcpp::Subscription<BidNotice>::SharedPtr;
  BidNoticeSub bid_notice_sub = nullptr;

  using BidProposal = rmf_task_msgs::msg::BidProposal;
  using BidProposalPub = rclcpp::Publisher<BidProposal>::SharedPtr;
  BidProposalPub bid_proposal_pub = nullptr;

  using DispatchRequest = rmf_task_msgs::msg::DispatchRequest;
  using DispatchRequestSub = rclcpp::Subscription<DispatchRequest>::SharedPtr;
  DispatchRequestSub dispatch_request_sub = nullptr;

  using DockSummary = rmf_fleet_msgs::msg::DockSummary;
  using DockSummarySub = rclcpp::Subscription<DockSummary>::SharedPtr;
  DockSummarySub dock_summary_sub = nullptr;

  template<typename... Args>
  static std::shared_ptr<FleetUpdateHandle> make(Args&&... args)
  {
    FleetUpdateHandle handle;
    handle._pimpl = rmf_utils::make_unique_impl<Implementation>(
          Implementation{std::forward<Args>(args)...});

    // Create subs and pubs for bidding
    auto default_qos = rclcpp::SystemDefaultsQoS();
    auto transient_qos = default_qos; transient_qos.transient_local();
    
    // Publish BidProposal
    handle._pimpl->bid_proposal_pub =
      handle._pimpl->node->create_publisher<BidProposal>(
        BidProposalTopicName, default_qos);

    // Subscribe BidNotice
    handle._pimpl->bid_notice_sub =
      handle._pimpl->node->create_subscription<BidNotice>(
        BidNoticeTopicName,
        default_qos,
        [p = handle._pimpl.get()](const BidNotice::SharedPtr msg)
        {
          p->bid_notice_cb(msg);
        });

    // Subscribe DispatchRequest
    handle._pimpl->dispatch_request_sub =
      handle._pimpl->node->create_subscription<DispatchRequest>(
        DispatchRequestTopicName,
        default_qos,
        [p = handle._pimpl.get()](const DispatchRequest::SharedPtr msg)
        {
          // TODO(YV)
        });

    // Subscribe DockSummary
    handle._pimpl->dock_summary_sub =
      handle._pimpl->node->create_subscription<DockSummary>(
        DockSummaryTopicName,
        transient_qos,
        [p = handle._pimpl.get()](const DockSummary::SharedPtr msg)
        {
          p->dock_summary_cb(msg);
        });

    return std::make_shared<FleetUpdateHandle>(std::move(handle));
  }

  // struct DeliveryEstimate
  // {
  //   rmf_traffic::Time time = rmf_traffic::Time::max();
  //   RobotContextPtr robot = nullptr;
  //   rmf_utils::optional<rmf_traffic::agv::Plan::Start> pickup_start;
  //   rmf_utils::optional<rmf_traffic::agv::Plan::Start> dropoff_start;
  //   rmf_utils::optional<rmf_traffic::agv::Plan::Start> finish;
  // };

  // struct LoopEstimate
  // {
  //   rmf_traffic::Time time = rmf_traffic::Time::max();
  //   RobotContextPtr robot = nullptr;
  //   rmf_utils::optional<rmf_traffic::agv::Plan::Start> init_start;
  //   rmf_utils::optional<rmf_traffic::agv::Plan::Start> loop_start;
  //   rmf_utils::optional<rmf_traffic::agv::Plan::Start> loop_end;
  // };

  void dock_summary_cb(const DockSummary::SharedPtr msg)
  {
    for (const auto& dock : msg->docks)
    {
      if (dock.fleet_name == name)
      {
        dock_param_map.clear();
        for (const auto& param : dock.params)
          dock_param_map.insert({param.start, param});
        break;
      }
    }

    return;
  }

  void bid_notice_cb(const BidNotice::SharedPtr msg)
  {
    if (!accept_task)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Fleet [%s] is not configured to accept any task requests. Use "
        "FleetUpdateHadndle::accept_task_requests(~) to define a callback "
        "for accepting requests", name.c_str());

      return;
    }

    if (!accept_task(msg->task_profile))
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] is configured to not accept task [%s]",
        name.c_str(),
        msg->task_profile.task_id.c_str());

        return;
    }

    if (!task_planner
      || !initialized_task_planner)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Fleet [%s] is not configured with parameters for task planning."
        "Use FleetUpdateHandle::set_task_planner_params(~) to set the "
        "parameters required.", name.c_str());

      return;
    }

    // Determine task type and convert to request pointer
    rmf_task::RequestPtr new_request = nullptr;
    const auto& task_profile = msg->task_profile;
    const auto& task_type = task_profile.type;
    const rmf_traffic::Time start_time = rmf_traffic_ros2::convert(task_profile.start_time);
    // TODO (YV) get rid of ID field in RequestPtr
    std::string id = msg->task_profile.task_id;
    const auto& graph = planner->get_configuration().graph();

    RCLCPP_INFO(
      node->get_logger(),
      "Fleet [%s] is processing BidNotice with task_id:[%s] and type:[%d]...",
      name.c_str(), id.c_str(), task_type.value);

    // Process Cleaning task
    if (task_type.value == rmf_task_msgs::msg::TaskType::CLEANING_TASK)
    {
      if (task_profile.params.empty())
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Required param [zone] missing in TaskProfile. Rejecting BidNotice "
          " with task_id:[%s]" , id.c_str());

        return;
      }

      // Check for valid start waypoint
      const std::string start_wp_name = task_profile.params[0].value;
      const auto start_wp = graph.find_waypoint(start_wp_name);
      if (!start_wp)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Fleet [%s] does not have a named waypoint [%s] configured in its "
          "nav graph. Rejecting BidNotice with task_id:[%s]",
          name.c_str(), start_wp_name.c_str(), id.c_str());

          return;
      }

      // Get dock parameters
      const auto clean_param_it = dock_param_map.find(start_wp_name);
      if (clean_param_it == dock_param_map.end())
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Dock param for dock_name:[%s] unavailable. Rejecting BidNotice with "
          "task_id:[%s]", start_wp_name.c_str(), id.c_str());

        return;
      }
      const auto clean_param = clean_param_it->second;

      // Check for valid finish waypoint
      const std::string finish_wp_name = clean_param.finish;
      const auto finish_wp = graph.find_waypoint(finish_wp_name);
      if (!finish_wp)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Fleet [%s] does not have a named waypoint [%s] configured in its "
          "nav graph. Rejecting BidNotice with task_id:[%s]",
          name.c_str(), finish_wp_name.c_str(), id.c_str());

          return;
      }

      // Interpolate docking waypoint into trajectory
      std::vector<Eigen::Vector3d>  positions;
      for (const auto& location: clean_param.path)
        positions.push_back({location.x, location.y, location.yaw});

      rmf_traffic::Trajectory cleaning_trajectory =
        rmf_traffic::agv::Interpolate::positions(
          planner->get_configuration().vehicle_traits(),
          start_time,
          positions); 

      // TODO(YV) get rid of id field in RequestPtr
      std::stringstream id_stream(id);
      std::size_t request_id;
      id_stream >> request_id;

      new_request = rmf_task::requests::Clean::make(
        request_id,
        start_wp->index(),
        finish_wp->index(),
        cleaning_trajectory,
        motion_sink,
        ambient_sink,
        tool_sink,
        planner,
        start_time,
        drain_battery);
    }

    // TODO(YV)
    else if (task_type.value == rmf_task_msgs::msg::TaskType::DELIVERY_TASK)
    {

    }
    else if (task_type.value == rmf_task_msgs::msg::TaskType::LOOP_TASK)
    {

    }
    else
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Invalid TaskType in TaskProfile. Rejecting BidNotice with task_id:[%s]",
        id.c_str());

      return;
    }
    

    if (!new_request)
      return;

    // Combine new request ptr with request ptr of tasks in task manager queues

    // Update robot states

    // Generate new task assignments while accommodating for the new
    // request
    // Call greedy_plan but run optimal_plan() in a separate thread

    // Store results in internal map and publish BidProposal
  }

  static Implementation& get(FleetUpdateHandle& fleet)
  {
    return *fleet._pimpl;
  }

  static const Implementation& get(const FleetUpdateHandle& fleet)
  {
    return *fleet._pimpl;
  }

  // TODO(MXG): Come up with a better design for task dispatch
  // rmf_utils::optional<DeliveryEstimate> estimate_delivery(
  //     const rmf_task_msgs::msg::Delivery& request) const;

  // void perform_delivery(
  //     const rmf_task_msgs::msg::Delivery& request,
  //     const DeliveryEstimate& estimate);

  // rmf_utils::optional<LoopEstimate> estimate_loop(
  //     const rmf_task_msgs::msg::Loop& request) const;

  // void perform_loop(
  //     const rmf_task_msgs::msg::Loop& request,
  //     const LoopEstimate& estimate);
};

//==============================================================================
// void request_delivery(
//     const rmf_task_msgs::msg::Delivery& request,
//     const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets);

// //==============================================================================
// void request_loop(
//     const rmf_task_msgs::msg::Loop& request,
//     const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets);

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_FLEETUPDATEHANDLE_HPP
