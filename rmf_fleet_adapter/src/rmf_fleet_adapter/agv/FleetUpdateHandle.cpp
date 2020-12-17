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

#include "internal_FleetUpdateHandle.hpp"
#include "internal_RobotUpdateHandle.hpp"
#include "RobotContext.hpp"

#include "../tasks/Delivery.hpp"
#include "../tasks/Loop.hpp"

#include <rmf_task/requests/Clean.hpp>
#include <rmf_task/requests/Delivery.hpp>
#include <rmf_task/requests/Loop.hpp>

#include <rmf_task_msgs/msg/clean.hpp>
#include <rmf_task_msgs/msg/delivery.hpp> 
#include <rmf_task_msgs/msg/loop.hpp>

#include <iostream>
#include <unordered_map>

namespace rmf_fleet_adapter {
namespace agv {

namespace {
//==============================================================================
class LiaisonNegotiator : public rmf_traffic::schedule::Negotiator
{
public:

  LiaisonNegotiator(
      std::shared_ptr<rmf_traffic::schedule::Negotiator> negotiator)
    : w_negotiator(negotiator)
  {
    // Do nothing
  }

  std::weak_ptr<rmf_traffic::schedule::Negotiator> w_negotiator;

  void respond(
      const TableViewerPtr& table_viewer,
      const ResponderPtr& responder) final
  {
    const auto negotiator = w_negotiator.lock();
    if (!negotiator)
    {
      // If we no longer have access to the upstream negotiator, then we simply
      // forfeit.
      //
      // TODO(MXG): Consider issuing a warning here
      return responder->forfeit({});
    }

    negotiator->respond(table_viewer, responder);
  }

};
} // anonymous namespace

//==============================================================================
void FleetUpdateHandle::Implementation::dock_summary_cb(
  const DockSummary::SharedPtr& msg)
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

//==============================================================================
void FleetUpdateHandle::Implementation::bid_notice_cb(
  const BidNotice::SharedPtr msg)
{
  if (task_managers.empty())
    return;

  if (msg->task_profile.task_id.empty())
    return;

  if (bid_notice_assignments.find(msg->task_profile.task_id)
      != bid_notice_assignments.end())
    return;

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
  rmf_task::ConstRequestPtr new_request = nullptr;
  const auto& task_profile = msg->task_profile;
  const auto& task_type = task_profile.task_type;
  const rmf_traffic::Time start_time = rmf_traffic_ros2::convert(task_profile.start_time);
  // TODO (YV) get rid of ID field in RequestPtr
  std::string id = msg->task_profile.task_id;
  const auto& graph = planner->get_configuration().graph();

  // Process Cleaning task
  if (task_type.type == rmf_task_msgs::msg::TaskType::TYPE_CLEAN)
  {
    if (task_profile.clean.start_waypoint.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [clean.start_waypoint] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    // Check for valid start waypoint
    const std::string start_wp_name = task_profile.clean.start_waypoint;
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
    const auto& clean_param = clean_param_it->second;

    // Check for valid finish waypoint
    const std::string& finish_wp_name = clean_param.finish;
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
    
    if (cleaning_trajectory.size() == 0)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Unable to generate cleaning trajectory from positions specified "
        " in DockSummary msg for [%s]", start_wp_name.c_str());
      
      return;
    }

    new_request = rmf_task::requests::Clean::make(
      id,
      start_wp->index(),
      finish_wp->index(),
      cleaning_trajectory,
      motion_sink,
      ambient_sink,
      tool_sink,
      planner,
      start_time,
      drain_battery);

    RCLCPP_INFO(
      node->get_logger(),
      "Generated Clean request for task_id:[%s]", id.c_str());
  }

  else if (task_type.type == rmf_task_msgs::msg::TaskType::TYPE_DELIVERY)
  {
    const auto& delivery = task_profile.delivery;
    if (delivery.pickup_place_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.pickup_place_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    if (delivery.pickup_dispenser.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.pickup_dispenser] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    if (delivery.dropoff_place_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.dropoff_place_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    if (delivery.dropoff_place_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.dropoff_place_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    if (delivery.dropoff_ingestor.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [delivery.dropoff_ingestor] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    const auto pickup_wp = graph.find_waypoint(delivery.pickup_place_name);
    if (!pickup_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), delivery.pickup_place_name.c_str(), id.c_str());

        return;
    }

    const auto dropoff_wp = graph.find_waypoint(delivery.dropoff_place_name);
    if (!dropoff_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), delivery.dropoff_place_name.c_str(), id.c_str());

        return;
    }

    new_request = rmf_task::requests::Delivery::make(
      id,
      pickup_wp->index(),
      delivery.pickup_dispenser,
      dropoff_wp->index(),
      delivery.dropoff_ingestor,
      delivery.items,
      motion_sink,
      ambient_sink,
      planner,
      start_time,
      drain_battery);

    RCLCPP_INFO(
      node->get_logger(),
      "Generated Delivery request for task_id:[%s]", id.c_str());

  }
  else if (task_type.type == rmf_task_msgs::msg::TaskType::TYPE_LOOP)
  {
    const auto& loop = task_profile.loop;
    if (loop.start_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [loop.start_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    if (loop.finish_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [loop.finish_name] missing in TaskProfile."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    if (loop.num_loops < 1)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Required param [loop.num_loops] in TaskProfile is invalid."
        "Rejecting BidNotice with task_id:[%s]" , id.c_str());

      return;
    }

    const auto start_wp = graph.find_waypoint(loop.start_name);
    if (!start_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), loop.start_name.c_str(), id.c_str());

        return;
    }

    const auto finish_wp = graph.find_waypoint(loop.finish_name);
    if (!finish_wp)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] does not have a named waypoint [%s] configured in its "
        "nav graph. Rejecting BidNotice with task_id:[%s]",
        name.c_str(), loop.finish_name.c_str(), id.c_str());

        return;
    }

    new_request = rmf_task::requests::Loop::make(
      id,
      start_wp->index(),
      finish_wp->index(),
      loop.num_loops,
      motion_sink,
      ambient_sink,
      planner,
      start_time,
      drain_battery);

    RCLCPP_INFO(
      node->get_logger(),
      "Generated Loop request for task_id:[%s]", id.c_str());
  }
  else
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid TaskType [%d] in TaskProfile. Rejecting BidNotice with "
      "task_id:[%s]",
      task_type.type, id.c_str());

    return;
  }
  
  if (!new_request)
    return;

  // Collate robot states and combine new requestptr with requestptr of
  // non-charging tasks in task manager queues
  std::vector<rmf_task::agv::State> states;
  std::vector<rmf_task::agv::StateConfig> state_configs;
  std::vector<rmf_task::ConstRequestPtr> pending_requests;
  pending_requests.push_back(new_request);
  // Map robot index to name for BidProposal
  std::unordered_map<std::size_t, std::string> robot_name_map;
  std::size_t index = 0;
  for (const auto& t : task_managers)
  {
    states.push_back(t.second->expected_finish_state());
    state_configs.push_back(t.first->state_config());
    const auto requests = t.second->requests();
    pending_requests.insert(pending_requests.end(), requests.begin(), requests.end());

    robot_name_map.insert({index, t.first->name()});
    ++index;
  }

  RCLCPP_INFO(
    node->get_logger(), 
    "Planning for [%d] robot and [%d] request(s)", 
    states.size(), pending_requests.size());

  // Generate new task assignments while accommodating for the new
  // request
  const auto result = task_planner->optimal_plan(
    rmf_traffic_ros2::convert(node->now()),
    states,
    state_configs,
    pending_requests,
    nullptr);

  auto assignments_ptr = std::get_if<
    rmf_task::agv::TaskPlanner::Assignments>(&result);

  if (!assignments_ptr)
  {
    auto error = std::get_if<
      rmf_task::agv::TaskPlanner::TaskPlannerError>(&result);

    if (*error == rmf_task::agv::TaskPlanner::TaskPlannerError::low_battery)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "[TaskPlanner] Failed to compute assignments for task_id:[%s] due to"
        " insufficient initial battery charge for all robots in this fleet.",
        id.c_str());
    }

    else if (*error ==
      rmf_task::agv::TaskPlanner::TaskPlannerError::limited_capacity)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "[TaskPlanner] Failed to compute assignments for task_id:[%s] due to"
        " insufficient battery capacity to accommodate one or more requests by"
        " any of the robots in this fleet.", id.c_str());
    }

    else
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "[TaskPlanner] Failed to compute assignments for task_id:[%s]",
        id.c_str());
    }

    return;
  }

  const auto assignments = *assignments_ptr;

  if (assignments.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "[TaskPlanner] Failed to compute assignments for task_id:[%s]",
      id.c_str());

    return;
  }

  const double cost = task_planner->compute_cost(assignments);

  // Display assignments for debugging
  std::cout << "Cost: " << cost << std::endl;
  for (std::size_t i = 0; i < assignments.size(); ++i)
  {
    std:: cout << "--Agent: " << i << std::endl;
    for (const auto& a : assignments[i])
    {
      const auto& s = a.state();
      const double request_seconds = a.request()->earliest_start_time().time_since_epoch().count()/1e9;
      const double start_seconds = a.deployment_time().time_since_epoch().count()/1e9;
      const rmf_traffic::Time finish_time = s.finish_time();
      const double finish_seconds = finish_time.time_since_epoch().count()/1e9;
      std::cout << "    <" << a.request()->id() << ": " << request_seconds
                << ", " << start_seconds 
                << ", "<< finish_seconds << ", " << 100* s.battery_soc() 
                << "%>" << std::endl;
    }
  }
  std::cout << " ----------------------" << std::endl;

  // Publish BidProposal
  rmf_task_msgs::msg::BidProposal bid_proposal;
  bid_proposal.fleet_name = name;
  bid_proposal.task_profile = task_profile;
  bid_proposal.prev_cost = current_assignment_cost;
  bid_proposal.new_cost = cost;
  index = 0;
  for (const auto& agent : assignments)
  {
    for (const auto& assignment : agent)
    {
      if (assignment.request()->id() == id)
      {
        bid_proposal.finish_time = rmf_traffic_ros2::convert(
            assignment.state().finish_time());
        if (robot_name_map.find(index) != robot_name_map.end())
          bid_proposal.robot_name = robot_name_map[index];
        break;
      }
    }
    ++index;
  }
  bid_proposal_pub->publish(bid_proposal);
  RCLCPP_INFO(
    node->get_logger(),
    "Submitted BidProposal to accommodate task [%s] with new cost [%f]",
    id.c_str(), cost);

  // Store assignments in internal map
  bid_notice_assignments.insert({id, assignments});

}

void FleetUpdateHandle::Implementation::dispatch_request_cb(
  const DispatchRequest::SharedPtr msg)
{
  if (msg->fleet_name != name)
    return;

  const std::string id = msg->task_profile.task_id;
  const auto task_it = bid_notice_assignments.find(id);

  if (task_it == bid_notice_assignments.end())
    return;

  RCLCPP_INFO(
    node->get_logger(),
    "Bid for task_id:[%s] awarded to fleet [%s]",
    id.c_str(), name.c_str());

  // We currently only support adding tasks
  if (msg->method != DispatchRequest::ADD)
    return;
  
  const auto& assignments = task_it->second;
  
  if (assignments.size() != task_managers.size())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "The number of available robots do not match that in the assignments "
      "for task_id:[%s]. This request will be ignored", id.c_str());

    return;
  }

  std::size_t index = 0;
  for (auto& t : task_managers)
  {
    t.second->set_queue(assignments[index]);
    ++index;
  }

  current_assignment_cost = task_planner->compute_cost(assignments);

  RCLCPP_INFO(
    node->get_logger(),
    "Assignments updated for robots in fleet [%s]",
    name.c_str());
}

//==============================================================================
std::size_t FleetUpdateHandle::Implementation::get_nearest_charger(
  const rmf_traffic::agv::Planner::Start& start,
  const std::unordered_set<std::size_t>& charging_waypoints)
{
  assert(!charging_waypoints.empty());
  const auto& graph = planner->get_configuration().graph();
  Eigen::Vector2d p = graph.get_waypoint(start.waypoint()).get_location();

  if (start.location().has_value())
    p = *start.location();

  double min_dist = std::numeric_limits<double>::max();
  std::size_t nearest_charger = 0;
  for (const auto& wp : charging_waypoints)
  {
    const auto loc = graph.get_waypoint(wp).get_location();
    const double dist = (loc - p).norm();
    if (dist < min_dist)
    {
      min_dist = dist;
      nearest_charger = wp;
    }
  }

  return nearest_charger;
}

//==============================================================================
void FleetUpdateHandle::add_robot(
    std::shared_ptr<RobotCommandHandle> command,
    const std::string& name,
    const rmf_traffic::Profile& profile,
    rmf_traffic::agv::Plan::StartSet start,
    std::function<void(std::shared_ptr<RobotUpdateHandle>)> handle_cb)
{
  assert(!start.empty());
  rmf_traffic::schedule::ParticipantDescription description(
        name,
        _pimpl->name,
        rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
        profile);

  _pimpl->writer->async_make_participant(
        std::move(description),
        [worker = _pimpl->worker,
         command = std::move(command),
         start = std::move(start),
         handle_cb = std::move(handle_cb),
         fleet = shared_from_this()](
        rmf_traffic::schedule::Participant participant)
  {
    // TODO(YV) Get the battery % of this robot
    const std::size_t charger = fleet->_pimpl->get_nearest_charger(
        start[0], fleet->_pimpl->charging_waypoints);
    rmf_task::agv::State state = rmf_task::agv::State{start[0], charger, 1.0};
    rmf_task::agv::StateConfig state_config = rmf_task::agv::StateConfig{
      fleet->_pimpl->recharge_threshold};
    auto context = std::make_shared<RobotContext>(
          RobotContext{
            std::move(command),
            std::move(start),
            std::move(participant),
            fleet->_pimpl->snappable,
            fleet->_pimpl->planner,
            fleet->_pimpl->node,
            fleet->_pimpl->worker,
            fleet->_pimpl->default_maximum_delay,
            state,
            state_config,
            fleet->_pimpl->task_planner
          });

    // We schedule the following operations on the worker to make sure we do not
    // have a multiple read/write race condition on the FleetUpdateHandle.
    worker.schedule(
          [context, fleet, node = fleet->_pimpl->node,
           handle_cb = std::move(handle_cb)](const auto&)
    {
      // TODO(MXG): We need to perform this test because we do not currently
      // support the distributed negotiation in unit test environments. We
      // should create an abstract NegotiationRoom interface in rmf_traffic and
      // use that instead.
      if (fleet->_pimpl->negotiation)
      {
        context->_negotiation_license =
            fleet->_pimpl->negotiation
            ->register_negotiator(
              context->itinerary().id(),
              std::make_unique<LiaisonNegotiator>(context));
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Added a robot named [%s] with participant ID [%d]",
        context->name().c_str(), context->itinerary().id());

      fleet->_pimpl->task_managers.insert({context, TaskManager::make(context)});
      if (handle_cb)
      {
        handle_cb(RobotUpdateHandle::Implementation::make(std::move(context)));
      }
      else
      {
        RCLCPP_WARN(
          node->get_logger(),
          "FleetUpdateHandle::add_robot(~) was not provided a callback to "
          "receive the RobotUpdateHandle of the new robot. This means you will "
          "not be able to update the state of the new robot. This is likely to "
          "be a fleet adapter development error.");
      }
    });
  });
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::accept_task_requests(
    AcceptTaskRequest check)
{
  _pimpl->accept_task = std::move(check);
  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::accept_delivery_requests(
    AcceptDeliveryRequest check)
{
  _pimpl->accept_delivery = std::move(check);
  return *this;
}

//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::default_maximum_delay(
    rmf_utils::optional<rmf_traffic::Duration> value)
{
  _pimpl->default_maximum_delay = value;
  return *this;
}

//==============================================================================
rmf_utils::optional<rmf_traffic::Duration>
FleetUpdateHandle::default_maximum_delay() const
{
  return _pimpl->default_maximum_delay;
}

bool FleetUpdateHandle::set_task_planner_params(
    std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink)
{
  if (battery_system && motion_sink && ambient_sink && tool_sink)
  {

    _pimpl->battery_system  = battery_system;
    _pimpl->motion_sink = motion_sink;
    _pimpl->ambient_sink = ambient_sink;
    _pimpl->tool_sink = tool_sink;

    std::shared_ptr<rmf_task::agv::TaskPlanner::Configuration> task_config =
      std::make_shared<rmf_task::agv::TaskPlanner::Configuration>(
        *battery_system,
        motion_sink,
        ambient_sink,
        _pimpl->planner);
    
    _pimpl->task_planner = std::make_shared<rmf_task::agv::TaskPlanner>(
      task_config);
    
    _pimpl->initialized_task_planner = true;

    return _pimpl->initialized_task_planner;
  }

    return false;
}

bool FleetUpdateHandle::account_for_battery_drain(bool value)
{
  _pimpl->drain_battery = value;
  return _pimpl->drain_battery;
}
//==============================================================================
FleetUpdateHandle& FleetUpdateHandle::set_recharge_threshold(
  const double threshold)
{
  _pimpl->recharge_threshold = threshold;
  return *this;
}

//==============================================================================
FleetUpdateHandle::FleetUpdateHandle()
{
  // Do nothing
}

//==============================================================================
void dispatch_task(
    const rmf_task_msgs::msg::TaskProfile profile,
    const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets)
{
  for (auto& fleet : fleets)
  {
    auto& fimpl = FleetUpdateHandle::Implementation::get(*fleet);
    if (!fimpl.accept_task)
      continue;

    // TODO: currently the adapter supports multiple fleets. The test
    // assumption here is using a single fleet for each adapter
    rmf_task_msgs::msg::BidNotice bid;
    bid.task_profile = profile;
    fimpl.bid_notice_cb(
      std::make_shared<rmf_task_msgs::msg::BidNotice>(bid));
    
    rmf_task_msgs::msg::DispatchRequest req;
    req.task_profile = profile;
    req.fleet_name = fimpl.name;
    req.method = req.ADD;
    fimpl.dispatch_request_cb(
      std::make_shared<rmf_task_msgs::msg::DispatchRequest>(req));
  }
}

} // namespace agv
} // namespace rmf_fleet_adapter
