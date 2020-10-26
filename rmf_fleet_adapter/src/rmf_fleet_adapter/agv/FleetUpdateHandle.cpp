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
  const DockSummary::SharedPtr msg)
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
  rmf_task::RequestPtr new_request = nullptr;
  const auto& task_profile = msg->task_profile;
  const auto& task_type = task_profile.type;
  const rmf_traffic::Time start_time = rmf_traffic_ros2::convert(task_profile.start_time);
  // TODO (YV) get rid of ID field in RequestPtr
  std::string id = msg->task_profile.task_id;
  const auto& graph = planner->get_configuration().graph();

  // RCLCPP_INFO(
  //   node->get_logger(),
  //   "Fleet [%s] is processing BidNotice with task_id:[%s] and type:[%d]...",
  //   name.c_str(), id.c_str(), task_type.value);

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

    RCLCPP_INFO(
      node->get_logger(),
      "Generated Clean request");
  }

  else if (task_type.value == rmf_task_msgs::msg::TaskType::DELIVERY_TASK)
  {
    // TODO(YV)
  }
  else if (task_type.value == rmf_task_msgs::msg::TaskType::LOOP_TASK)
  {
    // TODO(YV)
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

  // Update robot states and combine new requestptr with requestptr of
  // non-charging tasks in task manager queues
  std::vector<rmf_task::agv::State> states;
  std::vector<rmf_task::agv::StateConfig> state_configs;
  std::vector<rmf_task::RequestPtr> pending_requests;
  pending_requests.push_back(new_request);
  for (const auto& t : task_managers)
  {
    states.push_back(t.first->state());
    state_configs.push_back(t.first->state_config());
    const auto requests = t.second->requests();
    pending_requests.insert(pending_requests.end(), requests.begin(), requests.end());
  }

  RCLCPP_INFO(
    node->get_logger(), 
    "Planning for [%d] robot and [%d] request(s)", 
    states.size(), pending_requests.size());

  // Generate new task assignments while accommodating for the new
  // request
  // Call greedy_plan but run optimal_plan() in a separate thread
  const auto assignments = task_planner->optimal_plan(
    rmf_traffic_ros2::convert(node->now()),
    states,
    state_configs,
    pending_requests,
    nullptr);

  const double cost = task_planner->compute_cost(assignments);

  // Publish BidProposal
  rmf_task_msgs::msg::BidProposal bid_proposal;
  bid_proposal.fleet_name = name;
  bid_proposal.task_profile = task_profile;
  bid_proposal.prev_cost = current_assignment_cost;
  bid_proposal.new_cost = cost;
  // TODO populate finish_time and robot_name

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
// auto FleetUpdateHandle::Implementation::estimate_delivery(
//     const rmf_task_msgs::msg::Delivery& request) const
// -> rmf_utils::optional<FleetUpdateHandle::Implementation::DeliveryEstimate>
// {
//   const auto& graph = planner->get_configuration().graph();
//   const auto pickup_wp = graph.find_waypoint(request.pickup_place_name);
//   if (!pickup_wp)
//     return rmf_utils::nullopt;

//   const auto dropoff_wp = graph.find_waypoint(request.dropoff_place_name);
//   if (!dropoff_wp)
//     return rmf_utils::nullopt;

//   const auto pickup_goal = rmf_traffic::agv::Plan::Goal(pickup_wp->index());
//   const auto dropoff_goal = rmf_traffic::agv::Plan::Goal(dropoff_wp->index());

//   // TODO(MXG): At some point we should consider parallelizing this estimation
//   // process and taking the existing schedule into account, but for now we'll
//   // try to use a very quick rough estimate.
//   DeliveryEstimate best;
//   for (const auto& element : task_managers)
//   {
//     const auto& mgr = *element.second;
//     auto start = mgr.expected_finish_location();
//     const auto pickup_plan = planner->plan(start, pickup_goal);
//     if (!pickup_plan)
//       continue;

//     const auto& pickup_plan_end = pickup_plan->get_waypoints().back();
//     assert(pickup_plan_end.graph_index());
//     const auto dropoff_start = rmf_traffic::agv::Plan::Start(
//           pickup_plan_end.time(),
//           *pickup_plan_end.graph_index(),
//           pickup_plan_end.position()[2]);

//     const auto dropoff_plan = planner->plan(dropoff_start, dropoff_goal);
//     if (!dropoff_plan)
//       continue;

//     const auto& final_wp = dropoff_plan->get_waypoints().back();

//     const auto estimate = final_wp.time();
//     rmf_traffic::agv::Plan::Start finish{
//       estimate,
//       *final_wp.graph_index(),
//       final_wp.position()[2]
//     };

//     if (estimate < best.time)
//     {
//       best = DeliveryEstimate{
//         estimate,
//         element.first,
//         std::move(start.front()),
//         std::move(dropoff_start),
//         std::move(finish)
//       };
//     }
//   }

//   if (best.robot)
//     return best;

//   return rmf_utils::nullopt;
// }

//==============================================================================
// void FleetUpdateHandle::Implementation::perform_delivery(
//     const rmf_task_msgs::msg::Delivery& request,
//     const DeliveryEstimate& estimate)
// {
//   auto& mgr = *task_managers.at(estimate.robot);
//   mgr.queue_task(
//         tasks::make_delivery(
//           request,
//           estimate.robot,
//           *estimate.pickup_start,
//           *estimate.dropoff_start),
//         *estimate.finish);
// }

//==============================================================================
// auto FleetUpdateHandle::Implementation::estimate_loop(
//     const rmf_task_msgs::msg::Loop& request) const
// -> rmf_utils::optional<LoopEstimate>
// {
//   if (request.robot_type != name)
//     return rmf_utils::nullopt;

//   const std::size_t n = request.num_loops;
//   if (n == 0)
//     return rmf_utils::nullopt;

//   const auto& graph = planner->get_configuration().graph();
//   const auto loop_start_wp = graph.find_waypoint(request.start_name);
//   if (!loop_start_wp)
//     return rmf_utils::nullopt;

//   const auto loop_end_wp = graph.find_waypoint(request.finish_name);
//   if (!loop_end_wp)
//     return rmf_utils::nullopt;

//   const auto loop_start_goal =
//       rmf_traffic::agv::Plan::Goal(loop_start_wp->index());

//   const auto loop_end_goal =
//       rmf_traffic::agv::Plan::Goal(loop_end_wp->index());

//   LoopEstimate best;
//   for (const auto& element : task_managers)
//   {
//     LoopEstimate estimate;
//     estimate.robot = element.first;

//     const auto& mgr = *element.second;
//     auto start = mgr.expected_finish_location();
//     const auto loop_init_plan = planner->plan(start, loop_start_goal);
//     if (!loop_init_plan)
//       continue;

//     rmf_traffic::Duration init_duration = std::chrono::seconds(0);
//     if (loop_init_plan->get_waypoints().size() > 1)
//     {
//       // If loop_init_plan is not empty, then that means we are not starting at
//       // the starting point of the loop. Therefore we will need an initial plan
//       // to reach the first point in the loop.
//       estimate.init_start = start.front();

//       init_duration =
//           loop_init_plan->get_waypoints().back().time()
//           - loop_init_plan->get_waypoints().front().time();
//     }

//     const auto loop_forward_start = [&]() -> rmf_traffic::agv::Plan::StartSet
//     {
//       if (loop_init_plan->get_waypoints().empty())
//         return start;

//       const auto& loop_init_wp = loop_init_plan->get_waypoints().back();
//       assert(loop_init_wp.graph_index());
//       return {rmf_traffic::agv::Plan::Start(
//             loop_init_wp.time(),
//             *loop_init_wp.graph_index(),
//             loop_init_wp.position()[2])};
//     }();

//     const auto loop_forward_plan =
//         planner->plan(loop_forward_start, loop_end_goal);
//     if (!loop_forward_plan)
//       continue;

//     // If the forward plan is empty then that means the start and end of the
//     // loop are the same, making it a useless request.
//     // TODO(MXG): We should probably make noise here instead of just ignoring
//     // the request.
//     if (loop_forward_plan->get_waypoints().empty())
//       return rmf_utils::nullopt;

//     estimate.loop_start = loop_forward_start.front();

//     const auto loop_duration =
//         loop_forward_plan->get_waypoints().back().time()
//         - loop_forward_plan->get_waypoints().front().time();

//     // We only need to provide this if there is supposed to be more than one
//     // loop.
//     const auto& final_wp = loop_forward_plan->get_waypoints().back();
//     assert(final_wp.graph_index());
//     estimate.loop_end = rmf_traffic::agv::Plan::Start{
//       final_wp.time(),
//       *final_wp.graph_index(),
//       final_wp.position()[2]
//     };

//     const auto start_time = [&]()
//     {
//       if (loop_init_plan->get_waypoints().empty())
//         return loop_forward_plan->get_waypoints().front().time();

//       return loop_init_plan->get_waypoints().front().time();
//     }();

//     estimate.time =
//         start_time + init_duration + (2*n - 1)*loop_duration;

//     estimate.loop_end->time(estimate.time);

//     if (estimate.time < best.time)
//       best = std::move(estimate);
//   }

//   if (best.robot)
//     return best;

//   return rmf_utils::nullopt;
// }

//==============================================================================
// void FleetUpdateHandle::Implementation::perform_loop(
//     const rmf_task_msgs::msg::Loop& request,
//     const LoopEstimate& estimate)
// {
//   auto& mgr = task_managers.at(estimate.robot);
//   mgr->queue_task(
//         tasks::make_loop(
//           request,
//           estimate.robot,
//           estimate.init_start,
//           *estimate.loop_start,
//           estimate.loop_end),
//         *estimate.loop_end);
// }

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
  std::size_t nearest_charger;
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
            state_config
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
    std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink,
    const bool drain_battery)
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
// void request_delivery(
//     const rmf_task_msgs::msg::Delivery& request,
//     const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets)
// {
//   FleetUpdateHandle::Implementation::DeliveryEstimate best;
//   FleetUpdateHandle::Implementation* chosen_fleet = nullptr;

//   for (auto& fleet : fleets)
//   {
//     auto& fimpl = FleetUpdateHandle::Implementation::get(*fleet);
//     if (!fimpl.accept_delivery || !fimpl.accept_delivery(request))
//       continue;

//     const auto estimate = fimpl.estimate_delivery(request);
//     if (!estimate)
//       continue;

//     if (estimate->time < best.time)
//     {
//       best = *estimate;
//       chosen_fleet = &fimpl;
//     }
//   }

//   if (!chosen_fleet)
//     return;

//   chosen_fleet->perform_delivery(request, best);
// }

//==============================================================================
// void request_loop(
//     const rmf_task_msgs::msg::Loop& request,
//     const std::vector<std::shared_ptr<FleetUpdateHandle>>& fleets)
// {
//   FleetUpdateHandle::Implementation::LoopEstimate best;
//   FleetUpdateHandle::Implementation* chosen_fleet = nullptr;

//   for (auto& fleet : fleets)
//   {
//     auto& fimpl = FleetUpdateHandle::Implementation::get(*fleet);
//     const auto estimate = fimpl.estimate_loop(request);
//     if (!estimate)
//       continue;

//     if (estimate->time < best.time)
//     {
//       best = *estimate;
//       chosen_fleet = &fimpl;
//     }
//   }

//   if (!chosen_fleet)
//     return;

//   chosen_fleet->perform_loop(request, best);
// }


} // namespace agv
} // namespace rmf_fleet_adapter
