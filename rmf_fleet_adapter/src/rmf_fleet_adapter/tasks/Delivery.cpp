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

#include "../phases/DispenseItem.hpp"
#include "../phases/IngestItem.hpp"
#include "../phases/GoToPlace.hpp"

#include "../agv/internal_FleetUpdateHandle.hpp"

#include "Delivery.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
rmf_utils::optional<DeliveryEstimate> estimate_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const std::shared_ptr<agv::FleetUpdateHandle>& fleet)
{
  const auto& fimpl = agv::FleetUpdateHandle::Implementation::get(*fleet);
  const auto planner = fimpl.planner;
  const auto& graph = planner->get_configuration().graph();

  const auto pickup_wp = graph.find_waypoint(request.pickup_place_name);
  if (!pickup_wp)
    return rmf_utils::nullopt;

  const auto dropoff_wp = graph.find_waypoint(request.dropoff_place_name);
  if (!dropoff_wp)
    return rmf_utils::nullopt;

  const auto pickup_goal = rmf_traffic::agv::Plan::Goal(pickup_wp->index());
  const auto dropoff_goal = rmf_traffic::agv::Plan::Goal(dropoff_wp->index());

  // TODO(MXG): At some point we should consider parallelizing this estimation
  // process and taking the existing schedule into account, but for now we'll
  // try to use a very quick rough estimate.
  DeliveryEstimate best;
  for (const auto& element : fimpl.task_managers)
  {
    const auto& mgr = *element.second;
    auto start = mgr.expected_finish_location();
    const auto pickup_plan = planner->plan(start, pickup_goal);
    if (!pickup_plan)
      continue;

    const auto& pickup_plan_end = pickup_plan->get_waypoints().back();
    assert(pickup_plan_end.graph_index());
    const auto dropoff_start = rmf_traffic::agv::Plan::Start(
          pickup_plan_end.time(),
          *pickup_plan_end.graph_index(),
          pickup_plan_end.position()[2]);

    const auto dropoff_plan = planner->plan(dropoff_start, dropoff_goal);
    if (!dropoff_plan)
      continue;

    const auto& final_wp = dropoff_plan->get_waypoints().back();

    const auto estimate = final_wp.time();
    rmf_traffic::agv::Plan::Start finish{
      estimate,
      *final_wp.graph_index(),
      final_wp.position()[2]
    };

    if (estimate < best.time)
    {
      best = DeliveryEstimate{
        estimate,
        element.first,
        std::move(start.front()),
        std::move(dropoff_start),
        std::move(finish)
      };
    }
  }

  if (best.robot)
    return best;

  return rmf_utils::nullopt;
}

//==============================================================================
std::shared_ptr<Task> make_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const agv::RobotContextPtr& context,
    rmf_traffic::agv::Plan::Start pickup_start,
    rmf_traffic::agv::Plan::Start dropoff_start)
{
  const auto& graph = context->navigation_graph();

  const auto pickup_wp =
      graph.find_waypoint(request.pickup_place_name)->index();

  Task::PendingPhases phases;
  phases.push_back(
        phases::GoToPlace::make(context, std::move(pickup_start), pickup_wp));

  phases.push_back(
        std::make_unique<phases::DispenseItem::PendingPhase>(
          context,
          request.task_id,
          request.pickup_dispenser,
          context->itinerary().description().owner(),
          request.items));

  const auto dropoff_wp =
      graph.find_waypoint(request.dropoff_place_name)->index();

  phases.push_back(
        phases::GoToPlace::make(context, std::move(dropoff_start), dropoff_wp));

  std::vector<rmf_ingestor_msgs::msg::IngestorRequestItem> ingestor_items;
  ingestor_items.reserve(request.items.size());
  for(auto& dispenser_item : request.items){
    rmf_ingestor_msgs::msg::IngestorRequestItem item{};
    item.type_guid = dispenser_item.type_guid;
    item.quantity = dispenser_item.quantity;
    item.compartment_name = dispenser_item.compartment_name;
    ingestor_items.push_back(std::move(item));
  }

  phases.push_back(
        std::make_unique<phases::IngestItem::PendingPhase>(
          context,
          request.task_id,
          request.dropoff_ingestor,
          context->itinerary().description().owner(),
          ingestor_items));

  return Task::make(request.task_id, std::move(phases), context->worker());
}

} // namespace task
} // namespace rmf_fleet_adapter
