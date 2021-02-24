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

#include <map>

#include <rmf_task/requests/Delivery.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class DeliveryDescription::Implementation
{
public:

  Implementation()
  {}

  std::size_t pickup_waypoint;
  std::string pickup_dispenser;
  std::size_t dropoff_waypoint;
  std::string dropoff_ingestor;
  std::vector<DispenserRequestItem> items;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> device_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  rmf_traffic::Time start_time;
  bool drain_battery;

  rmf_traffic::Duration invariant_duration;
  double invariant_battery_drain;
};

//==============================================================================
rmf_task::DescriptionPtr DeliveryDescription::make(
  std::size_t pickup_waypoint,
  std::string pickup_dispenser,
  std::size_t dropoff_waypoint,
  std::string dropoff_ingestor,
  std::vector<DispenserRequestItem> items,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  rmf_traffic::Time start_time,
  bool drain_battery)
{
  std::shared_ptr<DeliveryDescription> delivery(new DeliveryDescription());
  delivery->_pimpl->pickup_waypoint = pickup_waypoint;
  delivery->_pimpl->pickup_dispenser = std::move(pickup_dispenser);
  delivery->_pimpl->dropoff_waypoint = dropoff_waypoint;
  delivery->_pimpl->dropoff_ingestor = std::move(dropoff_ingestor);
  delivery->_pimpl->items = std::move(items);
  delivery->_pimpl->motion_sink = std::move(motion_sink);
  delivery->_pimpl->device_sink = std::move(device_sink);
  delivery->_pimpl->planner = std::move(planner);
  delivery->_pimpl->start_time = start_time;
  delivery->_pimpl->drain_battery = drain_battery;

  // Calculate duration of invariant component of task
  delivery->_pimpl->invariant_duration = rmf_traffic::Duration{0};
  delivery->_pimpl->invariant_battery_drain = 0.0;

  if (delivery->_pimpl->pickup_waypoint != delivery->_pimpl->dropoff_waypoint)
  {
    rmf_traffic::agv::Planner::Start start{
      start_time,
      delivery->_pimpl->pickup_waypoint,
      0.0};

    rmf_traffic::agv::Planner::Goal goal{delivery->_pimpl->dropoff_waypoint};
    const auto result_to_dropoff = delivery->_pimpl->planner->plan(start, goal);

    auto itinerary_start_time = start_time;
    for (const auto& itinerary : result_to_dropoff->get_itinerary())
    {
      const auto& trajectory = itinerary.trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const auto itinerary_duration = finish_time - itinerary_start_time;

      if (delivery->_pimpl->drain_battery)
      {
        // Compute battery drain
        const double dSOC_motion =
          delivery->_pimpl->motion_sink->compute_change_in_charge(trajectory);
        const double dSOC_device =
          delivery->_pimpl->device_sink->compute_change_in_charge(
            rmf_traffic::time::to_seconds(itinerary_duration));
        delivery->_pimpl->invariant_battery_drain += dSOC_motion + dSOC_device;  
      }

      delivery->_pimpl->invariant_duration += itinerary_duration;
      itinerary_start_time = finish_time;
    }
  }

  return delivery;
}

//==============================================================================
DeliveryDescription::DeliveryDescription()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
rmf_utils::optional<rmf_task::Estimate> DeliveryDescription::estimate_finish(
  const agv::State& initial_state,
  const agv::Constraints& task_planning_constraints,
  const std::shared_ptr<EstimateCache> estimate_cache) const
{
  rmf_traffic::agv::Plan::Start final_plan_start{
    initial_state.finish_time(),
    _pimpl->dropoff_waypoint,
    initial_state.location().orientation()};
  agv::State state{
    std::move(final_plan_start),
    initial_state.charging_waypoint(),
    initial_state.battery_soc()};

  rmf_traffic::Duration variant_duration(0);

  const rmf_traffic::Time start_time = initial_state.finish_time();
  double battery_soc = initial_state.battery_soc();
  double dSOC_motion = 0.0;
  double dSOC_device = 0.0;

  // Factor in battery drain while moving to start waypoint of task
  if (initial_state.waypoint() != _pimpl->pickup_waypoint)
  {
    const auto endpoints = std::make_pair(initial_state.waypoint(),
      _pimpl->pickup_waypoint);
    const auto& cache_result = estimate_cache->get(endpoints);
    // Use previously memoized values if possible
    if (cache_result)
    {
      variant_duration = cache_result->duration;
      if (_pimpl->drain_battery)
        battery_soc = battery_soc - cache_result->dsoc;
    }
    else
    {
      // Compute plan to pickup waypoint along with battery drain
      rmf_traffic::agv::Planner::Goal goal{endpoints.second};
      const auto result_to_pickup = _pimpl->planner->plan(
        initial_state.location(), goal);
      // We assume we can always compute a plan
      auto itinerary_start_time = start_time;
      double variant_battery_drain = 0.0;
      for (const auto& itinerary : result_to_pickup->get_itinerary())
      {  
        const auto& trajectory = itinerary.trajectory();
        const auto& finish_time = *trajectory.finish_time();
        const rmf_traffic::Duration itinerary_duration =
          finish_time - itinerary_start_time;

        if (_pimpl->drain_battery)
        {
          // Compute battery drain
          dSOC_motion = _pimpl->motion_sink->compute_change_in_charge(
            trajectory);
          dSOC_device =
            _pimpl->device_sink->compute_change_in_charge(
              rmf_traffic::time::to_seconds(itinerary_duration));
          battery_soc = battery_soc - dSOC_motion - dSOC_device;
          variant_battery_drain += dSOC_device + dSOC_motion;
        }
        itinerary_start_time = finish_time;
        variant_duration += itinerary_duration;
      }
      estimate_cache->set(endpoints, variant_duration,
        variant_battery_drain);
    }

    if (battery_soc <= task_planning_constraints.threshold_soc())
      return rmf_utils::nullopt;
  }

  const rmf_traffic::Time ideal_start = _pimpl->start_time - variant_duration;
  const rmf_traffic::Time wait_until =
    initial_state.finish_time() > ideal_start ?
    initial_state.finish_time() : ideal_start;

  // Factor in battery drain while waiting to move to start waypoint. If a robot
  // is initially at a charging waypoint, it is assumed to be continually charging
  if (_pimpl->drain_battery && wait_until > initial_state.finish_time() &&
    initial_state.waypoint() != initial_state.charging_waypoint())
  {
    rmf_traffic::Duration wait_duration(wait_until - initial_state.finish_time());
    dSOC_device = _pimpl->device_sink->compute_change_in_charge(
      rmf_traffic::time::to_seconds(wait_duration));
    battery_soc = battery_soc - dSOC_device;

    if (battery_soc <= task_planning_constraints.threshold_soc())
    {
      return rmf_utils::nullopt;
    }
  }

  // Factor in invariants
  state.finish_time(
    wait_until + variant_duration + _pimpl->invariant_duration);

  if (_pimpl->drain_battery)
  {
    battery_soc -= _pimpl->invariant_battery_drain;
    if (battery_soc <= task_planning_constraints.threshold_soc())
      return rmf_utils::nullopt;

    // Check if the robot has enough charge to head back to nearest charger
    double retreat_battery_drain = 0.0;
    if ( _pimpl->dropoff_waypoint != state.charging_waypoint())
    {
      const auto endpoints = std::make_pair(_pimpl->dropoff_waypoint,
        state.charging_waypoint());
      const auto& cache_result = estimate_cache->get(endpoints);
      if (cache_result)
      {
        retreat_battery_drain = cache_result->dsoc;
      }
      else
      {
        rmf_traffic::agv::Planner::Start start{
          state.finish_time(),
          endpoints.first,
          0.0};

        rmf_traffic::agv::Planner::Goal goal{endpoints.second};

        const auto result_to_charger = _pimpl->planner->plan(start, goal);
        // We assume we can always compute a plan
        auto itinerary_start_time = state.finish_time();
        rmf_traffic::Duration retreat_duration(0);
        for (const auto& itinerary : result_to_charger->get_itinerary())
        {
          const auto& trajectory = itinerary.trajectory();
          const auto& finish_time = *trajectory.finish_time();
          const rmf_traffic::Duration itinerary_duration =
            finish_time - itinerary_start_time;

          dSOC_motion = _pimpl->motion_sink->compute_change_in_charge(trajectory);
          dSOC_device = _pimpl->device_sink->compute_change_in_charge(
              rmf_traffic::time::to_seconds(itinerary_duration));
          retreat_battery_drain += dSOC_motion + dSOC_device;

          itinerary_start_time = finish_time;
          retreat_duration += itinerary_duration;
        }
        estimate_cache->set(endpoints, retreat_duration,
          retreat_battery_drain);
      }
    }

    if (battery_soc - retreat_battery_drain <= task_planning_constraints.threshold_soc())
      return rmf_utils::nullopt;
    
    state.battery_soc(battery_soc);
  }

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration DeliveryDescription::invariant_duration() const
{
  return _pimpl->invariant_duration;
}

//==============================================================================
std::size_t DeliveryDescription::pickup_waypoint() const
{
  return _pimpl->pickup_waypoint;
}

//==============================================================================
const std::string& DeliveryDescription::pickup_dispenser() const
{
  return _pimpl->pickup_dispenser;
}

//==============================================================================
const std::string& DeliveryDescription::dropoff_ingestor() const
{
  return _pimpl->dropoff_ingestor;
}

//==============================================================================
std::size_t DeliveryDescription::dropoff_waypoint() const
{
  return _pimpl->dropoff_waypoint;
}

//==============================================================================
const std::vector<DeliveryDescription::DispenserRequestItem>&
DeliveryDescription::items() const
{
  return _pimpl->items;
}

//==============================================================================
DeliveryDescription::Start DeliveryDescription::dropoff_start(
  const DeliveryDescription::Start& start) const
{
  if (start.waypoint() == _pimpl->pickup_waypoint)
    return start;

  rmf_traffic::agv::Planner::Goal goal{_pimpl->pickup_waypoint};

  const auto result = _pimpl->planner->plan(start, goal);
  // We assume we can always compute a plan
  const auto& trajectory =
      result->get_itinerary().back().trajectory();
  const auto& finish_time = *trajectory.finish_time();
  const double orientation = trajectory.back().position()[2];

  rmf_traffic::agv::Planner::Start dropoff_start{
    finish_time,
    _pimpl->pickup_waypoint,
    orientation};

  return dropoff_start;

}

//==============================================================================
ConstRequestPtr Delivery::make(
  const std::string& id,
  std::size_t pickup_waypoint,
  std::string pickup_dispenser,
  std::size_t dropoff_waypoint,
  std::string dropoff_ingestor,
  std::vector<DispenserRequestItem> items,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  rmf_traffic::Time start_time,
  bool drain_battery,
  ConstPriorityPtr priority)
{
  const auto description = DeliveryDescription::make(
    pickup_waypoint,
    pickup_dispenser,
    dropoff_waypoint,
    dropoff_ingestor,
    items,
    motion_sink,
    device_sink,
    planner,
    start_time,
    drain_battery);

  return std::make_shared<Request>(id, start_time, priority, description);

}


} // namespace requests
} // namespace rmf_task
