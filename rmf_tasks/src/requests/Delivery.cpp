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

#include <iostream>
#include <map>

#include <rmf_tasks/requests/Delivery.hpp>

namespace rmf_tasks {
namespace requests {

//==============================================================================
class Delivery::Implementation
{
public:

  Implementation()
  {}

  std::size_t _id;
  std::size_t _pickup_waypoint;
  std::size_t _dropoff_waypoint;
  std::shared_ptr<rmf_battery::MotionPowerSink> _motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> _device_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  rmf_traffic::Time _start_time;
  bool _drain_battery;

  rmf_traffic::Duration _invariant_duration;
  double _invariant_battery_drain;
};

//==============================================================================
rmf_tasks::Request::SharedPtr Delivery::make(
  std::size_t id,
  std::size_t pickup_waypoint,
  std::size_t dropoff_waypoint,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  rmf_traffic::Time start_time,
  bool drain_battery)
{
  std::shared_ptr<Delivery> delivery(new Delivery());
  delivery->_pimpl->_id = id;
  delivery->_pimpl->_pickup_waypoint = pickup_waypoint;
  delivery->_pimpl->_dropoff_waypoint = dropoff_waypoint;
  delivery->_pimpl->_motion_sink = std::move(motion_sink);
  delivery->_pimpl->_device_sink = std::move(device_sink);
  delivery->_pimpl->_planner = std::move(planner);
  delivery->_pimpl->_start_time = start_time;
  delivery->_pimpl->_drain_battery = drain_battery;

  // Calculate duration of invariant component of task
  const auto plan_start_time = std::chrono::steady_clock::now();
  rmf_traffic::agv::Planner::Start start{
    plan_start_time,
    delivery->_pimpl->_pickup_waypoint,
    0.0};

  rmf_traffic::agv::Planner::Goal goal{delivery->_pimpl->_dropoff_waypoint};
  const auto result_to_dropoff = delivery->_pimpl->_planner->plan(start, goal);

  const auto trajectory = result_to_dropoff->get_itinerary().back().trajectory();
  const auto& finish_time = *trajectory.finish_time();
  
  delivery->_pimpl->_invariant_duration = finish_time - plan_start_time;
  delivery->_pimpl->_invariant_battery_drain = 0.0;

  if (delivery->_pimpl->_drain_battery)
  {
    // Compute battery drain
    const double dSOC_motion =
      delivery->_pimpl->_motion_sink->compute_change_in_charge(trajectory);
    const double dSOC_device =
      delivery->_pimpl->_device_sink->compute_change_in_charge(
        rmf_traffic::time::to_seconds(delivery->_pimpl->_invariant_duration));
    delivery->_pimpl->_invariant_battery_drain = dSOC_motion + dSOC_device;  
  }

  return delivery;
}

//==============================================================================
Delivery::Delivery()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
std::size_t Delivery::id() const
{
  return _pimpl->_id;
}

//==============================================================================
rmf_utils::optional<rmf_tasks::Estimate> Delivery::estimate_finish(
  rmf_traffic::Time relative_start_time,
  const agv::State& initial_state,
  const agv::StateConfig& state_config) const
{
  agv::State state(
    _pimpl->_dropoff_waypoint, 
    initial_state.charging_waypoint(),
    initial_state.finish_duration(),
    initial_state.battery_soc());

  rmf_traffic::Duration variant_duration(0);

  const rmf_traffic::Time start_time =
    relative_start_time + initial_state.finish_duration();

  double battery_soc = initial_state.battery_soc();
  double dSOC_motion = 0.0;
  double dSOC_device = 0.0;

  if (initial_state.waypoint() != _pimpl->_pickup_waypoint)
  {
    // Compute plan to pickup waypoint along with battery drain
    rmf_traffic::agv::Planner::Start start{
      start_time,
      initial_state.waypoint(),
      0.0};

    rmf_traffic::agv::Planner::Goal goal{_pimpl->_pickup_waypoint};

    const auto result_to_pickup = _pimpl->_planner->plan(start, goal);
    // We assume we can always compute a plan
    const auto& trajectory =
      result_to_pickup->get_itinerary().back().trajectory();
    const auto& finish_time = *trajectory.finish_time();
    variant_duration = finish_time - start_time;

    if(_pimpl->_drain_battery)
    {
      // Compute battery drain
      dSOC_motion = _pimpl->_motion_sink->compute_change_in_charge(trajectory);
      dSOC_device =
        _pimpl->_device_sink->compute_change_in_charge(
          rmf_traffic::time::to_seconds(variant_duration));
      battery_soc = battery_soc - dSOC_motion - dSOC_device;
    }

    if (battery_soc <= state_config.threshold_soc())
    {
      // std::cout << " -- Delivery: Unable to reach pickup" << std::endl;
      return rmf_utils::nullopt;
    }
  }

  const rmf_traffic::Time ideal_start = _pimpl->_start_time - variant_duration;
  const rmf_traffic::Time state_finish_time =
    relative_start_time + initial_state.finish_duration();
  const rmf_traffic::Time wait_until = state_finish_time > ideal_start ?
    state_finish_time : ideal_start;

  // Factor in invariants
  const rmf_traffic::Time new_finish_time =
    wait_until + variant_duration + _pimpl->_invariant_duration;
  state.finish_duration(new_finish_time - relative_start_time);

  if (_pimpl->_drain_battery)
  {
    battery_soc -= _pimpl->_invariant_battery_drain;
    if (battery_soc <= state_config.threshold_soc())
    {
      // std::cout << " -- Delivery: Unable to reach dropoff" << std::endl;
      return rmf_utils::nullopt;
    }

    // Check if the robot has enough charge to head back to nearest charger
    double retreat_battery_drain = 0.0;
    if ( _pimpl->_dropoff_waypoint != state.charging_waypoint())
    {
      rmf_traffic::agv::Planner::Start start{
        relative_start_time + state.finish_duration(),
        _pimpl->_dropoff_waypoint,
        0.0};

      rmf_traffic::agv::Planner::Goal goal{state.charging_waypoint()};

      const auto result_to_charger = _pimpl->_planner->plan(start, goal);
      // We assume we can always compute a plan
      const auto& trajectory =
          result_to_charger->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const rmf_traffic::Duration retreat_duration =
        finish_time - (relative_start_time + state.finish_duration());
      
      dSOC_motion = _pimpl->_motion_sink->compute_change_in_charge(trajectory);
      dSOC_device = _pimpl->_device_sink->compute_change_in_charge(
          rmf_traffic::time::to_seconds(retreat_duration));
      retreat_battery_drain = dSOC_motion + dSOC_device;
    }

    if (battery_soc - retreat_battery_drain <= state_config.threshold_soc())
      return rmf_utils::nullopt;
    
    state.battery_soc(battery_soc);
  }

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration Delivery::invariant_duration() const
{
  return _pimpl->_invariant_duration;
}

//==============================================================================
rmf_traffic::Time Delivery::earliest_start_time() const
{
  return _pimpl->_start_time;
}

//==============================================================================
} // namespace requests
} // namespace rmf_tasks
