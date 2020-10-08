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

#include <rmf_tasks/requests/ChargeBattery.hpp>

namespace rmf_tasks {
namespace requests {

//==============================================================================
class ChargeBattery::Implementation
{
public:

  Implementation()
  {}

  // fixed id for now
  std::size_t _id = 1001;
  rmf_battery::agv::BatterySystemPtr _battery_system;
  std::shared_ptr<rmf_battery::MotionPowerSink> _motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> _device_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  bool _drain_battery;

  // soc to always charge the battery up to
  double _charge_soc = 1.0;
  rmf_traffic::Duration _invariant_duration;
};

//==============================================================================
rmf_tasks::Request::SharedPtr ChargeBattery::make(
  rmf_battery::agv::BatterySystem battery_system,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  bool drain_battery)
{
  std::shared_ptr<ChargeBattery> charge_battery(new ChargeBattery());
  charge_battery->_pimpl->_battery_system =
    rmf_battery::agv::BatterySystemPtr(new rmf_battery::agv::BatterySystem(
      battery_system.nominal_capacity(),
      battery_system.nominal_capacity(),
      battery_system.charging_current(),
      battery_system.type(),
      battery_system.profile()));
  charge_battery->_pimpl->_motion_sink = std::move(motion_sink);
  charge_battery->_pimpl->_device_sink = std::move(device_sink);
  charge_battery->_pimpl->_planner = std::move(planner);
  charge_battery->_pimpl->_drain_battery = drain_battery;
  charge_battery->_pimpl->_invariant_duration =
    rmf_traffic::time::from_seconds(0.0);
  return charge_battery;
}

//==============================================================================
ChargeBattery::ChargeBattery()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
std::size_t ChargeBattery::id() const
{
  return _pimpl->_id;
}

//==============================================================================
rmf_utils::optional<rmf_tasks::Estimate> ChargeBattery::estimate_finish(
  rmf_traffic::Time relative_start_time,
  const agv::State& initial_state,
  const agv::StateConfig& state_config) const
{
  if (abs(initial_state.battery_soc() - _pimpl->_charge_soc) < 1e-3)
    return rmf_utils::nullopt;

  // Compute time taken to reach charging waypoint from current location
  agv::State state(
    initial_state.charging_waypoint(),
    initial_state.charging_waypoint(),
    initial_state.finish_duration(),
    initial_state.battery_soc());

  const auto start_time = relative_start_time + initial_state.finish_duration();

  double battery_soc = initial_state.battery_soc();
  rmf_traffic::Duration variant_duration(0);

  if (initial_state.waypoint() != initial_state.charging_waypoint())
  {
    // Compute plan to charging waypoint along with battery drain
    rmf_traffic::agv::Planner::Start start{
      start_time,
      initial_state.waypoint(),
      0.0};

    rmf_traffic::agv::Planner::Goal goal{initial_state.charging_waypoint()};

    const auto result = _pimpl->_planner->plan(start, goal);
    const auto& trajectory = result->get_itinerary().back().trajectory();
    const auto& finish_time = *trajectory.finish_time();
    const rmf_traffic::Duration variant_duration = finish_time - start_time;

    if (_pimpl->_drain_battery)
    {
      const double dSOC_motion = _pimpl->_motion_sink->compute_change_in_charge(
        trajectory);
      const double dSOC_device = _pimpl->_device_sink->compute_change_in_charge(
        rmf_traffic::time::to_seconds(variant_duration));
      battery_soc = battery_soc - dSOC_motion - dSOC_device;
    }

    // If a robot cannot reach its charging dock given its initial battery soc
    if (battery_soc <= state_config.threshold_soc())
      return rmf_utils::nullopt;
  }

  // Default _charge_soc = 1.0
  double delta_soc = _pimpl->_charge_soc - battery_soc;
  assert(delta_soc >= 0.0);
  double time_to_charge =
    (3600 * delta_soc * _pimpl->_battery_system->nominal_capacity()) /
    _pimpl->_battery_system->charging_current();

  const rmf_traffic::Time wait_until =
    relative_start_time + initial_state.finish_duration();

  const rmf_traffic::Time new_finish_time =
    wait_until + variant_duration +
    rmf_traffic::time::from_seconds(time_to_charge);
  const rmf_traffic::Duration new_finish_duration =
    new_finish_time - relative_start_time;
  state.finish_duration(new_finish_duration);
  state.battery_soc(_pimpl->_charge_soc);

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration ChargeBattery::invariant_duration() const
{
  return _pimpl->_invariant_duration;
}

//==============================================================================
rmf_traffic::Time ChargeBattery::earliest_start_time() const
{
  return rmf_traffic::Time::min();
}

//==============================================================================
} // namespace requests
} // namespace rmf_tasks
