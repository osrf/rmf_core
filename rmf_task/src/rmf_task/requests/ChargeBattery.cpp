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

#include <string>
#include <sstream>
#include <random>

#include <rmf_task/requests/ChargeBattery.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
namespace {
std::string generate_uuid(const std::size_t length = 3)
{
  std::stringstream ss;
  for (std::size_t i = 0; i < length; ++i)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    const auto random_char = dis(gen);
    std::stringstream hexstream;
    hexstream << std::hex << random_char;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}

} // anonymous namespace

//==============================================================================
class ChargeBattery::Implementation
{
public:

  Implementation()
  {}

  // fixed id for now
  std::string id = "Charge";
  rmf_battery::agv::BatterySystemPtr battery_system;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> device_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  rmf_traffic::Time start_time;
  bool drain_battery;

  // soc to always charge the battery up to
  double charge_soc = 1.0;
  rmf_traffic::Duration invariant_duration;
};

//==============================================================================
rmf_task::ConstRequestPtr ChargeBattery::make(
  rmf_battery::agv::BatterySystem battery_system,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  rmf_traffic::Time start_time,
  bool drain_battery)
{
  std::shared_ptr<ChargeBattery> charge_battery(new ChargeBattery());
  charge_battery->_pimpl->id += generate_uuid();
  charge_battery->_pimpl->battery_system =
    rmf_battery::agv::BatterySystemPtr(new rmf_battery::agv::BatterySystem(
      battery_system.nominal_voltage(),
      battery_system.capacity(),
      battery_system.charging_current(),
      battery_system.type(),
      battery_system.profile()));
  charge_battery->_pimpl->motion_sink = std::move(motion_sink);
  charge_battery->_pimpl->device_sink = std::move(device_sink);
  charge_battery->_pimpl->planner = std::move(planner);
  charge_battery->_pimpl->start_time = start_time;
  charge_battery->_pimpl->drain_battery = drain_battery;
  charge_battery->_pimpl->invariant_duration =
    rmf_traffic::time::from_seconds(0.0);
  return charge_battery;
}

//==============================================================================
ChargeBattery::ChargeBattery()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
std::string ChargeBattery::id() const
{
  return _pimpl->id;
}

//==============================================================================
rmf_utils::optional<rmf_task::Estimate> ChargeBattery::estimate_finish(
  const agv::State& initial_state,
  const agv::StateConfig& state_config,
  const std::shared_ptr<EstimateCache> estimate_cache) const
{
  // Important to return nullopt if a charging task is not needed. In the task
  // planner, if a charging task is added, the node's latest time may be set to
  // the finishing time of the charging task, and consequently fall below the
  // segmentation threshold, causing `solve` to return. This may cause an infinite
  // loop as a new identical charging task is added in each call to `solve` before
  // returning.
  if ((abs(initial_state.battery_soc() - _pimpl->charge_soc) < 1e-3)
    && initial_state.waypoint() == initial_state.charging_waypoint())
    return rmf_utils::nullopt;

  // Compute time taken to reach charging waypoint from current location
  rmf_traffic::agv::Plan::Start final_plan_start{
    initial_state.finish_time(),
    initial_state.charging_waypoint(),
    initial_state.location().orientation()};
  agv::State state{
    std::move(final_plan_start),
    initial_state.charging_waypoint(),
    initial_state.battery_soc()};

  const auto start_time = initial_state.finish_time();

  double battery_soc = initial_state.battery_soc();
  double dSOC_motion = 0.0;
  double dSOC_device = 0.0;
  rmf_traffic::Duration variant_duration(0);

  if (initial_state.waypoint() != initial_state.charging_waypoint())
  {
    const auto endpoints = std::make_pair(initial_state.waypoint(),
      initial_state.charging_waypoint());
    const auto& cache_result = estimate_cache->get(endpoints);
    // Use memoized values if possible
    if (cache_result)
    {
      variant_duration = cache_result->duration;
      battery_soc = battery_soc - cache_result->dsoc;
    }
    else
    {
      // Compute plan to charging waypoint along with battery drain
      rmf_traffic::agv::Planner::Goal goal{endpoints.second};
      const auto result = _pimpl->planner->plan(
        initial_state.location(), goal);
      const auto& trajectory = result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      variant_duration = finish_time - start_time;

      if (_pimpl->drain_battery)
      {
        dSOC_motion = _pimpl->motion_sink->compute_change_in_charge(
          trajectory);
        dSOC_device = _pimpl->device_sink->compute_change_in_charge(
          rmf_traffic::time::to_seconds(variant_duration));
        battery_soc = battery_soc - dSOC_motion - dSOC_device;
      }

      estimate_cache->set(endpoints, variant_duration,
        dSOC_motion + dSOC_device);
    }

    // If a robot cannot reach its charging dock given its initial battery soc
    if (battery_soc <= state_config.threshold_soc())
      return rmf_utils::nullopt;
  }

  // Default _charge_soc = 1.0
  double delta_soc = _pimpl->charge_soc - battery_soc;
  assert(delta_soc >= 0.0);
  double time_to_charge =
    (3600 * delta_soc * _pimpl->battery_system->capacity()) /
    _pimpl->battery_system->charging_current();

  const rmf_traffic::Time wait_until = initial_state.finish_time();
  state.finish_time(
    wait_until + variant_duration +
    rmf_traffic::time::from_seconds(time_to_charge));

  state.battery_soc(_pimpl->charge_soc);

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration ChargeBattery::invariant_duration() const
{
  return _pimpl->invariant_duration;
}

//==============================================================================
rmf_traffic::Time ChargeBattery::earliest_start_time() const
{
  return _pimpl->start_time;
}

//==============================================================================
const rmf_battery::agv::BatterySystem& ChargeBattery::battery_system() const
{
  return *_pimpl->battery_system;
}

double ChargeBattery::max_charge_soc() const
{
  return _pimpl->charge_soc;
}

} // namespace requests
} // namespace rmf_task
