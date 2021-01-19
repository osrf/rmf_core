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

#include "ReadyToCharge.hpp"

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
auto ReadyToCharge::Active::observe() const -> const rxcpp::observable<StatusMsg>&
{
  return _status_obs;
}

//==============================================================================
rmf_traffic::Duration ReadyToCharge::Active::estimate_remaining_time() const
{
  // TODO 
  return rmf_traffic::time::from_seconds(1);
}

//==============================================================================
void ReadyToCharge::Active::emergency_alarm(const bool value)
{
  // Assume charging station is a holding point
}

//==============================================================================
void ReadyToCharge::Active::cancel()
{
  // TODO
}

//==============================================================================
const std::string& ReadyToCharge::Active::description() const
{
  return _description;
}

//==============================================================================
ReadyToCharge::Active::Active(
  agv::RobotContextPtr context)
: _context(std::move(context)),
{
  using rmf_charger_msgs::msg::ChargerState;
  _description = "Charger Negotiation";

  _status_obs = _context->node()->charger_state()
    .lift<ChargerState::SharedPtr>(on_subscribe([]()
    {

    }));
}

//==============================================================================
std::shared_ptr<Task::ActivePhase> ReadyToCharge::Pending::begin()
{
  auto active =
      std::shared_ptr<Active>(new Active(_context));
  return active;
}

//==============================================================================
rmf_traffic::Duration ReadyToCharge::Pending::estimate_phase_duration() const
{
  return rmf_traffic::Duration{0};
}

//==============================================================================
const std::string& ReadyToCharge::Pending::description() const
{
  return _description;
}

//==============================================================================
ReadyToCharge::Pending::Pending(
  agv::RobotContextPtr context)
: _context(std::move(context)),
  _battery_system(battery_system),
  _charge_to_soc(charge_to_soc),
  _time_estimate(time_estimate)
{
  _description =
    "Charging robot to [" + std::to_string(100.0 * charge_to_soc) + "%]";
}

//==============================================================================
auto WaitForCharge::make(
    agv::RobotContextPtr context,
    rmf_battery::agv::BatterySystem battery_system,
    double charge_to_soc) -> std::unique_ptr<Pending>
{

  const double capacity = battery_system.capacity();
  const double charging_current = battery_system.charging_current();
  const double time_estimate =
    3600.0 * capacity * (charge_to_soc - context->current_battery_soc()) / charging_current;
  
  return std::unique_ptr<Pending>(
        new Pending(context, battery_system, charge_to_soc, time_estimate));
}

}
};