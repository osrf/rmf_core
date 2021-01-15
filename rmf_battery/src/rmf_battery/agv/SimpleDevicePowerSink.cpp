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

#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <cmath>

namespace rmf_battery {
namespace agv {

class SimpleDevicePowerSink::Implementation
{
public:
  BatterySystem battery_system;
  PowerSystem power_system;
};

//==============================================================================
SimpleDevicePowerSink::SimpleDevicePowerSink(
  const BatterySystem& battery_system,
  const PowerSystem& power_system)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{battery_system, power_system}))
{
  // Do nothing
}

//==============================================================================
const BatterySystem& SimpleDevicePowerSink::battery_system() const
{
  return _pimpl->battery_system;
}

//==============================================================================
const PowerSystem& SimpleDevicePowerSink::power_system() const
{
  return _pimpl->power_system;
}


//==============================================================================
double SimpleDevicePowerSink::compute_change_in_charge(
  const double run_time) const
{
  const double capacity = _pimpl->battery_system.capacity();
  const double nominal_voltage = _pimpl->battery_system.nominal_voltage();
  const double nominal_power = _pimpl->power_system.nominal_power();

  const double dE = nominal_power * run_time;
  const double dQ = dE / nominal_voltage;
  // We multiply the capacity by 3600 to convert from units of Ampere-hours to
  // Ampere-seconds
  const double dSOC = dQ / (capacity * 3600.0);

  return dSOC;
}

} // namespace agv
} // namespace rmf_battery
