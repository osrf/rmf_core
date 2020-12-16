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

#include <rmf_battery/agv/BatterySystem.hpp>

namespace rmf_battery {
namespace agv {

//==============================================================================
class BatterySystem::Implementation
{
public:
  double nominal_voltage;
  double capacity;
  double charging_current;
};

//==============================================================================
std::optional<BatterySystem> BatterySystem::make(
  double nominal_voltage,
  double capacity,
  double charging_current)
{
  if (nominal_voltage <= 0.0 ||
    capacity <= 0.0 ||
    charging_current <= 0.0)
  {
    return std::nullopt;
  }

  BatterySystem battery_system;
  battery_system._pimpl->nominal_voltage = nominal_voltage;
  battery_system._pimpl->capacity = capacity;
  battery_system._pimpl->charging_current = charging_current;

  return battery_system;
}

//==============================================================================
BatterySystem::BatterySystem()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
double BatterySystem::nominal_voltage() const
{
  return _pimpl->nominal_voltage;
}

//==============================================================================
double BatterySystem::capacity() const
{
  return _pimpl->capacity;
}

//==============================================================================
double BatterySystem::charging_current() const
{
  return _pimpl->charging_current;
}

} // namespace agv
} // namespace rmf_battery