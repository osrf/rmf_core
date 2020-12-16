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

#include <rmf_battery/agv/PowerSystem.hpp>

namespace rmf_battery {
namespace agv {

//==============================================================================
class PowerSystem::Implementation
{
public:
  double nominal_power;
};

//==============================================================================
std::optional<PowerSystem> PowerSystem::make(double nominal_power)
{
  if (nominal_power < 0.0)
    return std::nullopt;
  
  PowerSystem power_system;
  power_system._pimpl->nominal_power = nominal_power;

  return power_system;
}

//==============================================================================
PowerSystem::PowerSystem()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
double PowerSystem::nominal_power() const
{
  return _pimpl->nominal_power;
}

} // namespace agv
} // namespace rmf_battery
