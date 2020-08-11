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

#include <rmf_battery/agv/SystemTraits.hpp>

namespace rmf_battery {
namespace agv {

class SystemTraits::PowerSystem::Implementation
{
public:

  double nominal_power;
  double nominal_voltage;
  double efficiency;
};

//==============================================================================
SystemTraits::PowerSystem::PowerSystem(
  const double nominal_power,
  const double nominal_voltage,
  const double efficiency)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{nominal_power, nominal_voltage, efficiency}))
{
  // Do nothing
}

//==============================================================================
auto SystemTraits::PowerSystem::nominal_power(double nom_power) ->PowerSystem&
{
  _pimpl->nominal_power = nom_power;
  return *this;
}

//==============================================================================
double SystemTraits::PowerSystem::nominal_power() const
{
  return _pimpl->nominal_power;
}

//==============================================================================
auto SystemTraits::PowerSystem::nominal_voltage(double nom_voltage)
->PowerSystem&
{
  _pimpl->nominal_voltage = nom_voltage;
  return *this;
}

//==============================================================================
double SystemTraits::PowerSystem::nominal_voltage() const
{
  return _pimpl->nominal_voltage;
}
//==============================================================================
auto SystemTraits::PowerSystem::efficiency(double efficiency)
->PowerSystem&
{
  _pimpl->efficiency = efficiency;
  return *this;
}

//==============================================================================
double SystemTraits::PowerSystem::efficiency() const
{
  return _pimpl->efficiency;
}

} // namespace agv
} // namespace rmf_battery