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

class PowerSystem::Implementation
{
public:
  std::string name;
  double nominal_power;
  double efficiency;
};

//==============================================================================
PowerSystem::PowerSystem(
  const std::string name,
  const double nominal_power,
  const double efficiency)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{name, nominal_power, efficiency}))
{
  // Do nothing
}

//==============================================================================
auto PowerSystem::name(std::string name) -> PowerSystem&
{
  _pimpl->name = name;
  return *this;
}

//==============================================================================
const std::string& PowerSystem::name() const
{
  return _pimpl->name;
}

//==============================================================================
auto PowerSystem::nominal_power(double nom_power) ->PowerSystem&
{
  _pimpl->nominal_power = nom_power;
  return *this;
}

//==============================================================================
double PowerSystem::nominal_power() const
{
  return _pimpl->nominal_power;
}

//==============================================================================
auto PowerSystem::efficiency(double efficiency)
->PowerSystem&
{
  _pimpl->efficiency = efficiency;
  return *this;
}

//==============================================================================
double PowerSystem::efficiency() const
{
  return _pimpl->efficiency;
}

//==============================================================================
bool PowerSystem::valid() const
{
  return !_pimpl->name.empty() && _pimpl->nominal_power > 0.0
    && _pimpl->efficiency > 0.0;
}


} // namespace agv
} // namespace rmf_battery