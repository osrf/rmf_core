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

using BatteryProfile = BatterySystem::BatteryProfile;
using BatteryType = BatterySystem::BatteryType;

class BatterySystem::BatteryProfile::Implementation
{
public:
  double resistance;
  double max_voltage;
  double exp_voltage;
  double exp_capacity;
};

//==============================================================================
BatterySystem::BatteryProfile::BatteryProfile(
  const double resistance,
  const double max_voltage,
  const double exp_voltage,
  const double exp_capacity)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{resistance, max_voltage, exp_voltage, exp_capacity}))
{
  // Do nothing
}

//==============================================================================
auto BatteryProfile::resistance(double resistance) -> BatteryProfile&
{
  _pimpl->resistance = resistance;
  return *this;
}

//==============================================================================
double BatteryProfile::resistance() const
{
  return _pimpl->resistance;
}

//==============================================================================
auto BatteryProfile::max_voltage(double max_voltage) -> BatteryProfile&
{
  _pimpl->max_voltage = max_voltage;
  return *this;
}

//==============================================================================
double BatteryProfile::max_voltage() const
{
  return _pimpl->max_voltage;
}

//==============================================================================
auto BatteryProfile::exp_voltage(double exp_voltage) -> BatteryProfile&
{
  _pimpl->exp_voltage = exp_voltage;
  return *this;
}

//==============================================================================
double BatteryProfile::exp_voltage() const
{
  return _pimpl->exp_voltage;
}

//==============================================================================
auto BatteryProfile::exp_capacity(double exp_capacity) -> BatteryProfile&
{
  _pimpl->exp_capacity = exp_capacity;
  return *this;
}

//==============================================================================
double BatteryProfile::exp_capacity() const
{
  return _pimpl->exp_capacity;
}

//==============================================================================
bool BatteryProfile::valid() const
{
  return _pimpl->resistance > 0.0 && _pimpl->max_voltage > 0.0 &&
    _pimpl->exp_voltage > 0.0 && _pimpl->exp_capacity > 0.0;
}

//==============================================================================
class BatterySystem::Implementation
{
public:
  double nominal_voltage;
  double nominal_capacity;
  double charging_current;
  BatteryType type;
  rmf_utils::optional<BatteryProfile> profile;
};

//==============================================================================
BatterySystem::BatterySystem(
  const double nominal_voltage,
  const double nominal_capacity,
  const double charging_current,
  BatteryType type,
  rmf_utils::optional<BatteryProfile> profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        nominal_voltage,
        nominal_capacity,
        charging_current,
        type,
        std::move(profile)
      }))
{
  // Do nothing
}

//==============================================================================
auto BatterySystem::nominal_voltage(double nom_voltage) -> BatterySystem&
{
  _pimpl->nominal_voltage = nom_voltage;
  return *this;
}

//==============================================================================
double BatterySystem::nominal_voltage() const
{
  return _pimpl->nominal_voltage;
}

//==============================================================================
auto BatterySystem::nominal_capacity(double nom_capacity) -> BatterySystem&
{
  _pimpl->nominal_capacity = nom_capacity;
  return *this;
}

//==============================================================================
double BatterySystem::nominal_capacity() const
{
  return _pimpl->nominal_capacity;
}

//==============================================================================
auto BatterySystem::charging_current(double charging_current) -> BatterySystem&
{
  _pimpl->charging_current = charging_current;
  return *this;
}

//==============================================================================
double BatterySystem::charging_current() const
{
  return _pimpl->charging_current;
}

//==============================================================================
auto BatterySystem::type(BatteryType type)
-> BatterySystem&
{
  _pimpl->type = type;
  return *this;
}

//==============================================================================
BatteryType BatterySystem::type() const
{
  return _pimpl->type;
}

//==============================================================================
auto BatterySystem::profile(
  rmf_utils::optional<BatterySystem::BatteryProfile> profile) -> BatterySystem&
{
  _pimpl->profile = std::move(profile);
  return *this;
}

//==============================================================================
rmf_utils::optional<BatteryProfile> BatterySystem::profile() const
{
  return _pimpl->profile;
}

//==============================================================================
bool BatterySystem::valid() const
{
  bool valid = _pimpl->nominal_voltage > 0.0 &&
    _pimpl->nominal_capacity > 0.0 && _pimpl->charging_current > 0.0 &&
    (_pimpl->type == BatteryType::LeadAcid
    || _pimpl->type == BatteryType::LiIon);
  if (_pimpl->profile)
    return valid && _pimpl->profile->valid();

  return valid;

}

} // namespace agv
} // namespace rmf_battery