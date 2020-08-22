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

using BatteryProfile = SystemTraits::BatterySystem::BatteryProfile;
using BatteryType = SystemTraits::BatterySystem::BatteryType;

class SystemTraits::PowerSystem::Implementation
{
public:
  std::string name;
  double nominal_power;
  double nominal_voltage;
  double efficiency;
};

//==============================================================================
SystemTraits::PowerSystem::PowerSystem(
  const std::string name,
  const double nominal_power,
  const double nominal_voltage,
  const double efficiency)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{std::move(name), nominal_power, nominal_voltage,
        efficiency}))
{
  // Do nothing
}

//==============================================================================
auto SystemTraits::PowerSystem::name(std::string name) -> PowerSystem&
{
  _pimpl->name = name;
  return *this;
}

//==============================================================================
std::string SystemTraits::PowerSystem::name() const
{
  return _pimpl->name;
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

//==============================================================================
bool SystemTraits::PowerSystem::valid() const
{
  return !_pimpl->name.empty() && _pimpl->nominal_power > 0.0
    && _pimpl->nominal_voltage > 0.0 && _pimpl->efficiency > 0.0;
}

//==============================================================================
class SystemTraits::BatterySystem::BatteryProfile::Implementation
{
public:
  double resistance;
  double max_voltage;
  double exp_voltage;
  double exp_capacity;
};

//==============================================================================
SystemTraits::BatterySystem::BatteryProfile::BatteryProfile(
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
auto BatteryProfile::resistance(double resistance)
-> BatteryProfile&
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
auto BatteryProfile::max_voltage(double max_voltage)
-> BatteryProfile&
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
auto BatteryProfile::exp_voltage(double exp_voltage)
-> BatteryProfile&
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
auto BatteryProfile::exp_capacity(double exp_capacity)
-> BatteryProfile&
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
class SystemTraits::BatterySystem::Implementation
{
public:
  double nominal_voltage;
  double nominal_capacity;
  double charging_current;
  BatteryType type;
  rmf_utils::optional<BatteryProfile> profile;
};

//==============================================================================
SystemTraits::BatterySystem::BatterySystem(
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
auto SystemTraits::BatterySystem::nominal_voltage(double nom_voltage)
-> BatterySystem&
{
  _pimpl->nominal_voltage = nom_voltage;
  return *this;
}

//==============================================================================
double SystemTraits::BatterySystem::nominal_voltage() const
{
  return _pimpl->nominal_voltage;
}

//==============================================================================
auto SystemTraits::BatterySystem::nominal_capacity(double nom_capacity)
-> BatterySystem&
{
  _pimpl->nominal_capacity = nom_capacity;
  return *this;
}

//==============================================================================
double SystemTraits::BatterySystem::nominal_capacity() const
{
  return _pimpl->nominal_capacity;
}

//==============================================================================
auto SystemTraits::BatterySystem::charging_current(double charging_current)
-> BatterySystem&
{
  _pimpl->charging_current = charging_current;
  return *this;
}

//==============================================================================
double SystemTraits::BatterySystem::charging_current() const
{
  return _pimpl->charging_current;
}

//==============================================================================
auto SystemTraits::BatterySystem::type(BatteryType type)
-> BatterySystem&
{
  _pimpl->type = type;
  return *this;
}

//==============================================================================
BatteryType SystemTraits::BatterySystem::type() const
{
  return _pimpl->type;
}

//==============================================================================
auto SystemTraits::BatterySystem::profile(
  rmf_utils::optional<BatterySystem::BatteryProfile> profile)
-> BatterySystem&
{
  _pimpl->profile = std::move(profile);
  return *this;
}

//==============================================================================
rmf_utils::optional<BatteryProfile> SystemTraits::BatterySystem::profile() const
{
  return _pimpl->profile;
}

//==============================================================================
bool SystemTraits::BatterySystem::valid() const
{
  bool valid = _pimpl->nominal_voltage > 0.0 &&
    _pimpl->nominal_capacity > 0.0 && _pimpl->charging_current > 0.0 &&
    (_pimpl->type == BatteryType::LeadAcid
    || _pimpl->type == BatteryType::LiIon);
  if (_pimpl->profile)
    return valid && _pimpl->profile->valid();

  return valid;

}

//==============================================================================
class SystemTraits::MechanicalSystem::Implementation
{
public:
  double mass;
  double inertia;
  double friction_coefficient;
};

//==============================================================================
SystemTraits::MechanicalSystem::MechanicalSystem(
  const double mass,
  const double inertia,
  const double friction_coefficient)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        mass,
        inertia,
        friction_coefficient,
      }))
{
  // Do nothing
}

//==============================================================================
auto SystemTraits::MechanicalSystem::mass(double mass) -> MechanicalSystem&
{
  _pimpl->mass = mass;
  return *this;
}

//==============================================================================
double SystemTraits::MechanicalSystem::mass() const
{
  return _pimpl->mass;
}

//==============================================================================
auto SystemTraits::MechanicalSystem::friction_coefficient(double friction_coeff)
-> MechanicalSystem&
{
  _pimpl->friction_coefficient = friction_coeff;
  return *this;
}

//==============================================================================
double SystemTraits::MechanicalSystem::friction_coefficient() const
{
  return _pimpl->friction_coefficient;
}

//==============================================================================
auto SystemTraits::MechanicalSystem::inertia(double inertia)
-> MechanicalSystem&
{
  _pimpl->inertia = inertia;
  return *this;
}

//==============================================================================
double SystemTraits::MechanicalSystem::inertia() const
{
  return _pimpl->inertia;
}

//==============================================================================
bool SystemTraits::MechanicalSystem::valid() const
{
  return _pimpl->mass > 0.0 && _pimpl->friction_coefficient > 0.0 &&
    _pimpl->inertia > 0.0;
}

//==============================================================================
class SystemTraits::Implementation
{
public:
  MechanicalSystem mechanical_system;
  BatterySystem battery_system;
  PowerSystems power_systems;
};

//==============================================================================
SystemTraits::SystemTraits(
  const MechanicalSystem mechanical_system,
  const BatterySystem battery_system,
  const PowerSystems power_systems)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation
      {
        std::move(mechanical_system),
        std::move(battery_system),
        std::move(power_systems)
      }))
{
  // Do nothing
}

//==============================================================================
auto SystemTraits::mechanical_system(MechanicalSystem mechanical_system)
-> SystemTraits&
{
  _pimpl->mechanical_system = std::move(mechanical_system);
  return *this;
}

//==============================================================================
const SystemTraits::MechanicalSystem SystemTraits::mechanical_system() const
{
  return _pimpl->mechanical_system;
}

//==============================================================================
auto SystemTraits::battery_system(BatterySystem battery_system)
-> SystemTraits&
{
  _pimpl->battery_system = std::move(battery_system);
  return *this;
}

//==============================================================================
const SystemTraits::BatterySystem SystemTraits::battery_system() const
{
  return _pimpl->battery_system;
}

//==============================================================================
auto SystemTraits::power_systems(PowerSystems power_systems)
-> SystemTraits&
{
  _pimpl->power_systems = std::move(power_systems);
  return *this;
}

//==============================================================================
const SystemTraits::PowerSystems SystemTraits::power_systems() const
{
  return _pimpl->power_systems;
}

//==============================================================================
bool SystemTraits::valid() const
{
  bool valid = true;
  for (const auto& power_system : _pimpl->power_systems)
    valid = valid && power_system.second.valid();
  return _pimpl->battery_system.valid() && _pimpl->mechanical_system.valid() &&
    valid;
}

} // namespace agv
} // namespace rmf_battery