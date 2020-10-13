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

#include <cmath>

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
  double nominal_capacity;
  double discharge_current;
};

//==============================================================================
BatterySystem::BatteryProfile::BatteryProfile(
  const double resistance,
  const double max_voltage,
  const double exp_voltage,
  const double exp_capacity,
  const double nominal_capacity,
  const double discharge_current)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation
      {
        resistance,
        max_voltage,
        exp_voltage,
        exp_capacity,
        nominal_capacity,
        discharge_current
      }))
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
double BatteryProfile::nominal_capacity() const
{
  return _pimpl->nominal_capacity;
}

//==============================================================================
auto BatteryProfile::nominal_capacity(double nominal_capacity) -> BatteryProfile&
{
  _pimpl->nominal_capacity = nominal_capacity;
  return *this;
}

//==============================================================================
double BatteryProfile::exp_capacity() const
{
  return _pimpl->exp_capacity;
}

//==============================================================================
auto BatteryProfile::discharge_current(double discharge_current) -> BatteryProfile&
{
  _pimpl->discharge_current = discharge_current;
  return *this;
}

//==============================================================================
double BatteryProfile::discharge_current() const
{
  return _pimpl->discharge_current;
}

//==============================================================================
bool BatteryProfile::valid() const
{
  return _pimpl->resistance > 0.0 && _pimpl->max_voltage > 0.0 &&
    _pimpl->exp_voltage > 0.0 && _pimpl->exp_capacity > 0.0 &&
    _pimpl->nominal_capacity > 0.0 && _pimpl->discharge_current > 0.0;
}

//==============================================================================
class BatterySystem::Implementation
{
public:
  double nominal_voltage;
  double capacity;
  double charging_current;
  BatteryType type;
  rmf_utils::optional<BatteryProfile> profile;

  // Battery parameters used in get_voltage() to derive voltage of the battery
  // given its state of charge.
  // Ref: O. Tremblay, L. Dessaint and A. Dekkiche, "A Generic Battery Model for 
  // the Dynamic Simulation of Hybrid Electric Vehicles," 2007 IEEE Vehicle
  // Power and Propulsion Conference, Arlington, TX, 2007, pp. 284-289,
  // doi: 10.1109/VPPC.2007.4544139.
  struct ProfileParams
  {
    double a; // V
    double b; // (Ah)^-1
    double k; // V
    double e0;// V
  };

  ProfileParams params = {0.0, 0.0, 0.0, 0.0};
};

//==============================================================================
BatterySystem::BatterySystem(
  const double nominal_voltage,
  const double capacity,
  const double charging_current,
  BatteryType type,
  rmf_utils::optional<BatteryProfile> profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        nominal_voltage,
        capacity,
        charging_current,
        type,
        std::move(profile)
      }))
{
  if (_pimpl->profile.has_value())
  {
    assert(profile->valid());
    _pimpl->params.a = profile->max_voltage() - profile->exp_voltage();
    _pimpl->params.b = 3.0 / profile->exp_capacity();
    _pimpl->params.k = (profile->max_voltage() - _pimpl->nominal_voltage +
      _pimpl->params.a*(
        exp(-_pimpl->params.b*_pimpl->profile->nominal_capacity()) - 1)*(
          _pimpl->capacity - _pimpl->profile->nominal_capacity())) /
      _pimpl->profile->nominal_capacity();
    _pimpl->params.e0 = _pimpl->profile->max_voltage() + _pimpl->params.k +
      _pimpl->profile->resistance() * _pimpl->profile->discharge_current() -
      _pimpl->params.a;
  }
}

rmf_utils::optional<double> BatterySystem::estimate_voltage(const double soc) const
{
  assert(soc > 0.0 && soc <= 1.0);
  if (!_pimpl->profile.has_value() || !_pimpl->profile->valid())
    return rmf_utils::nullopt;

  return _pimpl->params.e0 - _pimpl->params.k*(1/soc) + _pimpl->params.a *
    exp(-_pimpl->params.b * _pimpl->capacity*(1 - soc));
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
auto BatterySystem::capacity(double nom_capacity) -> BatterySystem&
{
  _pimpl->capacity = nom_capacity;
  return *this;
}

//==============================================================================
double BatterySystem::capacity() const
{
  return _pimpl->capacity;
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
    _pimpl->capacity > 0.0 && _pimpl->charging_current > 0.0 &&
    (_pimpl->type == BatteryType::LeadAcid
    || _pimpl->type == BatteryType::LiIon);
  if (_pimpl->profile)
    return valid && _pimpl->profile->valid();

  return valid;

}

} // namespace agv
} // namespace rmf_battery