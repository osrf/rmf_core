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

#include <rmf_battery/agv/MechanicalSystem.hpp>

namespace rmf_battery {
namespace agv {

//==============================================================================
class MechanicalSystem::Implementation
{
public:
  double mass;
  double inertia;
  double friction_coefficient;
};

//==============================================================================
MechanicalSystem::MechanicalSystem(
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
auto MechanicalSystem::mass(double mass) -> MechanicalSystem&
{
  _pimpl->mass = mass;
  return *this;
}

//==============================================================================
double MechanicalSystem::mass() const
{
  return _pimpl->mass;
}

//==============================================================================
auto MechanicalSystem::friction_coefficient(double friction_coeff)
-> MechanicalSystem&
{
  _pimpl->friction_coefficient = friction_coeff;
  return *this;
}

//==============================================================================
double MechanicalSystem::friction_coefficient() const
{
  return _pimpl->friction_coefficient;
}

//==============================================================================
auto MechanicalSystem::inertia(double inertia)
-> MechanicalSystem&
{
  _pimpl->inertia = inertia;
  return *this;
}

//==============================================================================
double MechanicalSystem::inertia() const
{
  return _pimpl->inertia;
}

//==============================================================================
bool MechanicalSystem::valid() const
{
  return _pimpl->mass > 0.0 && _pimpl->friction_coefficient > 0.0 &&
    _pimpl->inertia > 0.0;
}

} // namespace agv
} // namespace rmf_battery
