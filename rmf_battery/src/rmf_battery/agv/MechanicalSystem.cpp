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
  double moment_of_inertia;
  double friction_coefficient;
};

//==============================================================================
std::optional<MechanicalSystem> MechanicalSystem::make(
  double mass,
  double moment_of_inertia,
  double friction_coefficient)
{
  if (mass <= 0.0 || moment_of_inertia <= 0.0 || friction_coefficient <= 0.0)
    return std::nullopt;

  MechanicalSystem mechanical_system;
  mechanical_system._pimpl->mass = mass;
  mechanical_system._pimpl->moment_of_inertia = moment_of_inertia;
  mechanical_system._pimpl->friction_coefficient = friction_coefficient;

  return mechanical_system;
}
//==============================================================================
MechanicalSystem::MechanicalSystem()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
double MechanicalSystem::mass() const
{
  return _pimpl->mass;
}

//==============================================================================
double MechanicalSystem::friction_coefficient() const
{
  return _pimpl->friction_coefficient;
}

//==============================================================================
double MechanicalSystem::moment_of_inertia() const
{
  return _pimpl->moment_of_inertia;
}

} // namespace agv
} // namespace rmf_battery
