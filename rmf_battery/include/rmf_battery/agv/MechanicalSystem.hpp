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

#ifndef RMF_BATTERY__AGV__MECHANICALSYSTEM_HPP
#define RMF_BATTERY__AGV__MECHANICALSYSTEM_HPP

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_battery {
namespace agv {


class MechanicalSystem
{
public:
  /// Constructor
  ///
  /// \param[in] mass
  ///   The mass of the robot in kilograms
  ///
  /// \param[in] inertia
  ///   The moment of inertia of the robot along its yaw axis
  ///
  /// \param[in] friction_coefficient
  ///   The coefficient of kinetic friction measured at the wheels of the robot. 
  ///   This value is used to compute the energy loss due to rotation of the
  ///   vehicle's wheels during locomotion.
  MechanicalSystem(
    double mass,
    double inertia,
    double friction_coefficient);

  MechanicalSystem& mass(double mass);
  double mass() const;

  MechanicalSystem& inertia(double inertia);
  double inertia() const;

  MechanicalSystem& friction_coefficient(double friction_coeff);
  double friction_coefficient() const;

  /// Returns true if the values are valid, i.e. greater than zero.
  bool valid() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using MechanicalSystemPtr = std::shared_ptr<MechanicalSystem>;
using ConstMechanicalSystemPtr = std::shared_ptr<const MechanicalSystem>;

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__MECHANICALSYSTEM_HPP
