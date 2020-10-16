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

#ifndef RMF_BATTERY__AGV__POWERSYSTEM_HPP
#define RMF_BATTERY__AGV__POWERSYSTEM_HPP

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_battery {
namespace agv {


class PowerSystem
{
public:

  /// Constructor
  ///
  /// \param[in] name
  ///   A string representing the name of this power system
  ///
  /// \param[in] nominal_power
  ///   The rated nominal power consumption in Watts for this power system
  ///
  PowerSystem(
    std::string name,
    double nominal_power);

  PowerSystem& name(std::string name);
  const std::string& name() const;

  PowerSystem& nominal_power(double nom_power);
  double nominal_power() const;

  /// Returns true if the values are valid, i.e. greater than zero.
  bool valid() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using PowerSystemPtr = std::shared_ptr<PowerSystem>;
using ConstPowerSystemPtr = std::shared_ptr<const PowerSystem>;

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__POWERSYSTEM_HPP
