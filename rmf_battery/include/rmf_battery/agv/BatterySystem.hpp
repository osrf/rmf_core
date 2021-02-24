
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

#ifndef RMF_BATTERY__AGV__BATTERYSYSTEM_HPP
#define RMF_BATTERY__AGV__BATTERYSYSTEM_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <optional>

namespace rmf_battery {
namespace agv {

class BatterySystem
{
public:
  /// Returns a BatterySystem object if valid values were supplied for the
  /// various fields else returns std::nullopt. Here valid implies that the
  /// values are greater than zero.
  /// \param[in] nominal_voltage
  ///   The nominal voltage of the battery in Volts
  ///
  /// \param[in] capacity
  ///   The nominal capacity of the battery in Ampere-hours
  ///
  /// \param[in] charging_current
  ///   The rated current in Amperes for charging the battery
  static std::optional<BatterySystem> make(
    double nominal_voltage,
    double capacity,
    double charging_current);

  /// Get the nominal voltage of this battery system
  double nominal_voltage() const;

  /// Get the capacity of this battery system
  double capacity() const;

  /// Get the charging current of this battery system
  double charging_current() const;

  class Implementation;
private:
  BatterySystem();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using BatterySystemPtr = std::shared_ptr<BatterySystem>;
using ConstBatterySystemPtr = std::shared_ptr<const BatterySystem>;

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__BATTERYSYSTEM_HPP
