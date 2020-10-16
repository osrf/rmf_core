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

#ifndef RMF_BATTERY__SIMPLEDEVICEPOWERSINK_HPP
#define RMF_BATTERY__SIMPLEDEVICEPOWERSINK_HPP

#include <rmf_battery/DevicePowerSink.hpp>
#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/PowerSystem.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_battery {
namespace agv {

//==============================================================================
class SimpleDevicePowerSink : public DevicePowerSink
{
public:

  /// Constructor
  ///
  /// \param[in] battery_system
  ///   The BatterySystem of the robot
  ///
  /// \param[in] power_system
  ///   The PowerSystem for this device  
  SimpleDevicePowerSink(
    BatterySystem& battery_system,
    PowerSystem& power_system);

  /// Get a constant reference to the battery system
  const BatterySystem& battery_system() const;

  /// Get a constant reference to the power system
  const PowerSystem& power_system() const;

  /// Compute change in state-of-charge of the battery due to an onboard
  /// device over a time period.
  ///
  /// \param[in] run_time
  ///   The duration in seconds over which the power system drains charge from
  ///   the battery
  ///
  /// \return The charge depleted as a fraction of the total battery capacity
  virtual double compute_change_in_charge(
    const double run_time) const final;
  
  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__SIMPLEDEVICEPOWERSINK_HPP
