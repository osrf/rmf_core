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

#ifndef RMF_BATTERY__DEVICEPOWERSINK_HPP
#define RMF_BATTERY__DEVICEPOWERSINK_HPP

namespace rmf_battery {


//==============================================================================
class DevicePowerSink
{
public:

  /// Compute change in state-of-charge of the battery due to an onboard
  /// device over a time period.
  ///
  /// \param[in] run_time
  ///   The duration in seconds over which the power system drains charge from
  ///   the battery
  ///
  /// \return The charge depleted as a fraction of the total battery capacity
  virtual double compute_change_in_charge(
    const double run_time) const = 0;

  virtual ~DevicePowerSink() = default;
};

} // namespace rmf_battery

#endif // RMF_BATTERY__DEVICEPOWERSINK_HPP
