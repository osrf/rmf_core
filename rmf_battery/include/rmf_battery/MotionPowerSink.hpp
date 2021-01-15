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

#ifndef RMF_BATTERY__MOTIONPOWERSINK_HPP
#define RMF_BATTERY__MOTIONPOWERSINK_HPP

#include <rmf_traffic/Trajectory.hpp>

namespace rmf_battery {


//==============================================================================
class MotionPowerSink
{
public:

  /// Compute change in state-of-charge of the battery due to locomotion
  /// of the robot along a trajectory.
  ///
  /// \param[in] trajectory
  ///   A valid rmf_traffic:::Trajectory over which the change in charge has to
  ///   to be computed
  ///
  /// \return The charge depleted as a fraction of the total battery capacity
  virtual double compute_change_in_charge(
    const rmf_traffic::Trajectory& trajectory) const = 0;

  virtual ~MotionPowerSink() = default;
};

} // namespace rmf_battery

#endif // RMF_BATTERY__MOTIONPOWERSINK_HPP
