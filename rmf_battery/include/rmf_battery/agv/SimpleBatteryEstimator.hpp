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

#ifndef RMF_BATTERY__AGV__SIMPLEBATTERYESTIMATOR_HPP
#define RMF_BATTERY__AGV__SIMPLEBATTERYESTIMATOR_HPP

#include <rmf_battery/EstimateBattery.hpp>

#include <rmf_utils/impl_ptr.hpp>


namespace rmf_battery {
namespace agv {

class SimpleBatteryEstimator : public EstimateBattery
{
public:

  /// Constructor
  SimpleBatteryEstimator(SystemTraits& system_traits);

  SimpleBatteryEstimator& system_traits(SystemTraits system_traits);

  const SystemTraits system_traits() const;

  /// Computes state-of-charge estimate of battery at the end of a trajectory.
  ///
  /// \param[in] trajectory
  ///   A valid rmf_traffic:::Trajectory
  ///
  /// \param[in] initial_soc
  ///   The initial state of charge of the robot at the beginning of the
  ///   trajectory as a fraction of total battery capacity
  ///
  /// \param[in] power_map
  ///   An optional unordered map with keys representing names of power systems
  ///   and values of trajectories during which the power system is active.
  ///
  /// \return The remaining charge at the end of the trajectory as a fraction of
  ///   total battery capacity
  double compute_state_of_charge(
    const rmf_traffic::Trajectory& trajectory,
    const double initial_soc,
    rmf_utils::optional<PowerMap> power_map = rmf_utils::nullopt
  ) const final;

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__SIMPLEBATTERYESTIMATOR_HPP