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

#ifndef RMF_BATTERY__AGV__CLEANINGTASKPLANNER_HPP
#define RMF_BATTERY__AGV__CLEANINGTASKPLANNER_HPP

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/Trajectory.hpp>

#include <rmf_battery/agv/SystemTraits.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <vector>

namespace rmf_battery {
namespace agv {

class CleaningTaskPlanner
{
public:
  using Planner = rmf_traffic::agv::Planner;
  /// Constructor
  ///
  /// \param[in] system_traits
  ///   An instance of rmf_battery::agv::SystemTraits.
  ///
  /// \param[in] planner
  ///   An instance of rmf_traffic::agv::Planner. A valid reference is to be 
  ///   maintained by the user.
  CleaningTaskPlanner(
    SystemTraits& system_traits,
    Planner& planner);

  /// Get a const reference to the rmf_battery::agv::SystemTraits instance
  const SystemTraits& system_traits() const;

  /// Get a const reference to the rmf_traffic::agv::Planner instance
  const Planner& planner() const;

  /// Produce a set of valid trajectories if the robot can successfully travel
  /// from start location to the cleaning waypoint, complete the cleaning
  /// process and has sufficent battery to return to its nearest charging
  /// waypoint if required. If no feasible plan is found, an empty set is
  /// returned
  ///
  /// \param[in] start
  ///   The starting conditions
  ///
  /// \param[in] cleaning_start_waypoint
  ///   The waypoint where the robot needs to reach to begin cleaning
  ///
  /// \param[in] cleaning_trajectory
  ///   The trajectory to be followed by the robot while cleaning
  ///
  /// \param[in] cleaning_system
  ///   The name of the cleaning system. The name must match one of the power
  ///   systems of the robot as configured in SystemTraits.
  ///
  /// \param[in] cleaning_end_waypoint
  ///   The waypoint where the robot stops after cleaning
  ///
  /// \param[in] charging_station_waypoint
  ///   The nearest charging station waypoint for the robot
  ///
  std::vector<rmf_traffic::Trajectory> plan(
    const Planner::Start& start,
    const std::size_t cleaning_start_waypoint,
    const rmf_traffic::Trajectory& cleaning_trajectory,
    const std::string& cleaning_system,
    const std::size_t cleaning_end_waypoint,
    const std::size_t charging_station_waypoint);

  class Implementation;

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__CLEANINGTASKPLANNER_HPP
