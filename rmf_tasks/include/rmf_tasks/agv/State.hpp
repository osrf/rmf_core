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

#ifndef INCLUDE__RMF_TASKS__AGV__STATE_HPP
#define INCLUDE__RMF_TASKS__AGV__STATE_HPP

#include <chrono>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_tasks {
namespace agv {

/// This state representation is used for task planning.
class State
{
public:

  /// Constructor
  ///
  /// \param[in] plan_start
  ///   Current state's next start for new plans, includes the time which
  ///   the plan can feasibly start, according to the finishing time of any
  ///   tasks that the robot is currently performing.
  ///
  /// \param[in] charging_waypoint
  ///   The charging waypoint index of this robot.
  ///
  /// \param[in] battery_soc
  ///   Current battery state of charge of the robot. This value needs to be
  ///   between 0.0 to 1.0.
  State(
    rmf_traffic::agv::Plan::Start plan_start,
    std::size_t charging_waypoint,
    double battery_soc);
  
  /// The next plan start based on the current state.
  rmf_traffic::agv::Plan::Start plan_start() const;

  /// Sets the next plan start.
  State& plan_start(rmf_traffic::agv::Plan::Start new_plan_start);

  /// Robot's charging waypoint index.
  std::size_t charging_waypoint() const;

  /// Sets the charging waypoint index.
  State& charging_waypoint(std::size_t new_charging_waypoint);

  /// The current battery state of charge of the robot. This value is between
  /// 0.0 and 1.0.
  double battery_soc() const;

  /// Sets a new battery state of charge value. This value needs to be between
  /// 0.0 and 1.0.
  State& battery_soc(double new_battery_soc);

  /// The current location waypoint index.
  std::size_t waypoint() const;

  /// Sets the current location waypoint index.
  State& waypoint(std::size_t new_waypoint);

  /// The time which the robot finishes its current task or when it is ready for
  /// a new task.
  rmf_traffic::Time finish_time() const;

  /// Sets the finish time for the robot.
  State& finish_time(rmf_traffic::Time new_finish_time);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_tasks

#endif // INCLUDE__RMF_TASKS__AGV__STATE_HPP
