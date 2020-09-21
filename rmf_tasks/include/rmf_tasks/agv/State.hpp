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

namespace rmf_tasks {
namespace agv {

class State
{
public:

  /// Constructor
  ///
  /// \param[in] waypoint
  /// \param[in] charging_waypoint
  /// \param[in] finish_duration
  ///   The duration of time until current tasks are finished.
  /// \param[in] battery_soc
  /// \param[in] threshold_soc
  State(
    std::size_t waypoint, 
    std::size_t charging_waypoint,
    rmf_traffic::Duration finish_duration = rmf_traffic::Duration(0),
    double battery_soc = 1.0,
    double threshold_soc = 0.2);
  
  std::size_t waypoint() const;

  State& waypoint(std::size_t new_waypoint);

  std::size_t charging_waypoint() const;

  State& charging_waypoint(std::size_t new_charging_waypoint);

  rmf_traffic::Duration finish_duration() const;

  State& finish_duration(rmf_traffic::Duration new_finish_duration);

  double battery_soc() const;

  State& battery_soc(double new_battery_soc);

  double threshold_soc() const;

  State& threshold_soc(double new_threshold_soc);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_tasks

#endif // INCLUDE__RMF_TASKS__AGV__STATE_HPP
