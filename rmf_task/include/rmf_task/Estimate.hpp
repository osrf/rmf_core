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

#ifndef RMF_TASK__ESTIMATE_HPP
#define RMF_TASK__ESTIMATE_HPP

#include <optional>
#include <utility>

#include <rmf_task/agv/State.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {

/// Estimates for requests
class Estimate
{
public:

  /// Constructor of an estimate of a task request.
  ///
  /// \param[in] finish_state
  ///   Finish state of the robot once it completes the task request.
  ///
  /// \param[in] wait_until
  ///   The ideal time the robot starts executing this task.
  Estimate(agv::State finish_state, rmf_traffic::Time wait_until);

  /// Finish state of the robot once it completes the task request.
  agv::State finish_state() const;

  /// Sets a new finish state for the robot.
  Estimate& finish_state(agv::State new_finish_state);

  /// The ideal time the robot starts executing this task.
  rmf_traffic::Time wait_until() const;

  /// Sets a new starting time for the robot to execute the task request.
  Estimate& wait_until(rmf_traffic::Time new_wait_until);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// Stores computed estimates between pairs of waypoints
class EstimateCache
{
public:
  /// Constructs an EstimateCache
  ///
  /// \param[in] N
  ///   The maximum number of waypoints in the navigation graph
  EstimateCache(std::size_t N);

  /// Struct containing the estimated duration and charge required to travel between
  /// a waypoint pair.
  struct CacheElement
  {
    rmf_traffic::Duration duration;
    double dsoc; // Positive if charge is consumed
  };

  /// Returns the saved estimate values for the path between the supplied waypoints,
  /// if present.
  std::optional<CacheElement> get(std::pair<size_t, size_t> waypoints) const;

  /// Saves the estimated duration and change in charge between the supplied waypoints.
  void set(std::pair<size_t, size_t> waypoints,
    rmf_traffic::Duration duration, double dsoc);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_task

#endif // RMF_TASK__ESTIMATE_HPP
