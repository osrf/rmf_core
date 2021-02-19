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

#ifndef RMF_TASK__REQUESTS__CLEAN_HPP
#define RMF_TASK__REQUESTS__CLEAN_HPP

#include <chrono>
#include <string>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_utils/optional.hpp>

#include <rmf_task/agv/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

class CleanDescription : public rmf_task::Request::Description
{
public:

  static DescriptionPtr make(
    std::size_t start_waypoint,
    std::size_t end_waypoint,
    rmf_traffic::Trajectory& cleaning_path,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> cleaning_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    bool drain_battery = true);

  rmf_utils::optional<rmf_task::Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::Constraints& task_planning_constraints,
    const std::shared_ptr<EstimateCache> estimate_cache) const final;

  rmf_traffic::Duration invariant_duration() const final;

  /// Get the start waypoint in this request
  std::size_t start_waypoint() const;

  /// Get the end waypoint in this request
  std::size_t end_waypoint() const;

  /// Get the Start at the end of the cleaning trajectory from an initial Start
  rmf_traffic::agv::Planner::Start location_after_clean(
    rmf_traffic::agv::Planner::Start start) const;

  class Implementation;
private:
  CleanDescription();

  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Clean
{
public:

  static ConstRequestPtr make(
    const std::string& id,
    std::size_t start_waypoint,
    std::size_t end_waypoint,
    rmf_traffic::Trajectory& cleaning_path,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> cleaning_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    bool drain_battery = true,
    ConstPriorityPtr priority = nullptr);
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__CLEAN_HPP
