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

#ifndef RMF_TASK__REQUESTS__LOOP_HPP
#define RMF_TASK__REQUESTS__LOOP_HPP

#include <chrono>
#include <string>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

#include <rmf_task/agv/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

class Loop : public rmf_task::Request
{
public:

  using Start = rmf_traffic::agv::Planner::Start;

  static ConstRequestPtr make(
    std::string id,
    std::size_t start_waypoint,
    std::size_t finish_waypoint,
    std::size_t num_loops,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    bool drain_battery = true);

  std::string id() const final;

  rmf_utils::optional<rmf_task::Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::StateConfig& state_config,
    const std::shared_ptr<EstimateCache> estimate_cache) const final;

  rmf_traffic::Duration invariant_duration() const final;

  rmf_traffic::Time earliest_start_time() const final;

  std::size_t start_waypoint() const;

  std::size_t finish_waypoint() const;

  std::size_t num_loops() const;

  Start loop_start(const Start& start) const;

  Start loop_end(const Start& start) const;

  class Implementation;
private:
  Loop();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using LoopRequestPtr = std::shared_ptr<Loop>;
using ConstLoopRequestPtr = std::shared_ptr<const Loop>;

} // namespace tasks
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__LOOP_HPP
