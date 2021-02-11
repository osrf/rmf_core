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

#ifndef RMF_TASK__REQUESTS__CHARGEBATTERY_HPP
#define RMF_TASK__REQUESTS__CHARGEBATTERY_HPP

#include <string>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_utils/optional.hpp>

#include <rmf_task/agv/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

namespace rmf_task {
namespace requests {

class ChargeBattery : public rmf_task::Request
{
public:

  static ConstRequestPtr make(
    rmf_battery::agv::BatterySystem battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    bool drain_battery = true,
    ConstPriorityPtr priority = nullptr);

  rmf_utils::optional<rmf_task::Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::Constraints& task_planning_constraints,
    const std::shared_ptr<EstimateCache> estimate_cache) const final;

  rmf_traffic::Duration invariant_duration() const final;

  /// Get the battery system in this request
  const rmf_battery::agv::BatterySystem& battery_system() const;

  /// Retrieve the charge soc which the battery will be charged upto
  double max_charge_soc() const;

  class Implementation;
private:
  ChargeBattery(
    std::string& id,
    rmf_traffic::Time earlier_start_time,
    ConstPriorityPtr priority);
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using ChargeBatteryRequestPtr = std::shared_ptr<ChargeBattery>;
using ConstChargeBatteryRequestPtr = std::shared_ptr<const ChargeBattery>;

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__CHARGEBATTERY_HPP
