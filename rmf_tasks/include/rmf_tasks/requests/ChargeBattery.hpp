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

#ifndef INCLUDE__RMF_TASKS__REQUESTS__CHARGEBATTERY_HPP
#define INCLUDE__RMF_TASKS__REQUESTS__CHARGEBATTERY_HPP

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_utils/optional.hpp>

#include <rmf_tasks/agv/State.hpp>
#include <rmf_tasks/Request.hpp>
#include <rmf_tasks/Estimate.hpp>

namespace rmf_tasks {
namespace requests {

class ChargeBattery : public rmf_tasks::Request
{
public:

  static rmf_tasks::Request::SharedPtr make(
    rmf_battery::agv::BatterySystem battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    bool drain_battery = true);

  std::size_t id() const final;

  rmf_utils::optional<rmf_tasks::Estimate> estimate_finish(
    rmf_traffic::Time time_now,
    const agv::State& initial_state,
    const agv::StateConfig& state_config) const final;

  rmf_traffic::Duration invariant_duration() const final;

  rmf_traffic::Time earliest_start_time() const final;

  class Implementation;
private:
  ChargeBattery();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace requests
} // namespace rmf_tasks

#endif // INCLUDE__RMF_TASKS__REQUESTS__CHARGEBATTERY_HPP
