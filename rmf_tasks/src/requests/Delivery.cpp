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

#include <rmf_tasks/requests/Delivery.hpp>

namespace rmf_tasks {
namespace requests {

class Delivery::Implementation
{
public:

  Implementation(
    std::size_t id,
    std::size_t pickup_waypoint,
    std::size_t dropoff_waypoint,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    bool drain_battery = true,
    rmf_traffic::Time start_time)
  : _id(id),
    _pickup_waypoint(pickup_waypoint)
  {}

  std::size_t _id;
  std::size_t _pickup_waypoint;
  std::size_t _dropoff_waypoint;
  std::shared_ptr<rmf_battery::MotionPowerSink> _motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> _device_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  bool _drain_battery;
  double _invariant_battery_drain;
  rmf_traffic::Duration _invariant_duration;
  rmf_traffic::Time _start_time;
};

std::size_t Delivery::id() const
{}

rmf_utils::optional<rmf_tasks::Estimate> estimate_finish(
  const RobotState& initial_state) const final;

rmf_traffic::Duration invariant_duration() const final;

rmf_traffic::Time earliest_start_time() const final;


} // namespace requests
} // namespace rmf_tasks
