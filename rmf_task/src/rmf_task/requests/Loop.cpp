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

#include <rmf_task/requests/Loop.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class Loop::Implementation
{
public:

  Implementation()
  {}

  std::size_t id;
  std::size_t start_waypoint;
  std::size_t finish_waypoint;
  std::size_t num_loops;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> device_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  bool drain_battery;
  rmf_traffic::Time start_time;

  rmf_traffic::Duration invariant_duration;
  double invariant_battery_drain;
};



} // namespace requests
} // namespace rmf_task