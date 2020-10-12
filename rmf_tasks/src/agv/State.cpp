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

#include <stdexcept>

#include <rmf_tasks/agv/State.hpp>

namespace rmf_tasks {
namespace agv {

//==============================================================================
class State::Implementation
{
public:

  Implementation(
    rmf_traffic::agv::Plan::Start plan_start,
    std::size_t charging_waypoint,
    double battery_soc)
  : _plan_start(std::move(plan_start)),
    _charging_waypoint(charging_waypoint),
    _battery_soc(battery_soc)
  {
    if (_battery_soc < 0.0 || _battery_soc > 1.0)
      throw std::invalid_argument(
        "Battery State of Charge needs be between 0.0 and 1.0.");
  }

  rmf_traffic::agv::Plan::Start _plan_start;
  std::size_t _charging_waypoint;
  double _battery_soc;
};

//==============================================================================
State::State(
  rmf_traffic::agv::Plan::Start plan_start,
  std::size_t charging_waypoint,
  double battery_soc)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation(std::move(plan_start), charging_waypoint, battery_soc)))
{}

//==============================================================================
rmf_traffic::agv::Plan::Start State::plan_start() const
{
  return _pimpl->_plan_start;
}

//==============================================================================
State& State::plan_start(rmf_traffic::agv::Plan::Start new_plan_start)
{
  _pimpl->_plan_start = std::move(new_plan_start);
  return *this;
}

//==============================================================================
std::size_t State::charging_waypoint() const
{
  return _pimpl->_charging_waypoint;
}

//==============================================================================
State& State::charging_waypoint(std::size_t new_charging_waypoint)
{
  _pimpl->_charging_waypoint = new_charging_waypoint;
  return *this;
}

//==============================================================================
double State::battery_soc() const
{
  return _pimpl->_battery_soc;
}

//==============================================================================
State& State::battery_soc(double new_battery_soc)
{
  if (new_battery_soc < 0.0 || new_battery_soc > 1.0)
    throw std::invalid_argument(
      "Battery State of Charge needs be between 0.0 and 1.0.");

  _pimpl->_battery_soc = new_battery_soc;
  return *this;
}

//==============================================================================
std::size_t State::waypoint() const
{
  return _pimpl->_plan_start.waypoint();
}

//==============================================================================
State& State::waypoint(std::size_t new_waypoint)
{
  _pimpl->_plan_start.waypoint(new_waypoint);
  return *this;
}

//==============================================================================
rmf_traffic::Time State::finish_time() const
{
  return _pimpl->_plan_start.time();
}

//==============================================================================
State& State::finish_time(rmf_traffic::Time new_finish_time)
{
  _pimpl->_plan_start.time(new_finish_time);
  return *this;
}

//==============================================================================
} // namespace agv
} // namespace rmf_tasks
