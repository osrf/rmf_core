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

#include <rmf_tasks/agv/TaskPlanner.hpp>

#include <thread>

namespace rmf_tasks {
namespace agv {


//==============================================================================
class TaskPlanner::Configuration::Implementation
{
public:

  Request::SharedPtr charge_battery_request;
  FilterType filter_type;
};

TaskPlanner::Configuration::Configuration(
  Request::SharedPtr charge_battery_request,
  const FilterType filter_type)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(charge_battery_request),
        filter_type
      }))
{
  // Do nothing
}

//==============================================================================
Request::SharedPtr TaskPlanner::Configuration::charge_battery_request() const
{
  return std::move(_pimpl->charge_battery_request);
}

//==============================================================================
auto TaskPlanner::Configuration::charge_battery_request(
  Request::SharedPtr charge_battery_request) -> Configuration&
{
  _pimpl->charge_battery_request = std::move(charge_battery_request);
  return *this;
}

//==============================================================================
TaskPlanner::FilterType TaskPlanner::Configuration::filter_type() const
{
  return _pimpl->filter_type;
}

//==============================================================================
auto TaskPlanner::Configuration::filter_type(
  TaskPlanner::FilterType filter_type) -> Configuration&
{
  _pimpl->filter_type = filter_type;
  return *this;
}

//==============================================================================
class TaskPlanner::Assignment::Implementation
{
public:

  std::size_t task_id;
  State state;
  rmf_traffic::Time earliest_start_time;
};

//==============================================================================
TaskPlanner::Assignment::Assignment(
  std::size_t task_id,
  State state,
  rmf_traffic::Time earliest_start_time)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        task_id,
        std::move(state),
        earliest_start_time
      }))
{
  // Do nothing
}

//==============================================================================
std::size_t TaskPlanner::Assignment::task_id() const 
{
  return _pimpl->task_id;
}

//==============================================================================
const State& TaskPlanner::Assignment::state() const
{
  return _pimpl->state;
}

//==============================================================================
const rmf_traffic::Time& TaskPlanner::Assignment::earliest_start_time() const
{
  return _pimpl->earliest_start_time;
}

//==============================================================================

namespace {

} // anonymous namespace


class TaskPlanner::Implementation
{

}


} // namespace agv
} // namespace rmf_tasks
