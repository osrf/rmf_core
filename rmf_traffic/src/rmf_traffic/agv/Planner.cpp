/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic/agv/Planner.hpp>

#include "planning_internal.hpp"

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Planner::Configuration::Implementation
{
public:

  Graph graph;
  VehicleTraits traits;
  Interpolate::Options interpolation;

};

//==============================================================================
Planner::Configuration::Configuration(
    Graph graph,
    VehicleTraits traits,
    Interpolate::Options interpolation)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(graph),
               std::move(traits),
               std::move(interpolation)
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Configuration::graph(Graph graph) -> Configuration&
{
  _pimpl->graph = std::move(graph);
  return *this;
}

//==============================================================================
Graph& Planner::Configuration::graph()
{
  return _pimpl->graph;
}

//==============================================================================
const Graph& Planner::Configuration::graph() const
{
  return _pimpl->graph;
}

//==============================================================================
auto Planner::Configuration::vehicle_traits(VehicleTraits traits)
-> Configuration&
{
  _pimpl->traits = std::move(traits);
  return *this;
}

//==============================================================================
VehicleTraits& Planner::Configuration::vehicle_traits()
{
  return _pimpl->traits;
}

//==============================================================================
const VehicleTraits& Planner::Configuration::vehicle_traits() const
{
  return _pimpl->traits;
}

//==============================================================================
auto Planner::Configuration::interpolation(Interpolate::Options interpolate)
-> Configuration&
{
  _pimpl->interpolation = std::move(interpolate);
  return *this;
}

//==============================================================================
Interpolate::Options& Planner::Configuration::interpolation()
{
  return _pimpl->interpolation;
}

//==============================================================================
const Interpolate::Options& Planner::Configuration::interpolation() const
{
  return _pimpl->interpolation;
}

//==============================================================================
class Planner::Options::Implementation
{
public:

  const schedule::Viewer* viewer;
  Duration min_hold_time;

};

//==============================================================================
Planner::Options::Options(
    const schedule::Viewer& viewer,
    const Duration min_hold_time)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               &viewer,
               min_hold_time
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Options::schedule_viewer(const schedule::Viewer& viewer)
-> Options&
{
  _pimpl->viewer = &viewer;
  return *this;
}

//==============================================================================
const schedule::Viewer& Planner::Options::schedule_viewer() const
{
  return *_pimpl->viewer;
}

//==============================================================================
auto Planner::Options::minimum_holding_time(const Duration holding_time)
-> Options&
{
  _pimpl->min_hold_time = holding_time;
  return *this;
}

//==============================================================================
Duration Planner::Options::minimum_holding_time() const
{
  return _pimpl->min_hold_time;
}

//==============================================================================
class Planner::Start::Implementation
{
public:

  Time time;
  std::size_t waypoint;
  double orientation;

};

//==============================================================================
Planner::Start::Start(
    const Time initial_time,
    const std::size_t initial_waypoint,
    const double initial_orientation)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               initial_time,
               initial_waypoint,
               initial_orientation
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Start::time(const Time initial_time) -> Start&
{
  _pimpl->time = initial_time;
  return *this;
}

//==============================================================================
Time Planner::Start::time() const
{
  return _pimpl->time;
}

//==============================================================================
auto Planner::Start::waypoint(const std::size_t initial_waypoint) -> Start&
{
  _pimpl->waypoint = initial_waypoint;
  return *this;
}

//==============================================================================
std::size_t Planner::Start::waypoint() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Planner::Start::orientation(const double initial_orientation) -> Start&
{
  _pimpl->orientation = initial_orientation;
  return *this;
}

//==============================================================================
double Planner::Start::orientation() const
{
  return _pimpl->orientation;
}

//==============================================================================
class Planner::Goal::Implementation
{
public:

  std::size_t waypoint;

  // TODO(MXG): Replace with std::optional when we have C++17 support
  bool has_orientation;
  double orientation;

};

//==============================================================================
Planner::Goal::Goal(const std::size_t waypoint)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               waypoint,
               false,
               0.0
             }))
{
  // Do nothing
}

//==============================================================================
Planner::Goal::Goal(
    const std::size_t waypoint,
    const double goal_orientation)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               waypoint,
               true,
               goal_orientation
             }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Goal::waypoint(const std::size_t goal_waypoint) -> Goal&
{
  _pimpl->waypoint = goal_waypoint;
  return *this;
}

//==============================================================================
std::size_t Planner::Goal::waypoint() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Planner::Goal::orientation(const double goal_orientation) -> Goal&
{
  _pimpl->has_orientation = true;
  _pimpl->orientation = goal_orientation;
  return *this;
}

//==============================================================================
auto Planner::Goal::any_orientation() -> Goal&
{
  _pimpl->has_orientation = false;
  return *this;
}

//==============================================================================
const double* Planner::Goal::orientation() const
{
  if(_pimpl->has_orientation)
    return &_pimpl->orientation;

  return nullptr;
}

//==============================================================================
class Planner::Implementation
{
public:

  internal::planning::CacheManager cache_mgr;

  Options default_options;

  Configuration configuration;

};

//==============================================================================
Planner::Planner(
    Configuration config,
    Options default_options)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               internal::planning::make_cache(config),
               std::move(default_options),
               config
             }))
{
  // Do nothing
}

//==============================================================================
Plan Planner::plan(Start start, Goal goal) const
{
  return _pimpl->cache_mgr.get().plan(
        std::move(start), std::move(goal), _pimpl->default_options);
}

//==============================================================================
Plan Planner::plan(Start start, Goal goal, Options options) const
{
  return _pimpl->cache_mgr.get().plan(
        std::move(start), std::move(goal), std::move(options));
}

//==============================================================================
class Plan::Implementation
{
public:

  bool solved;

  internal::planning::CacheManager cache_mgr;

  Planner::Options options;

};

//==============================================================================
bool Plan::valid() const
{
  return (_pimpl && _pimpl->solved);
}

//==============================================================================
Plan::operator bool() const
{
  return valid();
}

} // namespace agv
} // namespace rmf_traffic
