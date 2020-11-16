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

#include <map>

#include <rmf_task/requests/Clean.hpp>

namespace rmf_task {
namespace requests {

//==============================================================================
class Clean::Implementation
{
public:

  Implementation()
  {}

  std::string id;
  std::size_t start_waypoint;
  std::size_t end_waypoint;
  rmf_traffic::Trajectory cleaning_path;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> cleaning_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  bool drain_battery;
  rmf_traffic::Time start_time;

  rmf_traffic::Duration invariant_duration;
  double invariant_battery_drain;
};

//==============================================================================
rmf_task::ConstRequestPtr Clean::make(
  std::string id,
  std::size_t start_waypoint,
  std::size_t end_waypoint,
  rmf_traffic::Trajectory& cleaning_path,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> cleaning_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  rmf_traffic::Time start_time,
  bool drain_battery)
{
  std::shared_ptr<Clean> clean(new Clean());
  clean->_pimpl->id = id;
  clean->_pimpl->start_waypoint = start_waypoint;
  clean->_pimpl->end_waypoint = end_waypoint;
  clean->_pimpl->cleaning_path = cleaning_path;
  clean->_pimpl->motion_sink = std::move(motion_sink);
  clean->_pimpl->ambient_sink = std::move(ambient_sink);
  clean->_pimpl->cleaning_sink = std::move(cleaning_sink);
  clean->_pimpl->planner = std::move(planner);
  clean->_pimpl->drain_battery = drain_battery;
  clean->_pimpl->start_time = start_time;

  // Calculate duration of invariant component of task
  const auto& cleaning_start_time = cleaning_path.begin()->time();
  const auto& cleaning_finish_time = *cleaning_path.finish_time();
  
  clean->_pimpl->invariant_duration =
    cleaning_finish_time - cleaning_start_time;
  clean->_pimpl->invariant_battery_drain = 0.0;

  if (clean->_pimpl->drain_battery)
  {
    // Compute battery drain over invariant path
    const double dSOC_motion =
      clean->_pimpl->motion_sink->compute_change_in_charge(cleaning_path);
    const double dSOC_ambient =
      clean->_pimpl->ambient_sink->compute_change_in_charge(
        rmf_traffic::time::to_seconds(clean->_pimpl->invariant_duration));
    const double dSOC_cleaning =
      clean->_pimpl->cleaning_sink->compute_change_in_charge(
        rmf_traffic::time::to_seconds(clean->_pimpl->invariant_duration));
    clean->_pimpl->invariant_battery_drain = dSOC_motion + dSOC_ambient +
      dSOC_cleaning;
  }

  return clean;
}

//==============================================================================
Clean::Clean()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
std::string Clean::id() const
{
  return _pimpl->id;
}

//==============================================================================
rmf_utils::optional<rmf_task::Estimate> Clean::estimate_finish(
  const agv::State& initial_state,
  const agv::StateConfig& state_config,
  const std::shared_ptr<EstimateCache> estimate_cache) const
{
  rmf_traffic::agv::Plan::Start final_plan_start{
    initial_state.finish_time(),
    _pimpl->end_waypoint,
    initial_state.location().orientation()};
  agv::State state{
    std::move(final_plan_start),
    initial_state.charging_waypoint(),
    initial_state.battery_soc()};

  rmf_traffic::Duration variant_duration(0);
  rmf_traffic::Duration end_duration(0);

  const rmf_traffic::Time start_time = initial_state.finish_time();
  double battery_soc = initial_state.battery_soc();
  double dSOC_motion = 0.0;
  double dSOC_ambient = 0.0;

  if (initial_state.waypoint() != _pimpl->start_waypoint)
  {
    const auto endpoints = std::make_pair(initial_state.waypoint(),
      _pimpl->start_waypoint);
    const auto& cache_result = estimate_cache->get(endpoints);
    if (cache_result)
    {
      variant_duration = cache_result->duration;
      battery_soc = battery_soc - cache_result->dsoc;
    }
    else
    {
      rmf_traffic::agv::Planner::Goal goal{endpoints.second};

      const auto result_to_start = _pimpl->planner->plan(
        initial_state.location(), goal);
      // We assume we can always compute a plan
      const auto& trajectory =
        result_to_start->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      variant_duration = finish_time - start_time;

      if(_pimpl->drain_battery)
      {
        // Compute battery drain
        dSOC_motion = _pimpl->motion_sink->compute_change_in_charge(trajectory);
        dSOC_ambient =
          _pimpl->ambient_sink->compute_change_in_charge(
            rmf_traffic::time::to_seconds(variant_duration));
        battery_soc = battery_soc - dSOC_motion - dSOC_ambient;
      }

      estimate_cache->set(endpoints, variant_duration,
        dSOC_motion + dSOC_ambient);
    }

    if (battery_soc <= state_config.threshold_soc())
      return rmf_utils::nullopt;
  }

  if (_pimpl->start_waypoint != _pimpl->end_waypoint)
  {
    // TODO(YV) Account for battery drain and duration when robot moves from
    // end of cleaning trajectory to its end_waypoint. We currently define the
    // end_waypoint near the start_waypoint in the nav graph for minimum error
  }

  const rmf_traffic::Time ideal_start = _pimpl->start_time - variant_duration;
  const rmf_traffic::Time wait_until =
    initial_state.finish_time() > ideal_start ?
    initial_state.finish_time() : ideal_start;

  // Factor in invariants
  state.finish_time(
    wait_until + variant_duration + _pimpl->invariant_duration + end_duration);

  if (_pimpl->drain_battery)
  {
    battery_soc -= _pimpl->invariant_battery_drain;
    if (battery_soc <= state_config.threshold_soc())
      return rmf_utils::nullopt;

    // Check if the robot has enough charge to head back to nearest charger
    double retreat_battery_drain = 0.0;
    if ( _pimpl->end_waypoint != state.charging_waypoint())
    {
      const auto endpoints = std::make_pair(_pimpl->end_waypoint,
        state.charging_waypoint());
      const auto& cache_result = estimate_cache->get(endpoints);
      if (cache_result)
      {
        retreat_battery_drain = cache_result->dsoc;
      }
      else
      {
        rmf_traffic::agv::Planner::Start start{
          state.finish_time(),
          endpoints.first,
          0.0};

        rmf_traffic::agv::Planner::Goal goal{endpoints.second};

        const auto result_to_charger = _pimpl->planner->plan(start, goal);
        // We assume we can always compute a plan
        const auto& trajectory =
            result_to_charger->get_itinerary().back().trajectory();
        const auto& finish_time = *trajectory.finish_time();
        const rmf_traffic::Duration retreat_duration =
            finish_time - state.finish_time();

        dSOC_motion = _pimpl->motion_sink->compute_change_in_charge(trajectory);
        dSOC_ambient = _pimpl->ambient_sink->compute_change_in_charge(
            rmf_traffic::time::to_seconds(retreat_duration));
        retreat_battery_drain = dSOC_motion + dSOC_ambient;
        estimate_cache->set(endpoints, retreat_duration, retreat_battery_drain);
      }
    }

    if (battery_soc - retreat_battery_drain <= state_config.threshold_soc())
      return rmf_utils::nullopt;
    
    state.battery_soc(battery_soc);
  }

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration Clean::invariant_duration() const
{
  return _pimpl->invariant_duration;
}

//==============================================================================
rmf_traffic::Time Clean::earliest_start_time() const
{
  return _pimpl->start_time;
}

//==============================================================================
std::size_t Clean::start_waypoint() const
{
  return _pimpl->start_waypoint;
}

//==============================================================================
std::size_t Clean::end_waypoint() const
{
  return _pimpl->end_waypoint;
}

//==============================================================================
rmf_traffic::agv::Planner::Start Clean::location_after_clean(
    rmf_traffic::agv::Planner::Start start) const
{
  if (start.waypoint() == _pimpl->start_waypoint)
    return start;
  
  rmf_traffic::agv::Planner::Goal goal{_pimpl->start_waypoint};

  const auto result = _pimpl->planner->plan(start, goal);
  // We assume we can always compute a plan
  const auto& trajectory =
      result->get_itinerary().back().trajectory();
  const auto& finish_time = *trajectory.finish_time();
  const double orientation = trajectory.back().position()[2];
  
  rmf_traffic::agv::Planner::Start location_after_clean{
    finish_time + _pimpl->invariant_duration,
    _pimpl->start_waypoint,
    orientation};

  return location_after_clean;

}

//==============================================================================
} // namespace requests
} // namespace rmf_task
