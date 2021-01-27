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

  std::string id;
  std::size_t start_waypoint;
  std::size_t finish_waypoint;
  std::size_t num_loops;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  rmf_traffic::Time start_time;
  bool drain_battery;
  bool priority;

  rmf_traffic::Duration invariant_duration;
  double invariant_battery_drain;
};

//==============================================================================
ConstRequestPtr Loop::make(
  std::string id,
  std::size_t start_waypoint,
  std::size_t finish_waypoint,
  std::size_t num_loops,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  rmf_traffic::Time start_time,
  bool drain_battery,
  bool priority)
{
  std::shared_ptr<Loop> loop(new Loop());
  loop->_pimpl->id = id;
  loop->_pimpl->start_waypoint = start_waypoint;
  loop->_pimpl->finish_waypoint = finish_waypoint;
  loop->_pimpl->num_loops = num_loops;
  loop->_pimpl->motion_sink = std::move(motion_sink);
  loop->_pimpl->ambient_sink = std::move(ambient_sink);
  loop->_pimpl->planner = std::move(planner);
  loop->_pimpl->start_time = start_time;
  loop->_pimpl->drain_battery = drain_battery;
  loop->_pimpl->priority = priority;

  // Calculate the invariant duration and battery drain for this task
  loop->_pimpl->invariant_duration = rmf_traffic::Duration{0};
  loop->_pimpl->invariant_battery_drain = 0.0;
  if (loop->_pimpl->start_waypoint != loop->_pimpl->finish_waypoint)
  {
    rmf_traffic::agv::Planner::Start loop_start{
      start_time,
      loop->_pimpl->start_waypoint,
      0.0};
    rmf_traffic::agv::Planner::Goal loop_end_goal{
      loop->_pimpl->finish_waypoint};

    const auto forward_loop_plan = loop->_pimpl->planner->plan(
      loop_start, loop_end_goal);

    auto itinerary_start_time = start_time;
    double forward_battery_drain = 0.0;
    rmf_traffic::Duration forward_duration(0);
    for (const auto& itinerary : forward_loop_plan->get_itinerary())
    {
      const auto& trajectory = itinerary.trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const auto itinerary_duration = finish_time - itinerary_start_time;

      if (loop->_pimpl->drain_battery)
      {
        // Compute battery drain
        const double dSOC_motion =
          loop->_pimpl->motion_sink->compute_change_in_charge(trajectory);
        const double dSOC_device =
          loop->_pimpl->ambient_sink->compute_change_in_charge(
            rmf_traffic::time::to_seconds(itinerary_duration));
        forward_battery_drain += dSOC_motion + dSOC_device;
      }

      forward_duration += itinerary_duration;
      itinerary_start_time = finish_time;
    }
    loop->_pimpl->invariant_duration =
      (2 * num_loops - 1) * forward_duration;
    loop->_pimpl->invariant_battery_drain =
      (2 * num_loops - 1) * forward_battery_drain;
  }

  return loop;

}

//==============================================================================
Loop::Loop()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
std::string Loop::id() const
{
  return _pimpl->id;
}

//==============================================================================
bool Loop::priority() const
{
  return _pimpl->priority;
}

//==============================================================================
rmf_utils::optional<rmf_task::Estimate> Loop::estimate_finish(
  const agv::State& initial_state,
  const agv::Constraints& task_planning_constraints,
  const std::shared_ptr<EstimateCache> estimate_cache) const
{

  rmf_traffic::Duration variant_duration(0);

  const rmf_traffic::Time start_time = initial_state.finish_time();
  double battery_soc = initial_state.battery_soc();
  double dSOC_motion = 0.0;
  double dSOC_device = 0.0;

  // Check if a plan has to be generated from finish location to start_waypoint
  if (initial_state.waypoint() != _pimpl->start_waypoint)
  {
    auto endpoints = std::make_pair(initial_state.waypoint(),
      _pimpl->start_waypoint);
    const auto& cache_result = estimate_cache->get(endpoints);
    // Use previously memoized values if possible
    if (cache_result)
    {
      variant_duration = cache_result->duration;
      if (_pimpl->drain_battery)
        battery_soc = battery_soc - cache_result->dsoc;
    }
    else
    {
      // Compute plan to start_waypoint along with battery drain
      rmf_traffic::agv::Planner::Goal loop_start_goal{endpoints.second};
      const auto plan_to_start = _pimpl->planner->plan(
        initial_state.location(), loop_start_goal);
      // We assume we can always compute a plan
      auto itinerary_start_time = start_time;
      double variant_battery_drain = 0.0;
      for (const auto& itinerary : plan_to_start->get_itinerary())
      {
        const auto& trajectory = itinerary.trajectory();
        const auto& finish_time = *trajectory.finish_time();
        const rmf_traffic::Duration itinerary_duration =
         finish_time - itinerary_start_time;

        if (_pimpl->drain_battery)
        {
          // Compute battery drain
          dSOC_motion = _pimpl->motion_sink->compute_change_in_charge(trajectory);
          dSOC_device =
            _pimpl->ambient_sink->compute_change_in_charge(
              rmf_traffic::time::to_seconds(itinerary_duration));
          battery_soc = battery_soc - dSOC_motion - dSOC_device;
          variant_battery_drain += dSOC_motion + dSOC_device;
        }
        itinerary_start_time = finish_time;
        variant_duration += itinerary_duration;
      }
      estimate_cache->set(endpoints, variant_duration,
        variant_battery_drain);
    }

    if (battery_soc <= task_planning_constraints.threshold_soc())
      return rmf_utils::nullopt;
  }

  // Compute wait_until
  const rmf_traffic::Time ideal_start = _pimpl->start_time - variant_duration;
  const rmf_traffic::Time wait_until =
    initial_state.finish_time() > ideal_start ?
    initial_state.finish_time() : ideal_start;

  // Factor in battery drain while waiting to move to start waypoint. If a robot
  // is initially at a charging waypoint, it is assumed to be continually charging
  if (_pimpl->drain_battery && wait_until > initial_state.finish_time() &&
    initial_state.waypoint() != initial_state.charging_waypoint())
  {
    rmf_traffic::Duration wait_duration(wait_until - initial_state.finish_time());
    dSOC_device = _pimpl->ambient_sink->compute_change_in_charge(
      rmf_traffic::time::to_seconds(wait_duration));
    battery_soc = battery_soc - dSOC_device;

    if (battery_soc <= task_planning_constraints.threshold_soc())
    {
      return rmf_utils::nullopt;
    }
  }

  // Compute finish time
  const rmf_traffic::Time state_finish_time =
    wait_until + variant_duration  + _pimpl->invariant_duration;

  // Subtract invariant battery drain and check if robot can return to its charger
  double retreat_battery_drain = 0.0;
  if (_pimpl->drain_battery)
  {
    battery_soc -= _pimpl->invariant_battery_drain;
    if (battery_soc <= task_planning_constraints.threshold_soc())
      return rmf_utils::nullopt;

    if ( _pimpl->finish_waypoint != initial_state.charging_waypoint())
    {
      const auto endpoints = std::make_pair(_pimpl->finish_waypoint,
        initial_state.charging_waypoint());
      const auto& cache_result = estimate_cache->get(endpoints);
      if (cache_result)
      {
        retreat_battery_drain = cache_result->dsoc;
      }
      else
      {
        rmf_traffic::agv::Planner::Start retreat_start{
          state_finish_time,
          endpoints.first,
          0.0};

        rmf_traffic::agv::Planner::Goal charger_goal{
          endpoints.second};

        const auto result_to_charger = _pimpl->planner->plan(
          retreat_start, charger_goal);
        // We assume we can always compute a plan
        auto itinerary_start_time = state_finish_time;
        rmf_traffic::Duration retreat_duration(0);
        for (const auto& itinerary : result_to_charger->get_itinerary())
        {
          const auto& trajectory = itinerary.trajectory();
          const auto& finish_time = *trajectory.finish_time();
          const rmf_traffic::Duration itinerary_duration =
            finish_time - itinerary_start_time;

          dSOC_motion = _pimpl->motion_sink->compute_change_in_charge(
            trajectory);
          dSOC_device = _pimpl->ambient_sink->compute_change_in_charge(
              rmf_traffic::time::to_seconds(itinerary_duration));
          retreat_battery_drain += dSOC_motion + dSOC_device;

          itinerary_start_time = finish_time;
          retreat_duration += itinerary_duration;
        }
        estimate_cache->set(endpoints, retreat_duration,
          retreat_battery_drain);
      }

      if (battery_soc - retreat_battery_drain <= task_planning_constraints.threshold_soc())
        return rmf_utils::nullopt;
    }
  }

  // Return Estimate
  rmf_traffic::agv::Planner::Start location{
    state_finish_time,
    _pimpl->finish_waypoint,
    initial_state.location().orientation()};
  agv::State state{
    std::move(location),
    initial_state.charging_waypoint(),
    battery_soc};

  return Estimate(state, wait_until);
}

//==============================================================================
rmf_traffic::Duration Loop::invariant_duration() const
{
  return _pimpl->invariant_duration;
}

//==============================================================================
rmf_traffic::Time Loop::earliest_start_time() const
{
  return _pimpl->start_time;
}

//==============================================================================
std::size_t Loop::start_waypoint() const
{
  return _pimpl->start_waypoint;
}

//==============================================================================
std::size_t Loop::finish_waypoint() const
{
  return _pimpl->finish_waypoint;
}

//==============================================================================
std::size_t Loop::num_loops() const
{
  return _pimpl->num_loops;
}

//==============================================================================
Loop::Start Loop::loop_start(const Loop::Start& start) const
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

  rmf_traffic::agv::Planner::Start loop_start{
    finish_time,
    _pimpl->start_waypoint,
    orientation};

  return loop_start;
}

//==============================================================================
Loop::Start Loop::loop_end(const Loop::Start& start) const
{
  if (start.waypoint() == _pimpl->finish_waypoint)
    return start;

  rmf_traffic::agv::Planner::Goal goal{_pimpl->finish_waypoint};

  const auto result = _pimpl->planner->plan(start, goal);
  // We assume we can always compute a plan
  const auto& trajectory =
      result->get_itinerary().back().trajectory();
  const auto& finish_time = *trajectory.finish_time();
  const double orientation = trajectory.back().position()[2];

  rmf_traffic::agv::Planner::Start loop_end{
    finish_time,
    _pimpl->finish_waypoint,
    orientation};

  return loop_end;
}

} // namespace requests
} // namespace rmf_task
