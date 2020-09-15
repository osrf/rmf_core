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

#include "Loop.hpp"

#include "../phases/GoToPlace.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
rmf_utils::optional<LoopEstimate> estimate_loop(
    const rmf_task_msgs::msg::Loop& request,
    const std::shared_ptr<agv::FleetUpdateHandle>& fleet)
{
  const auto& fimpl = FleetUpdateHandle::Implementation::get(*fleet);
  if (request.robot_type != fimpl.name)
    return rmf_utils::nullopt;

  const std::size_t n = request.num_loops;
  if (n == 0)
    return rmf_utils::nullopt;

  const auto planner = fimpl.planner;
  const auto& graph = planner->get_configuration().graph();
  const auto loop_start_wp = graph.find_waypoint(request.start_name);
  if (!loop_start_wp)
    return rmf_utils::nullopt;

  const auto loop_end_wp = graph.find_waypoint(request.finish_name);
  if (!loop_end_wp)
    return rmf_utils::nullopt;

  const auto loop_start_goal =
      rmf_traffic::agv::Plan::Goal(loop_start_wp->index());

  const auto loop_end_goal =
      rmf_traffic::agv::Plan::Goal(loop_end_wp->index());

  LoopEstimate best;
  for (const auto& element : fimpl.task_managers)
  {
    LoopEstimate estimate;
    estimate.robot = element.first;

    const auto& mgr = *element.second;
    auto start = mgr.expected_finish_location();
    const auto loop_init_plan = planner->plan(start, loop_start_goal);
    if (!loop_init_plan)
      continue;

    rmf_traffic::Duration init_duration = std::chrono::seconds(0);
    if (loop_init_plan->get_waypoints().size() > 1)
    {
      // If loop_init_plan is not empty, then that means we are not starting at
      // the starting point of the loop. Therefore we will need an initial plan
      // to reach the first point in the loop.
      estimate.init_start = start.front();

      init_duration =
          loop_init_plan->get_waypoints().back().time()
          - loop_init_plan->get_waypoints().front().time();
    }

    const auto loop_forward_start = [&]() -> rmf_traffic::agv::Plan::StartSet
    {
      if (loop_init_plan->get_waypoints().empty())
        return start;

      const auto& loop_init_wp = loop_init_plan->get_waypoints().back();
      assert(loop_init_wp.graph_index());
      return {rmf_traffic::agv::Plan::Start(
            loop_init_wp.time(),
            *loop_init_wp.graph_index(),
            loop_init_wp.position()[2])};
    }();

    const auto loop_forward_plan =
        planner->plan(loop_forward_start, loop_end_goal);
    if (!loop_forward_plan)
      continue;

    // If the forward plan is empty then that means the start and end of the
    // loop are the same, making it a useless request.
    // TODO(MXG): We should probably make noise here instead of just ignoring
    // the request.
    if (loop_forward_plan->get_waypoints().empty())
      return rmf_utils::nullopt;

    estimate.loop_start = loop_forward_start.front();

    const auto loop_duration =
        loop_forward_plan->get_waypoints().back().time()
        - loop_forward_plan->get_waypoints().front().time();

    // We only need to provide this if there is supposed to be more than one
    // loop.
    const auto& final_wp = loop_forward_plan->get_waypoints().back();
    assert(final_wp.graph_index());
    estimate.loop_end = rmf_traffic::agv::Plan::Start{
      final_wp.time(),
      *final_wp.graph_index(),
      final_wp.position()[2]
    };

    const auto start_time = [&]()
    {
      if (loop_init_plan->get_waypoints().empty())
        return loop_forward_plan->get_waypoints().front().time();

      return loop_init_plan->get_waypoints().front().time();
    }();

    estimate.time =
        start_time + init_duration + (2*n - 1)*loop_duration;

    estimate.loop_end->time(estimate.time);

    if (estimate.time < best.time)
      best = std::move(estimate);
  }

  if (best.robot)
    return best;

  return rmf_utils::nullopt;
}

//==============================================================================
std::shared_ptr<Task> make_loop(
    const rmf_task_msgs::msg::Loop& request,
    const agv::RobotContextPtr& context,
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> init_start,
    rmf_traffic::agv::Plan::Start loop_start,
    rmf_utils::optional<rmf_traffic::agv::Plan::Start> loop_end)
{
  const auto& graph = context->navigation_graph();

  Task::PendingPhases phases;

  const auto start_wp =
      graph.find_waypoint(request.start_name)->index();

  const auto end_wp =
      graph.find_waypoint(request.finish_name)->index();

  if (init_start)
  {
    phases.push_back(
          phases::GoToPlace::make(context, *init_start, start_wp));
  }


  phases.push_back(
        phases::GoToPlace::make(context, loop_start, end_wp));

  for (std::size_t i=1; i < request.num_loops; ++i)
  {
    assert(loop_end);

    phases.push_back(
          phases::GoToPlace::make(context, *loop_end, start_wp));

    phases.push_back(
          phases::GoToPlace::make(context, loop_start, end_wp));
  }

  return Task::make(request.task_id, std::move(phases), context->worker());
}

} // namespace tasks
} // naemspace rmf_fleet_adapter
