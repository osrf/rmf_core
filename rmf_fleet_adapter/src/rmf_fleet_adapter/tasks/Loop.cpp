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
std::shared_ptr<Task> make_loop(
    const rmf_task::requests::ConstLoopRequestPtr request,
    const agv::RobotContextPtr& context,
    const rmf_traffic::agv::Plan::Start start,
    const rmf_traffic::Time deployment_time,
    const rmf_task::agv::State finish_state)
{

  Task::PendingPhases phases;
  const auto loop_start = request->loop_start(start);
  const auto loop_end = request->loop_end(loop_start);

  phases.push_back(
    phases::GoToPlace::make(
      context, std::move(start), request->start_waypoint()));

  phases.push_back(
    phases::GoToPlace::make(
      context, loop_start, request->finish_waypoint()));

  for (std::size_t i = 1; i < request->num_loops(); ++i)
  {
    phases.push_back(
      phases::GoToPlace::make(
        context, loop_end, request->start_waypoint()));

    phases.push_back(
      phases::GoToPlace::make(
        context, loop_start, request->finish_waypoint()));
  }

  return Task::make(
    std::to_string(request->id()),
    std::move(phases),
    context->worker(),
    deployment_time,
    finish_state,
    request);
}

// //==============================================================================
// std::shared_ptr<Task> make_loop(
//     const rmf_task_msgs::msg::Loop& request,
//     const agv::RobotContextPtr& context,
//     rmf_utils::optional<rmf_traffic::agv::Plan::Start> init_start,
//     rmf_traffic::agv::Plan::Start loop_start,
//     rmf_utils::optional<rmf_traffic::agv::Plan::Start> loop_end)
// {
//   const auto& graph = context->navigation_graph();

//   Task::PendingPhases phases;

//   const auto start_wp =
//       graph.find_waypoint(request.start_name)->index();

//   const auto end_wp =
//       graph.find_waypoint(request.finish_name)->index();

//   if (init_start)
//   {
//     phases.push_back(
//           phases::GoToPlace::make(context, *init_start, start_wp));
//   }


//   phases.push_back(
//         phases::GoToPlace::make(context, loop_start, end_wp));

//   for (std::size_t i=1; i < request.num_loops; ++i)
//   {
//     assert(loop_end);

//     phases.push_back(
//           phases::GoToPlace::make(context, *loop_end, start_wp));

//     phases.push_back(
//           phases::GoToPlace::make(context, loop_start, end_wp));
//   }

//   return Task::make(request.task_id, std::move(phases), context->worker());
// }

} // namespace tasks
} // namespace rmf_fleet_adapter
