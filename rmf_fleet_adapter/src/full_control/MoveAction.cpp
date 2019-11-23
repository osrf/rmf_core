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

#include "FleetAdapterNode.hpp"

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

namespace rmf_fleet_adapter {
namespace full_control {

namespace {
//==============================================================================
rmf_utils::optional<std::size_t> get_fastest_plan_index(
    const std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>>& plans)
{
  auto nearest = std::chrono::steady_clock::time_point::max();
  std::size_t i_nearest = std::numeric_limits<std::size_t>::max();
  for (std::size_t i=0; i < plans.size(); ++i)
  {
    const auto& plan = plans[i];
    if (plan)
    {
      assert(plan->get_trajectories().back().finish_time());
      const auto finish_time = *plan->get_trajectories().back().finish_time();
      if (finish_time < nearest)
      {
        nearest = finish_time;
        i_nearest = i;
      }
    }
  }

  if (i_nearest == std::numeric_limits<std::size_t>::max())
  {
    return rmf_utils::nullopt;
  }

  return i_nearest;
}

//==============================================================================
class MoveAction : public Action
{
public:

  MoveAction(
      FleetAdapterNode* node,
      const FleetAdapterNode::RobotState* state,
      const std::size_t goal_wp_index,
      const std::vector<std::size_t>& fallback_wps)
  : _node(node),
    _state(state),
    _goal_wp_index(goal_wp_index),
    _fallback_wps(fallback_wps)
  {
    // Do nothing
  }

  void find_plan()
  {
    _plans.clear();

    const auto& planner = _node->get_planner();

    // Add 3 seconds to the current time to give us some buffer
    const auto now = rmf_traffic_ros2::convert(_node->get_clock()->now())
        + std::chrono::seconds(3);

    const auto start_wp_index = _node->compute_closest_wp(_state->location);
    const double start_yaw = static_cast<double>(_state->location.yaw);

    bool interrupt_flag = false;
    auto options = planner.get_default_options();
    options.interrupt_flag(&interrupt_flag);

    bool main_plan_finished = false;
    std::condition_variable main_plan_cv;
    rmf_utils::optional<rmf_traffic::agv::Plan> main_plan;
    std::thread main_plan_thread = std::thread(
          [&]()
    {
      main_plan = planner.plan(
            rmf_traffic::agv::Plan::Start(
              now, start_wp_index, start_yaw),
            rmf_traffic::agv::Plan::Goal(_goal_wp_index),
            options);
      main_plan_finished = true;
      main_plan_cv.notify_all();
    });

    std::vector<std::thread> fallback_plan_threads;
    std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> fallback_plans;
    std::mutex fallback_plan_mutex;
    for (const std::size_t goal_wp : _fallback_wps)
    {
      fallback_plan_threads.emplace_back(std::thread([&]()
      {
        auto fallback_plan = planner.plan(
              rmf_traffic::agv::Plan::Start(
                now, start_wp_index, start_yaw),
              rmf_traffic::agv::Plan::Goal(goal_wp),
              options);

        std::unique_lock<std::mutex> lock(fallback_plan_mutex);
        fallback_plans.emplace_back(std::move(fallback_plan));
      }));
    }

    const auto giveup_time =
        std::chrono::steady_clock::now() + _node->get_plan_time();

    // Waiting for the main planning thread is a bit complicated, because we
    // want to avoid the possibility that the plan finishes and triggers the
    // condition variable before we check it.
    while (std::chrono::steady_clock::now() < giveup_time
           && !main_plan_finished)
    {
      std::mutex placeholder;
      std::unique_lock<std::mutex> lock(placeholder);
      main_plan_cv.wait_for(lock, std::chrono::milliseconds(100),
                            [&](){ return main_plan_finished; });
    }

    interrupt_flag = true;
    main_plan_thread.join();
    for (auto& fallback_thread : fallback_plan_threads)
      fallback_thread.join();

    if (main_plan)
      return execute_main(std::move(*std::move(main_plan)));

    return execute_fallback(std::move(fallback_plans));
  }

  void execute_main(rmf_traffic::agv::Plan plan)
  {
    _plans.emplace_back(std::move(plan));
    return execute_plan();
  }

  void execute_fallback(
      std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> fallback_plans)
  {
    const auto i_nearest_opt = get_fastest_plan_index(fallback_plans);
    if (!i_nearest_opt)
    {
      return _state->task->critical_failure(
            "The robot is trapped! Human intervention may be needed!");
    }
    const auto i_nearest = *i_nearest_opt;

    const auto& fallback_plan = *fallback_plans[i_nearest];
    const std::size_t fallback_waypoint =
        fallback_plan.get_waypoints().back().graph_index();
    const double fallback_orientation =
        fallback_plan.get_waypoints().back().position()[2];
    const auto fallback_end_time =
        *fallback_plan.get_trajectories().back().finish_time();

    const auto& planner = _node->get_planner();

    bool interrupt_flag = false;
    auto options = planner.get_default_options();
    options.interrupt_flag(&interrupt_flag);

    const auto t_spread = std::chrono::seconds(15);
    bool have_resume_plan = false;
    std::vector<std::thread> resume_plan_threads;
    std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> resume_plans;
    std::mutex resume_plan_mutex;
    std::condition_variable resume_plan_cv;
    for (std::size_t i=1; i < 9; ++i)
    {
      resume_plan_threads.emplace_back(std::thread([&]()
      {
        const auto resume_time = fallback_end_time + i*t_spread;
        auto resume_plan = planner.plan(
              rmf_traffic::agv::Plan::Start(
                resume_time, fallback_waypoint, fallback_orientation),
              rmf_traffic::agv::Plan::Goal(
                _goal_wp_index),
              options);

        std::unique_lock<std::mutex> lock(resume_plan_mutex);
        if (resume_plan)
          have_resume_plan = true;
        resume_plans.emplace_back(std::move(resume_plan));
        resume_plan_cv.notify_all();
      }));
    }

    const auto giveup_time =
        std::chrono::steady_clock::now() + _node->get_plan_time();

    while (std::chrono::steady_clock::now() < giveup_time && !have_resume_plan)
    {
      std::mutex placeholder;
      std::unique_lock<std::mutex> lock(placeholder);
      resume_plan_cv.wait_for(lock, std::chrono::milliseconds(100),
                              [&](){ return have_resume_plan; });
    }

    const auto quickest_finish_opt = get_fastest_plan_index(resume_plans);
    if (!quickest_finish_opt)
    {
      return _state->task->critical_failure(
            "The robot will be trapped at its fallback point! "
            "Human intervention may be needed!");
    }
    else
    {
      _plans.emplace_back(*resume_plans[*quickest_finish_opt]);
    }

    return execute_plan();
  }

  void execute_plan()
  {
    const auto& submit = _node->get_fields().submit_trajectories;

    rmf_traffic_msgs::srv::SubmitTrajectories::Request request;

    for (const auto& plan : _plans)
    {
      for (const auto& trajectory : plan.get_trajectories())
      {
        request.trajectories.emplace_back(
              rmf_traffic_ros2::convert(trajectory));
      }
    }

    request.fleet.fleet_id = _node->get_fleet_name();
    request.fleet.type =
        rmf_traffic_msgs::msg::FleetProperties::TYPE_RESPONSIVE;

    submit->async_send_request(
          std::make_shared<rmf_traffic_msgs::srv::SubmitTrajectories::Request>(
            std::move(request)));

  }

private:
  FleetAdapterNode* const _node;
  const FleetAdapterNode::RobotState* const _state;
  const std::size_t _goal_wp_index;
  const std::vector<std::size_t>& _fallback_wps;

  std::vector<rmf_traffic::agv::Plan> _plans;

};
} // anonymous namespace

//==============================================================================
std::unique_ptr<Action> make_move(
    FleetAdapterNode* node,
    const FleetAdapterNode::RobotState* state,
    const std::size_t goal_wp_index,
    const std::vector<std::size_t>& fallback_wps)
{
  return std::make_unique<MoveAction>(
        node, state, goal_wp_index, fallback_wps);
}

} // namespace full_control
} // namespace rmf_fleet_adapter
