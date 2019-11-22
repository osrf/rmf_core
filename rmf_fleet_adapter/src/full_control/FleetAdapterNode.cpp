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

#include "../rmf_fleet_adapter/ParseGraph.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rclcpp/executors.hpp>

namespace rmf_fleet_adapter {
namespace full_control {

//==============================================================================
std::shared_ptr<FleetAdapterNode> FleetAdapterNode::make(
    const std::string& fleet_name,
    const std::string& graph_file,
    rmf_traffic::agv::VehicleTraits traits,
    const rmf_traffic::Duration plan_time,
    const rmf_traffic::Duration wait_time)
{
  const auto node = std::shared_ptr<FleetAdapterNode>(
        new FleetAdapterNode(fleet_name, plan_time));

  auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
        *node, rmf_traffic::schedule::query_everything().spacetime());

  auto submit_trajectories = node->create_client<SubmitTrajectories>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName);

  auto delay_trajectories = node->create_client<DelayTrajectories>(
        rmf_traffic_ros2::DelayTrajectoriesSrvName);

  auto replace_trajectories = node->create_client<ReplaceTrajectories>(
        rmf_traffic_ros2::ReplaceTrajectoriesSrvName);

  rmf_traffic::agv::Graph graph;
  std::unordered_map<std::string, std::size_t> waypoint_keys;
  if (!parse_graph(graph_file, traits, *node, graph, waypoint_keys))
    return nullptr;

  using namespace std::chrono_literals;

  const auto stop_time = std::chrono::steady_clock::now() + wait_time;
  while(rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(node);

    bool ready = true;
    ready &= submit_trajectories->service_is_ready();
    ready &= delay_trajectories->service_is_ready();
    ready &= replace_trajectories->service_is_ready();
    ready &= (mirror_future.wait_for(0s) == std::future_status::ready);

    if (ready)
    {
      node->start(
            Fields{
              std::move(graph),
              std::move(traits),
              mirror_future.get(),
              std::move(submit_trajectories),
              std::move(delay_trajectories),
              std::move(replace_trajectories)
            });

      return node;
    }
  }

  RCLCPP_ERROR(
        node->get_logger(),
        "Timeout after waiting ["
        + std::to_string(rmf_traffic::time::to_seconds(wait_time))
        + "] to connect to the schedule");

  return nullptr;
}

//==============================================================================
rmf_traffic::Duration FleetAdapterNode::get_plan_time() const
{
  return _plan_time;
}

//==============================================================================
FleetAdapterNode::FleetAdapterNode(
    const std::string& fleet_name,
    rmf_traffic::Duration plan_time)
: rclcpp::Node(fleet_name + "__full_control_fleet_adapter"),
  _fleet_name(fleet_name),
  _plan_time(plan_time)
{
  // Do nothing
}

//==============================================================================
void FleetAdapterNode::start(Fields fields)
{
  _field = std::move(fields);
  _field->mirror.update();

  _delivery_sub = create_subscription<Delivery>(
        DeliveryTopicName, rclcpp::SystemDefaultsQoS(),
        [&](Delivery::UniquePtr msg)
  {
    this->delivery_request(std::move(msg));
  });
}

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
class MoveAction : public FleetAdapterNode::Action
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

  bool find_plan()
  {
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

    return choose_fallback(std::move(fallback_plans));
  }

  bool execute_main(rmf_traffic::agv::Plan plan)
  {

  }

  bool choose_fallback(
      std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> fallback_plans)
  {
    const auto i_nearest_opt = get_fastest_plan_index(fallback_plans);
    if (!i_nearest_opt)
    {
      _error = "The robot is trapped! Human intervention may be needed!";
      return false;
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

    while (std::chrono::steady_clock::now() < giveup_time
           && !have_resume_plan)
    {
      std::mutex placeholder;
      std::unique_lock<std::mutex> lock(placeholder);
      resume_plan_cv.wait_for(lock, std::chrono::milliseconds(100),
                              [&](){ return have_resume_plan; });
    }

    const auto quickest_finish_opt = get_fastest_plan_index(resume_plans);
    if (!quickest_finish_opt)
    {
      _error = "The robot will be trapped at its fallback point! "
          "Human intervention may be needed!";
    }
    else
    {
      _plans.push(*resume_plans[*quickest_finish_opt]);
    }

    return true;
  }

private:
  FleetAdapterNode* const _node;
  const FleetAdapterNode::RobotState* const _state;
  const std::size_t _goal_wp_index;
  const std::vector<std::size_t>& _fallback_wps;

  std::queue<rmf_traffic::agv::Plan> _plans;
  std::string _error;

};
} // anonymous namespace

//==============================================================================


} // namespace full_control
} // namespace rmf_fleet_adapter
