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
      FleetAdapterNode::RobotContext* state,
      const std::size_t goal_wp_index,
      const std::vector<std::size_t>& fallback_wps)
  : _node(node),
    _context(state),
    _goal_wp_index(goal_wp_index),
    _fallback_wps(fallback_wps),
    _state_listener(this),
    _waiting_for_schedule(true)
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

    const auto start_wp_index = _node->compute_closest_wp(_context->location);
    const double start_yaw = static_cast<double>(_context->location.yaw);

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
      return _context->task->critical_failure(
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
      return _context->task->critical_failure(
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
    std::vector<rmf_traffic_msgs::msg::Trajectory> trajectories;

    for (const auto& plan : _plans)
    {
      for (const auto& trajectory : plan.get_trajectories())
      {
        assert(trajectory.size() > 0);

        // If the trajectory has only one point then the robot doesn't need to
        // go anywhere.
        if (trajectory.size() < 2)
          continue;

        trajectories.emplace_back(
              rmf_traffic_ros2::convert(trajectory));
      }
    }

    _waiting_for_schedule = true;
    _context->listeners.insert(&_state_listener);

    if (!_schedule_ids.empty())
    {
      replace_plan(std::move(trajectories));
      return;
    }

    submit_plan(std::move(trajectories));
  }

  void replace_plan(std::vector<rmf_traffic_msgs::msg::Trajectory> trajectories)
  {
    using ReplaceTrajectories = rmf_traffic_msgs::srv::ReplaceTrajectories;

    const auto& replace = _node->get_fields().replace_trajectories;
    ReplaceTrajectories::Request request;

    request.replace_ids = _schedule_ids;
    request.trajectories = std::move(trajectories);

    _schedule_ids.clear();

    replace->async_send_request(
          std::make_shared<ReplaceTrajectories::Request>(
            std::move(request)),
          [&](rclcpp::Client<ReplaceTrajectories>::SharedFuture future)
    {
      const auto response = future.get();

      // We should never get an exceptional error message here. If we do, then
      // there is a serious bug in our MoveAction or the schedule service.
      assert(response->error.empty());

      this->_waiting_for_schedule = false;

      for (auto i = response->original_version+1;
           i <= response->latest_trajectory_version; ++i)
      {
        _schedule_ids.push_back(i);
      }

      command_plan();
    });
  }

  void submit_plan(std::vector<rmf_traffic_msgs::msg::Trajectory> trajectories)
  {
    using SubmitTrajectories = rmf_traffic_msgs::srv::SubmitTrajectories;

    const auto& submit = _node->get_fields().submit_trajectories;
    SubmitTrajectories::Request request;

    request.fleet.fleet_id = _node->get_fleet_name();
    request.fleet.type =
        rmf_traffic_msgs::msg::FleetProperties::TYPE_RESPONSIVE;
    request.trajectories = std::move(trajectories);

    submit->async_send_request(
          std::make_shared<SubmitTrajectories::Request>(
            std::move(request)),
          [&](rclcpp::Client<SubmitTrajectories>::SharedFuture future)
    {
      const auto response = future.get();

      // We should never get an exceptional error message here. If we do, then
      // there is a serious bug in our MoveAction or the schedule service.
      assert(response->error.empty());

      if (response->accepted)
      {
        this->_waiting_for_schedule = false;
        for (auto i = response->original_version+1;
             i <= response->current_version; ++ i)
        {
          _schedule_ids.push_back(i);
        }

        command_plan();
        return;
      }

      // The plan was rejected, so we will have to try to replan
      RCLCPP_WARN(
            _node->get_logger(),
            "The traffic plan for [" + _context->task->id() + "] was rejected. "
            + "We will retry.");

      find_plan();
    });
  }

  void command_plan()
  {
    _waypoints.clear();
    for (const auto& plan : _plans)
      for (const auto& wp : plan.get_waypoints())
        _waypoints.emplace_back(wp);

    _command_segment = 0;
    send_next_command();
  }

  void send_next_command()
  {
    _command.fleet_name = _node->get_fleet_name();
    _command.task_id =
        _context->task->id() + std::to_string(_command_segment++);

    _command.path.clear();

    const auto& graph = _node->get_graph();

    std::size_t i=0;
    for (; i < _waypoints.size(); ++i)
    {
      const auto& wp = _waypoints[i];

      // Break off the command wherever an event occurs, because those are
      // points where the fleet adapter will need to issue door/lift requests.
      if (wp.event())
        break;

      const auto& p = wp.position();

      rmf_fleet_msgs::msg::Location location;
      location.t = rmf_traffic_ros2::convert(wp.time());
      location.x = p[0];
      location.y = p[1];
      location.yaw = p[2];

      location.level_name =
          graph.get_waypoint(wp.graph_index()).get_map_name();

      _command.path.emplace_back(std::move(location));
    }

    _waypoints.erase(_waypoints.begin(), _waypoints.begin()+i);

    resend_last_command();
    _command_time = _node->get_clock()->now();
  }

  void resend_last_command()
  {
    _node->path_request_publisher->publish(_command);
  }

  virtual ~MoveAction();

  using RobotState = rmf_fleet_msgs::msg::RobotState;
  class StateListener : public Listener<RobotState>
  {
  public:

    StateListener(MoveAction* parent)
    : _parent(parent)
    {
      // Do nothing
    }

    void receive(const RobotState& msg) final
    {
      if (_parent->_waiting_for_schedule)
        return;

      if (msg.task_id != _parent->_command.task_id)
      {
        const auto now = _parent->_node->get_clock()->now();

        // Recompute a plan if the fleet driver has a huge delay.
        if (now - _parent->_command_time > std::chrono::seconds(5))
        {
          RCLCPP_ERROR(
                _parent->_node->get_logger(),
                "The fleet driver is being unresponsive to task plan ["
                + _parent->_context->task->id() + "]. Recomputing plan!");

          _parent->find_plan();
          return;
        }

        // Attempt to resend the command if the robot has not received it yet.
        if (now - _parent->_command_time > std::chrono::milliseconds(100))
          _parent->resend_last_command();

        return;
      }

      if (msg.path.size() <= 1)
        _parent->handle_event(msg);

      _parent->handle_delay(msg);
    }

    MoveAction* _parent;
  };

  void handle_event(const RobotState& msg)
  {
    const auto& event_wp = _waypoints.front();
    if (!event_wp.event())
      return;

    // Check if we're already handling the event
    if (_event_executor.is_active())
      return;

    const auto& l = msg.location;
    const Eigen::Vector2d p{l.x, l.y};
    const Eigen::Vector2d target = event_wp.position().block<2,1>(0,0);

    // TODO(MXG): Consider making this threshold configurable
    if ((p - target).norm() < 0.1)
      event_wp.event()->execute(_event_executor);
  }

  void handle_delay(const RobotState& msg)
  {

  }

  class EventExecutor : public rmf_traffic::agv::Graph::Lane::Executor
  {
  public:

    EventExecutor(MoveAction* parent)
    : _parent(parent),
      _is_active(false)
    {
      // Do nothing
    }

    bool is_active() const
    {
      return _is_active;
    }

  private:
    MoveAction* const _parent;
    bool _is_active;
  };

private:
  FleetAdapterNode* const _node;
  FleetAdapterNode::RobotContext* const _context;
  const std::size_t _goal_wp_index;
  const std::vector<std::size_t>& _fallback_wps;
  StateListener _state_listener;
  EventExecutor _event_executor;

  bool _waiting_for_schedule;
  rclcpp::Time _command_time;
  std::vector<rmf_traffic::agv::Plan> _plans;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
  rmf_fleet_msgs::msg::PathRequest _command;
  std::size_t _command_segment;
  std::vector<rmf_traffic::schedule::Version> _schedule_ids;

};

MoveAction::~MoveAction()
{
  _context->listeners.erase(&_state_listener);
}

} // anonymous namespace

//==============================================================================
std::unique_ptr<Action> make_move(
    FleetAdapterNode* node,
    const FleetAdapterNode::RobotContext* state,
    const std::size_t goal_wp_index,
    const std::vector<std::size_t>& fallback_wps)
{
  return std::make_unique<MoveAction>(
        node, state, goal_wp_index, fallback_wps);
}

} // namespace full_control
} // namespace rmf_fleet_adapter
