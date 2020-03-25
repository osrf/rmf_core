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

#include <rmf_utils/math.hpp>

#include "../rmf_fleet_adapter/make_trajectory.hpp"

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
      const auto finish_time = plan->get_itinerary().back().trajectory().finish_time();
      if (!finish_time)
      {
        // If this is an empty trajectory, then the robot is already sitting on
        // its destination, making this undoubtedly the fastest plan.
        assert(!plan->get_waypoints().empty());
        assert(plan->get_waypoints().front().graph_index());
        assert(plan->get_waypoints().size() == 1);
        assert(*plan->get_waypoints().front().graph_index()
               == *plan->get_waypoints().back().graph_index());
        return i;
      }

      if (*finish_time < nearest)
      {
        nearest = *finish_time;
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
      Task* parent,
      FleetAdapterNode::RobotContext* state,
      const std::size_t goal_wp_index,
      const std::size_t move_id)
  : _node(node),
    _task(parent),
    _context(state),
    _goal_wp_index(goal_wp_index),
    _base_task_id(parent->id() + ":move#" + std::to_string(move_id) + ":"),
    _fallback_wps(node->get_parking_spots()),
    _state_listener(this),
    _event_executor(this),
    _event_listener(this),
    _emergency_active(false),
    _waiting_on_emergency(false),
    _planner_options(nullptr),
    _validator(
      rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
        node->get_fields().mirror.viewer(),
        state->schedule.participant_id(),
        state->schedule.description().profile())),
    _handle(std::make_shared<int>(0))
  {
    // Do nothing
  }

  std::vector<rmf_traffic::agv::Plan> find_plan(
      const std::chrono::nanoseconds start_delay)
  {
    return find_plan(start_delay, _validator);
  }

  std::vector<rmf_traffic::agv::Plan> find_plan(
      const std::chrono::nanoseconds start_delay,
      const rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator)
  {
    _emergency_active = false;
    _waiting_on_emergency = false;
    std::vector<rmf_traffic::agv::Plan> plans;

    const auto& planner = _node->get_planner();

    Eigen::Vector3d pose = 
        {_context->location.x, _context->location.y, _context->location.yaw};
    const auto start_time = 
        rmf_traffic_ros2::convert(_node->get_clock()->now()) + start_delay;

    // TODO: further parameterize waypoint and lane merging distance
    const auto plan_starts = 
        rmf_traffic::agv::compute_plan_starts(
            planner.get_configuration().graph(), pose, start_time, 0.1, 1.0, 
            1e-8);

    if (plan_starts.empty())
    {
      RCLCPP_WARN(
          _node->get_logger(), 
          "The robot appears to be in an unrecoverable state, failed to find "
          "suitable waypoints on the graph to start planning.");
      return {};
    }

    bool interrupt_flag = false;
    _planner_options.interrupt_flag(&interrupt_flag);
    _planner_options.validator(std::move(validator));

    bool main_plan_solved = false;
    bool main_plan_failed = false;
    bool fallback_plan_solved = false;
    std::condition_variable plan_solved_cv;
    rmf_utils::optional<rmf_traffic::agv::Plan> main_plan;
    std::thread main_plan_thread = std::thread(
          [&]()
    {
      main_plan =
          planner.plan(
            plan_starts,
            rmf_traffic::agv::Plan::Goal(_goal_wp_index),
            _planner_options);

      if (main_plan)
      {
        main_plan_solved = true;
        plan_solved_cv.notify_all();
      }
      else
      {
        main_plan_failed = true;
      }
    });

    std::vector<std::thread> fallback_plan_threads;
    std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> fallback_plans;
    std::mutex fallback_plan_mutex;
    for (const std::size_t goal_wp : _fallback_wps)
    {
      fallback_plan_threads.emplace_back(std::thread([&, goal_wp]()
      {
        auto fallback_plan = 
            planner.plan(
              plan_starts,
              rmf_traffic::agv::Plan::Goal(goal_wp),
              _planner_options);

        std::unique_lock<std::mutex> lock(fallback_plan_mutex);
        if (fallback_plan)
        {
          fallback_plan_solved = true;
          plan_solved_cv.notify_all();
        }
        fallback_plans.emplace_back(std::move(fallback_plan));
      }));
    }

    const auto giveup_time =
        std::chrono::steady_clock::now() + _node->get_plan_time();

    const auto done_searching = [&]() -> bool
    {
      return main_plan_solved || (main_plan_failed && fallback_plan_solved);
    };

    // Waiting for the main planning thread is a bit complicated, because we
    // want to avoid the possibility that the plan finishes and triggers the
    // condition variable before we check it.
    while (std::chrono::steady_clock::now() < giveup_time && !done_searching())
    {
      std::mutex placeholder;
      std::unique_lock<std::mutex> lock(placeholder);
      plan_solved_cv.wait_for(
            lock, std::chrono::milliseconds(100),
            [&](){ return done_searching(); });
    }

    interrupt_flag = true;
    main_plan_thread.join();
    for (auto& fallback_thread : fallback_plan_threads)
      fallback_thread.join();

    if (main_plan)
    {
      plans.emplace_back(std::move(*std::move(main_plan)));
      return plans;
    }

    return use_fallback(std::move(fallback_plans));
  }

  void find_and_execute_plan(const std::chrono::nanoseconds start_delay)
  {
    auto plans = find_plan(start_delay);
    if (!plans.empty())
      return execute_plan(std::move(plans));
    cancel(std::chrono::seconds(1));
  }

  void respond(
      rmf_traffic::schedule::Negotiation::ConstTablePtr table,
      const Responder& responder,
      const bool* interrupt_flag) final
  {
    // TODO(MXG): Do something with the interrupt flag
    std::vector<rmf_traffic::agv::Plan> plans =
        find_plan(
          std::chrono::seconds(0),
          rmf_utils::make_clone<rmf_traffic::agv::NegotiatingRouteValidator>(
            *table, _context->schedule.description().profile()));

    if (plans.empty())
    {
      responder.reject();
      return;
    }

    auto itinerary = collect_routes(plans);
    std::weak_ptr<void> weak_handle = _handle;
    responder.submit(
          std::move(itinerary),
          [this, table, plans = std::move(plans),
           weak_handle = std::move(weak_handle)]()
    {
      auto handle = weak_handle.lock();
      if (!handle)
        return;

      execute_plan(std::move(plans));
    });
  }

  std::vector<rmf_traffic::agv::Plan> use_fallback(
      std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> fallback_plans)
  {
    std::vector<rmf_traffic::agv::Plan> plans;

    const auto i_nearest_opt = get_fastest_plan_index(fallback_plans);
    if (!i_nearest_opt)
    {
      RCLCPP_WARN(
            _node->get_logger(),
            "Robot [" + _context->robot_name() + "] is stuck! We will try to "
            "find a path again soon.");

      return plans;
    }
    const auto i_nearest = *i_nearest_opt;

    const auto& fallback_plan = *fallback_plans[i_nearest];
    const std::size_t fallback_waypoint =
        *fallback_plan.get_waypoints().back().graph_index();
    const double fallback_orientation =
        fallback_plan.get_waypoints().back().position()[2];
    const auto fallback_end_time =
        fallback_plan.get_waypoints().back().time();

    const auto& planner = _node->get_planner();

    bool interrupt_flag = false;
    _planner_options.interrupt_flag(&interrupt_flag);

    const auto t_spread = std::chrono::seconds(15);
    bool have_resume_plan = false;
    std::vector<std::thread> resume_plan_threads;
    std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> resume_plans;
    std::mutex resume_plan_mutex;
    std::condition_variable resume_plan_cv;

    for (std::size_t i=1; i < 9; ++i)
    {
      resume_plan_threads.emplace_back(std::thread([&, i]()
      {
        const auto resume_time = fallback_end_time + i*t_spread;
        auto resume_plan = planner.plan(
              rmf_traffic::agv::Plan::Start(
                resume_time, fallback_waypoint, fallback_orientation),
              rmf_traffic::agv::Plan::Goal(
                _goal_wp_index),
              _planner_options);

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

    interrupt_flag = true;
    for (auto& resume_plan_thread : resume_plan_threads)
      resume_plan_thread.join();

    const auto quickest_finish_opt = get_fastest_plan_index(resume_plans);
    if (!quickest_finish_opt)
    {
      RCLCPP_WARN(
            _node->get_logger(),
            "Robot [" + _context->robot_name() + "] is stuck! We will try to "
            "find a path again soon.");

      return plans;
    }
    else
    {
      plans.emplace_back(fallback_plan);
      plans.emplace_back(*resume_plans[*quickest_finish_opt]);
    }

    return plans;
  }

  std::vector<rmf_traffic::Route> collect_routes(
      std::vector<rmf_traffic::agv::Plan> plans,
      std::chrono::nanoseconds delay = std::chrono::seconds(0)) const
  {
    std::vector<rmf_traffic::Route> routes;
    bool first_trajectory = true;
    for (const auto& plan : plans)
    {
      for (auto r : plan.get_itinerary())
      {
        if (first_trajectory && r.trajectory().size() > 0)
        {
          first_trajectory = false;
          const auto now = rmf_traffic_ros2::convert(_node->now());
          assert(r.trajectory().start_time());
          if (now < *r.trajectory().start_time())
          {
            const auto& s = r.trajectory().front();
            r.trajectory().insert(
                  now,
                  s.position(),
                  Eigen::Vector3d::Zero());
          }
        }

        // If the trajectory has only one point then the robot doesn't need to
        // go anywhere.
        if (r.trajectory().size() < 2)
          continue;

        r.trajectory().begin()->adjust_times(delay);
        routes.emplace_back(std::move(r));
      }
    }

    return routes;
  }


  void execute_plan(std::vector<rmf_traffic::agv::Plan> plans)
  {
    assert(!plans.empty());
    _context->schedule.push_routes(
          collect_routes(plans),
          [&, plans](){ command_plans(plans); });
  }

  void command_plans(std::vector<rmf_traffic::agv::Plan> plans)
  {
    _remaining_waypoints.clear();
    for (const auto& plan : plans)
      for (const auto& wp : plan.get_waypoints())
        _remaining_waypoints.emplace_back(wp);

    return send_next_command(false);
  }

  std::string task_id() const
  {
    return _base_task_id + std::to_string(_command_id);
  }

  void send_next_command(bool continuous = true)
  {
    if (_remaining_waypoints.empty())
    {
      if (_emergency_active)
        report_waiting();
      else
        _task->next();
      return;
    }

    _command = rmf_fleet_msgs::msg::PathRequest();
    _command->fleet_name = _node->get_fleet_name();
    _command->robot_name = _context->robot_name();
    _command->task_id = task_id();
    ++_command_id;

    const auto& graph = _node->get_graph();

    auto make_location = [&](
        const Eigen::Vector3d& p,
        const rmf_traffic::Time t)
    {
      rmf_fleet_msgs::msg::Location location;
      location.t = rmf_traffic_ros2::convert(t);
      location.x = p[0];
      location.y = p[1];
      location.yaw = p[2];

      // TODO(MXG): Fix this assumption that every waypoint is on one map
      location.level_name = graph.get_waypoint(0).get_map_name();

      return location;
    };

    _command->path.clear();

    if (continuous && !_issued_waypoints.empty())
    {
      // Most continuations will be continuous from the last commanded waypoint,
      // so we should include that last commanded waypoint in the path that we
      // send.
      //
      // However, some events (such as docking) will leave the robot in a
      // different location than it started in, so those should not be
      // continuous.
      const auto& wp = _issued_waypoints.back();
      _command->path.emplace_back(make_location(wp.position(), wp.time()));
    }

    std::size_t i=0;
    for (; i < _remaining_waypoints.size(); ++i)
    {
      const auto& wp = _remaining_waypoints[i];
      _command->path.emplace_back(make_location(wp.position(), wp.time()));

      // Break off the command wherever an event occurs, because those are
      // points where the fleet adapter will need to issue door/lift requests.
      if (wp.event())
      {
        ++i;
        break;
      }
    }

    _issued_waypoints.clear();
    _issued_waypoints.insert(
                _issued_waypoints.end(),
                _remaining_waypoints.begin(),
                _remaining_waypoints.begin()+i);
    _remaining_waypoints.erase(
                _remaining_waypoints.begin(),
                _remaining_waypoints.begin()+i);

    const auto previous_delay = _finish_estimate - _original_finish_estimate;
    std::cout << "Noting previous delay: " << rmf_traffic::time::to_seconds(previous_delay)
              << std::endl;

    _finish_estimate = _issued_waypoints.back().time() + previous_delay;
    _original_finish_estimate = _finish_estimate;

    publish_command();
    _command_time = _node->get_clock()->now();
    _task->report_status();
  }

  void publish_command()
  {
    _node->path_request_publisher->publish(*_command);
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
      assert(_parent->_command || _parent->_retry_time);

      if (_parent->handle_docking(msg))
        return;

      if (_parent->handle_retry())
        return;

      if (!_parent->verify_task_id(msg))
        return;

      if (msg.path.empty())
        return _parent->handle_event(msg);

      _parent->handle_delay(msg);
    }

    MoveAction* _parent;
  };

  bool verify_task_id(const RobotState& msg)
  {
    if (!_command)
      return false;

    if (msg.task_id != _command->task_id)
    {
      const auto now = _node->get_clock()->now();

      // Recompute a plan if the fleet driver has a huge delay.
      if (now - _command_time > _node->get_delay_threshold())
      {
        RCLCPP_ERROR(
              _node->get_logger(),
              "The fleet driver is being unresponsive to task plan ["
              + task_id() + "]. Recomputing plan!");

        retry();
        return false;
      }

      // Attempt to resend the command if the robot has not received it yet.
      if (now - _command_time > std::chrono::milliseconds(200))
        publish_command();

      return false;
    }

    return true;
  }

  void handle_event(const RobotState& msg)
  {
    if (!_command)
      return;

    if (_issued_waypoints.empty())
      return;

    const auto& event_wp = _issued_waypoints.back();
    if (!event_wp.event())
    {
      const Eigen::Vector3d next_stop =
          _issued_waypoints.back().position();

      const auto& l = msg.location;
      const Eigen::Vector3d p{l.x, l.y, l.yaw};
      const double dist =
          (p.block<2,1>(0,0) - next_stop.block<2,1>(0,0)).norm();

      // TODO(MXG) Make this threshold configurable
      if (dist > 2.0)
      {
        RCLCPP_ERROR(
              _node->get_logger(),
              "The robot is very far [" + std::to_string(dist) + "m] from "
              "where it is supposed to be, but its path is empty");
        return;
      }

      // TODO(MXG) Make this threshold configurable
      if ( dist > 0.1 )
      {
        RCLCPP_WARN(
              _node->get_logger(),
              "The robot is somewhat far [" + std::to_string(dist) + "m] from "
              "where it is supposed to be, but we will proceed anyway.");
      }

      send_next_command();
      return;
    }

    // Check if we're already handling the event
    if (_event_executor.is_active())
      return;

    const auto& l = msg.location;
    const Eigen::Vector2d p{l.x, l.y};
    const Eigen::Vector2d target = event_wp.position().block<2,1>(0,0);

    // TODO(MXG): Consider making this threshold configurable
    if ((p - target).norm() < 0.5)
      event_wp.event()->execute(_event_executor);
  }

  void retry()
  {
    if (_emergency_active)
    {
      find_and_execute_emergency_plan();
      return;
    }

    find_and_execute_plan(_node->get_retry_wait());
  }

  bool handle_docking(const RobotState& msg)
  {
    if (!_waiting_on_docking)
      return false;

    if (msg.task_id != task_id())
    {
      _event_executor.request_docking(_current_dock_name);
      return true;
    }

    using RobotMode = rmf_fleet_msgs::msg::RobotMode;
    if (msg.mode.mode == RobotMode::MODE_DOCKING)
      return true;

    // If the task_id is the one for docking but the state's mode is not docking
    // then we assume that the docking is complete and that we should send the
    // next batch of commands.
    _event_executor.cancel();
    _waiting_on_docking = false;
    _current_dock_name.clear();
    send_next_command(false);
    return true;
  }

  bool handle_retry()
  {
    if (!_retry_time)
      return false;

    if (_node->now() < *_retry_time)
    {
      // We are supposed to retry eventually, but not yet. Therefore we return
      // true to short circuit the other update checks.
      return true;
    }

    _retry_time = rmf_utils::nullopt;

    retry();
    return true;
  }

  void handle_delay(const RobotState& msg)
  {
    if (!_command)
      return;

    bool s;
    auto trajectory_estimate =
        make_trajectory(msg, _node->get_fields().traits, s);

    const auto new_finish_estimate = *trajectory_estimate.finish_time();

    const auto total_delay = new_finish_estimate - _original_finish_estimate;
    if (total_delay > _node->get_delay_threshold())
    {
      RCLCPP_WARN(
            _node->get_logger(),
            "Attempting to replan the movement for [" + _context->robot_name()
            + "] because the delay has been too long ["
            + std::to_string(rmf_traffic::time::to_seconds(_node->get_delay_threshold()))
            + "]. Retrying in ["
            + std::to_string(rmf_traffic::time::to_seconds(_node->get_retry_wait()))
            + "] seconds.");
      // If the dealys have piled up, then consider just restarting altogether.
      return retry();
    }

    const auto new_delay = new_finish_estimate - _finish_estimate;
    // TODO(MXG): Make this threshold configurable
//    if (new_delay < std::chrono::seconds(3))
//      return;
    if (std::abs(new_delay.count()) < std::chrono::seconds(1).count())
    {
      return;
    }

    if (new_delay < std::chrono::seconds(0))
    {
      if (new_delay < std::chrono::seconds(-5))
      {
//        RCLCPP_WARN(
//              _node->get_logger(),
//              "Ignoring big negative delay ["
//              + std::to_string(rmf_traffic::time::to_seconds(new_delay)) + "]");
      }
      return;
    }

//    std::cout << "Adding delay: ["
//              << rmf_traffic::time::to_seconds(new_delay)
//              << "] total: "
//              << rmf_traffic::time::to_seconds(
//                   new_finish_estimate - _original_finish_estimate)
//              << std::endl;
//    if (new_delay < std::chrono::milliseconds(500))
//      return;

    const auto from_time =
        rmf_traffic_ros2::convert(msg.location.t) - new_delay;
    _finish_estimate = new_finish_estimate;

    if (total_delay < std::chrono::seconds(30))
    {
      _context->schedule.push_delay(new_delay, from_time);
    }
    else
    {
      const std::string& map_name = _node->get_graph().get_waypoint(0).get_map_name();
      _context->schedule.push_routes({{map_name, trajectory_estimate}}, [](){});
      _finish_estimate = new_finish_estimate;
      _original_finish_estimate = _finish_estimate;
    }
  }

  void report_waiting()
  {
    const auto now = rmf_traffic_ros2::convert(_node->get_clock()->now());
    if (!_waiting_on_emergency)
    {
      RCLCPP_INFO(
            _node->get_logger(),
            "Robot [" + _context->robot_name() + "] has arrived at its "
            "emergency parking spot and is waiting.");
      _waiting_on_emergency = true;
      const auto& p = _context->location;
      const Eigen::Vector3d position{p.x, p.y, p.yaw};

      rmf_traffic::Trajectory wait_trajectory;
      wait_trajectory.insert(now, position, Eigen::Vector3d::Zero());
      _finish_estimate = now + std::chrono::minutes(5);

      wait_trajectory.insert(
            _finish_estimate, position, Eigen::Vector3d::Zero());

      const std::string& map_name =
          _node->get_graph().get_waypoint(0).get_map_name();
      _context->schedule.push_routes({{map_name, wait_trajectory}}, [](){});
    }
    else
    {
      const auto remaining_scheduled_time =
          _finish_estimate - rmf_traffic_ros2::convert(_node->now());

      if (remaining_scheduled_time < std::chrono::minutes(3))
      {
        RCLCPP_INFO(
              _node->get_logger(),
              "Robot [" + _context->robot_name() +"] is continuing to wait "
              "at its emergency parking spot.");
        const auto new_finish_estimate = now + std::chrono::minutes(5);
        const auto schedule_delay = new_finish_estimate - _finish_estimate;
        _finish_estimate = new_finish_estimate;
        _context->schedule.push_delay(schedule_delay, now);
      }
    }
  }

  using DoorState = rmf_door_msgs::msg::DoorState;
  using LiftState = rmf_lift_msgs::msg::LiftState;
  class EventListener
      : public Listener<DoorState>,
        public Listener<LiftState>
  {
  public:

    EventListener(MoveAction* parent)
    : _parent(parent)
    {
      _parent->_node->door_state_listeners.insert(this);
      _parent->_node->lift_state_listeners.insert(this);
    }

    std::function<void(const DoorState&)> door;
    std::function<void(const LiftState&)> lift;

    void receive(const DoorState& msg) final
    {
      if (door)
        door(msg);
    }

    void receive(const LiftState& msg) final
    {
      if (lift)
        lift(msg);
    }

    ~EventListener()
    {
      _parent->_node->door_state_listeners.erase(this);
      _parent->_node->lift_state_listeners.erase(this);
    }

  private:
    MoveAction* const _parent;
  };

  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using ModeRequest = rmf_fleet_msgs::msg::ModeRequest;

  class EventExecutor : public rmf_traffic::agv::Graph::Lane::Executor
  {
  public:

    using Lane = rmf_traffic::agv::Graph::Lane;
    using DoorMode = rmf_door_msgs::msg::DoorMode;

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

    void request_docking(const std::string& dock_name)
    {
      ModeRequest request;

      using RobotMode = rmf_fleet_msgs::msg::RobotMode;
      request.mode.mode = RobotMode::MODE_DOCKING;

      using Parameter = rmf_fleet_msgs::msg::ModeParameter;
      Parameter p;
      p.name = "docking";
      p.value = dock_name;
      request.parameters.emplace_back(std::move(p));

      request.task_id = _parent->task_id();
      request.robot_name = _parent->_context->robot_name();

      _parent->_node->mode_request_publisher->publish(request);
    }

    void execute(const Lane::Dock& dock) override
    {
      if (_is_active)
        return;

      _is_active = true;

      _parent->_waiting_on_docking = true;
      _parent->_current_dock_name = dock.dock_name();

      status = " - Waiting for docking into ["
          + _parent->_current_dock_name + "]";

      ++_parent->_command_id;
      _parent->_task->report_status();
      request_docking(_parent->_current_dock_name);
    }


    void request_door_mode(
        const std::string& door_name,
        const uint32_t mode)
    {
      _command_time = _parent->_node->get_clock()->now();

      DoorRequest request;
      request.door_name = door_name;
      request.request_time = _command_time;
      request.requested_mode.value = mode;
      request.requester_id = _parent->_node->get_fleet_name()
          + "/" + _parent->_context->robot_name();

      _parent->_node->door_request_publisher->publish(request);
    }

    void wait_for_door_mode(
        const DoorState& msg,
        const uint32_t mode,
        const std::string& door_name,
        const rclcpp::Time& initial_time)
    {
      const auto time = rclcpp::Time(msg.door_time);
      if (time < initial_time)
        return;

      if (msg.door_name != door_name)
        return;

      if (mode == msg.current_mode.value)
      {
        _parent->_event_listener.door = nullptr;
        _is_active = false;
        _parent->send_next_command();
        return;
      }

      if ((time - _command_time).seconds() > 0.2)
      {
        // Send the command periodically as a precaution
        request_door_mode(door_name, mode);
      }
    }

    void execute(const Lane::DoorOpen& open) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string door_name = open.name();
      const auto initial_time = _parent->_node->get_clock()->now();

      status = " - Waiting for door [" + door_name + "] to open";

      _parent->_event_listener.door =
          [=](const DoorState& msg)
      {
        this->wait_for_door_mode(
              msg, DoorMode::MODE_OPEN, door_name, initial_time);
      };

      _parent->_task->report_status();
      request_door_mode(door_name, DoorMode::MODE_OPEN);
    }

    void execute(const Lane::DoorClose& close) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string door_name = close.name();
      const auto initial_time = _parent->_node->get_clock()->now();
      _command_time = initial_time;

      status = " - Waiting for door [" + door_name + "] to close";

      _parent->_event_listener.door =
          [=](const DoorState& msg)
      {
        this->wait_for_door_mode(
              msg, DoorMode::MODE_CLOSED, door_name, initial_time);
      };

      _parent->_task->report_status();
      request_door_mode(door_name, DoorMode::MODE_CLOSED);
    }

    void request_lift_mode(
        const std::string& lift_name,
        const std::string& floor_name,
        uint8_t lift_mode,
        uint8_t door_state)
    {
      _command_time = _parent->_node->get_clock()->now();

      LiftRequest request;
      request.lift_name = lift_name;
      request.session_id = _parent->_node->get_fleet_name()
          + "/" + _parent->_context->robot_name();
      request.request_type = lift_mode;
      request.destination_floor = floor_name;
      request.door_state = door_state;

      _parent->_node->lift_request_publisher->publish(request);
    }

    void wait_for_lift_mode(
        const LiftState& msg,
        const std::string& lift_name,
        const std::string& floor_name,
        const uint8_t lift_mode,
        const uint8_t door_state)
    {
      // TODO(MXG): Accepting any lift when lift_name is empty is a temporary
      // hack for an upcoming demo, and it should be removed immediately.
      if (!lift_name.empty() && msg.lift_name != lift_name)
        return;

      if ((_parent->_node->get_clock()->now() - _command_time).seconds() > 0.2)
      {
        // Send the command periodically as a precaution
        request_lift_mode(
              lift_name, floor_name, lift_mode, door_state);
      }

      if (msg.current_floor != floor_name)
        return;

      if (msg.door_state == door_state)
      {
        _parent->_event_listener.lift = nullptr;
        _is_active = false;
        _parent->send_next_command();
        return;
      }
    }

    void execute(const Lane::LiftDoorOpen& open) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string lift_name = open.lift_name();
      const std::string floor_name = open.floor_name();
      const auto initial_time = _parent->_node->get_clock()->now();

      status = " - Waiting for lift [" + lift_name + "] to open on floor ["
          + floor_name + "]";

      _parent->_event_listener.lift =
          [=](const LiftState& msg)
      {
        this->wait_for_lift_mode(
              msg, lift_name, floor_name,
              LiftRequest::REQUEST_AGV_MODE,
              LiftRequest::DOOR_OPEN);
      };

      _parent->_task->report_status();
      request_lift_mode(
            lift_name, floor_name,
            LiftRequest::REQUEST_AGV_MODE,
            LiftRequest::DOOR_OPEN);
    }

    void execute(const Lane::LiftDoorClose& close) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string lift_name = close.lift_name();
      const std::string floor_name = close.floor_name();
      const auto initial_time = _parent->_node->get_clock()->now();

      // We don't actually need to wait for the lift door to close, so we'll
      // just shoot off the END_SESSION message and carry on with the next
      // step of the plan.
      request_lift_mode(
            lift_name, floor_name,
            LiftRequest::REQUEST_END_SESSION,
            LiftRequest::DOOR_CLOSED);

      return _parent->send_next_command();
    }

    void execute(const Lane::LiftMove& move) final
    {
      if (_is_active)
        return;

      _is_active = true;

      const std::string lift_name = move.lift_name();
      const std::string floor_name = move.destination_floor();
      const auto initial_time = _parent->_node->get_clock()->now();

      _parent->_event_listener.lift =
          [=](const LiftState& msg)
      {
        this->wait_for_lift_mode(
              msg, lift_name, floor_name,
              LiftRequest::REQUEST_AGV_MODE,
              LiftRequest::DOOR_OPEN);
      };
    }

    void cancel()
    {
      _parent->_event_listener.door = nullptr;
      _parent->_event_listener.lift = nullptr;
      _is_active = false;
    }

    std::string status;

  private:
    rclcpp::Time _command_time;
    MoveAction* const _parent;
    bool _is_active;
  };

  void execute() final
  {
    _context->insert_listener(&_state_listener);
    find_and_execute_plan(std::chrono::seconds(0));
  }

  std::vector<rmf_traffic::agv::Plan> find_emergency_plan()
  {
    _emergency_active = true;

    const auto& planner = _node->get_planner();

    Eigen::Vector3d pose = 
        {_context->location.x, _context->location.y, _context->location.yaw};
    const auto start_time = 
        rmf_traffic_ros2::convert(_node->get_clock()->now()) + 
        std::chrono::nanoseconds(0);

    // TODO: further parameterize waypoint and lane merging distance
    const auto plan_starts =
        rmf_traffic::agv::compute_plan_starts(
            planner.get_configuration().graph(), pose, start_time, 0.1, 1.0,
            1e-8);

    if (plan_starts.empty())
    {
      RCLCPP_WARN(
          _node->get_logger(), 
          "The robot appears to be in an unrecoverable state, failed to find "
          "suitable waypoints on the graph to start planning.");
      return {};
    }

    bool interrupt_flag = false;
    _planner_options.interrupt_flag(&interrupt_flag);

    std::vector<std::thread> plan_threads;
    std::vector<rmf_utils::optional<rmf_traffic::agv::Plan>> candidate_plans;
    std::mutex plans_mutex;
    std::condition_variable plans_cv;
    bool have_plan = false;
    for (const std::size_t goal_wp : _fallback_wps)
    {
      plan_threads.emplace_back(std::thread([&, goal_wp]()
      {
        auto emergency_plan = 
            planner.plan(
              plan_starts,
              rmf_traffic::agv::Plan::Goal(goal_wp),
              _planner_options);

        std::unique_lock<std::mutex> lock(plans_mutex);
        if (emergency_plan)
          have_plan = true;

        candidate_plans.emplace_back(std::move(emergency_plan));
        plans_cv.notify_all();
      }));
    }

    const auto giveup_time =
        std::chrono::steady_clock::now() + 5*_node->get_plan_time();

    while (std::chrono::steady_clock::now() < giveup_time && !have_plan)
    {
      std::unique_lock<std::mutex> lock(plans_mutex);
      plans_cv.wait_for(lock, std::chrono::milliseconds(100),
                        [&](){ return have_plan; });
    }

    interrupt_flag =  true;
    for (auto& plan_thread : plan_threads)
      plan_thread.join();

    const auto quickest_finish_opt = get_fastest_plan_index(candidate_plans);
    if (!quickest_finish_opt)
    {
      RCLCPP_WARN(
            _node->get_logger(),
            "Robot [" + _context->robot_name() + "] is stuck while searching "
            "for an emergency plan! We will try to find a path again soon.");

      return {};
    }

    const std::size_t emergency_wp_index =
        *candidate_plans[*quickest_finish_opt]->get_waypoints().back().graph_index();
    const auto it =_node->get_waypoint_names().find(emergency_wp_index);
    const auto emergency_wp_name =
        (it == _node->get_waypoint_names().end()) ? "" : (":" + it->second);

    RCLCPP_INFO(
          _node->get_logger(),
          "Choosing emergency waypoint [" + std::to_string(emergency_wp_index)
          + emergency_wp_name + "] for [" + _context->robot_name() + "]");

    return {*candidate_plans[*quickest_finish_opt]};
  }

  void find_and_execute_emergency_plan()
  {
    auto plans = find_emergency_plan();
    if (!plans.empty())
      return execute_plan(std::move(plans));

    cancel(std::chrono::seconds(1));
  }

  void cancel(std::chrono::nanoseconds duration)
  {
    _issued_waypoints.clear();
    _remaining_waypoints.clear();
    _event_executor.cancel();

    _command = rmf_utils::nullopt;
    const auto now = _node->now();
//    _retry_time = now + duration;
    _retry_time = now;

    RCLCPP_WARN(
          _node->get_logger(),
          "Putting movement for [" + _context->robot_name()
          + "] on hold because we are encountering difficulties.");
    using ModeRequest = rmf_fleet_msgs::msg::ModeRequest;
    using RobotMode = rmf_fleet_msgs::msg::RobotMode;
    ModeRequest request;
    request.mode.mode = RobotMode::MODE_PAUSED;
    // TODO(MXG): Make this part of the task_id() function?
    request.task_id =
        task_id() + " - pause";
    request.fleet_name = _node->get_fleet_name();
    request.robot_name = _context->robot_name();

    auto hold = make_hold(
          _context->location,
          rmf_traffic_ros2::convert(now),
          std::chrono::seconds(2));
    // TODO(MXG): This is fragile. Robots ought to correctly report their level
    // name, but we can't rely on that for right now.
    const std::string& map_name = _node->get_graph().get_waypoint(0).get_map_name();
    _context->schedule.push_routes({{map_name, hold}}, [](){});

    _node->mode_request_publisher->publish(std::move(request));
  }

  void interrupt() final
  {
    if (_emergency_active)
      return;

    RCLCPP_INFO(
          _node->get_logger(),
          "Interrupting move task for [" + _context->robot_name() + "]");
    find_and_execute_emergency_plan();
  }

  void resume() final
  {
    if (!_emergency_active)
      return;

    RCLCPP_INFO(
          _node->get_logger(),
          "Resuming normal operations for [" + _context->robot_name() + "]");
    find_and_execute_plan(std::chrono::seconds(0));
  }

  Status get_status() const final
  {
    std::string status;

    const auto& waypoint_names = _node->get_waypoint_names();

    auto name_of = [&](std::size_t wp_index) -> std::string
    {
      const auto wp_it = waypoint_names.find(wp_index);
      return wp_it == waypoint_names.end()?
            "#" + std::to_string(_goal_wp_index) : wp_it->second;
    };

    status = "Moving to waypoint [" + name_of(_goal_wp_index) + "]";

    if (_emergency_active)
      status += " - Emergency Interruption";
    else
      status += " - In progress";

    if (_waiting_on_emergency)
    {
      status += " - Waiting for emergency to end";
      return {status};
    }

    if (_event_executor.is_active())
    {
      status += _event_executor.status;
    }

    rmf_utils::optional<rmf_traffic::agv::Plan::Waypoint> final_wp;
    if (!_remaining_waypoints.empty())
      final_wp = _remaining_waypoints.back();
    else if(!_issued_waypoints.empty())
      final_wp = _issued_waypoints.back();

    if (final_wp)
      return {status, rmf_traffic_ros2::to_ros2(final_wp->time())};

    return {status, _node->now()};
  }

private:
  FleetAdapterNode* const _node;
  Task* const _task;
  FleetAdapterNode::RobotContext* const _context;
  const std::size_t _goal_wp_index;
  const std::string _base_task_id;
  const std::vector<std::size_t>& _fallback_wps;
  StateListener _state_listener;
  EventExecutor _event_executor;
  EventListener _event_listener;

  rclcpp::Time _command_time;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _remaining_waypoints;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _issued_waypoints;
  rmf_traffic::Time _finish_estimate = rmf_traffic::Time(std::chrono::seconds(0));
  rmf_traffic::Time _original_finish_estimate = rmf_traffic::Time(std::chrono::seconds(0));
  rmf_utils::optional<Eigen::Vector3d> _next_stop;
  rmf_utils::optional<rmf_fleet_msgs::msg::PathRequest> _command;
  std::size_t _command_id = 0;

  rmf_utils::optional<rclcpp::Time> _retry_time = rmf_utils::nullopt;

  bool _waiting_on_docking = false;
  std::string _current_dock_name;

  bool _emergency_active = false;
  bool _waiting_on_emergency = false;

  rmf_traffic::agv::Planner::Options _planner_options;
  rmf_utils::clone_ptr<rmf_traffic::agv::ScheduleRouteValidator> _validator;

  std::shared_ptr<void> _handle;
};

MoveAction::~MoveAction()
{
  _context->remove_listener(&_state_listener);
}

} // anonymous namespace

//==============================================================================
std::unique_ptr<Action> make_move(
    FleetAdapterNode* node,
    FleetAdapterNode::RobotContext* state,
    Task* parent,
    const std::size_t goal_wp_index,
    const std::size_t move_id)
{
  return std::make_unique<MoveAction>(
        node, parent, state, goal_wp_index, move_id);
}

} // namespace full_control
} // namespace rmf_fleet_adapter
