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

#include "internal_TrafficLight.hpp"

#include "../services/FindPath.hpp"
#include "../services/Negotiate.hpp"

#include <rmf_utils/Modular.hpp>
#include <rmf_utils/math.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/schedule/StubbornNegotiator.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TrafficLight::UpdateHandle::Implementation::Data
    : public std::enable_shared_from_this<Data>
{
public:

  std::shared_ptr<CommandHandle> command;

  rmf_traffic::schedule::Participant itinerary;

  rmf_traffic::blockade::Participant blockade;

  rmf_traffic::agv::VehicleTraits traits;

  std::shared_ptr<rmf_traffic::schedule::Snappable> schedule;

  rxcpp::schedulers::worker worker;

  std::shared_ptr<rclcpp::Node> node;

  std::size_t current_version = 0;

  std::shared_ptr<rmf_traffic::Profile> profile;

  std::vector<Waypoint> path;

  std::shared_ptr<rmf_traffic::agv::Planner> planner;

  std::map<std::size_t, rmf_traffic::Time> arrival_timing;
  std::map<std::size_t, rclcpp::Time> departure_timing;
  std::map<std::size_t, std::size_t> pending_waypoint_index;

  std::vector<rmf_traffic::Route> active_itinerary;
  std::vector<rmf_traffic::Route> plan_itinerary;
  std::vector<rmf_traffic::agv::Plan::Waypoint> pending_waypoints;
  bool waiting_for_departure = false;
  std::optional<std::vector<CommandHandle::Checkpoint>> ready_checkpoints;
  std::size_t next_departure_checkpoint = 0;

  std::size_t processing_version = 0;

  std::shared_ptr<services::FindPath> find_path_service;
  rxcpp::subscription find_path_subscription;

  /// A timer that periodically checks if the next checkpoint is ready, when
  /// we're sitting and waiting for the departure time of our current checkpoint
  rclcpp::TimerBase::SharedPtr ready_check_timer;

  /// A timer that periodically checks if we need to delay our scheduled
  /// trajectory while we wait for the blockade manager to give us permission
  /// to depart from a checkpoint
  rclcpp::TimerBase::SharedPtr waiting_timer;

  struct NegotiateManagers
  {
    rmf_rxcpp::subscription_guard subscription;
    rclcpp::TimerBase::SharedPtr timer;
  };
  using NegotiatePtr = std::shared_ptr<services::Negotiate>;
  using NegotiateServiceMap =
      std::unordered_map<NegotiatePtr, NegotiateManagers>;
  NegotiateServiceMap negotiate_services;

  /// Completely change the path that this participant is following
  void update_path(
      std::size_t version,
      const std::vector<Waypoint>& new_path);

  /// Calculate the preferred timing of the participant
  void plan_timing(
    rmf_traffic::agv::Plan::Start start,
    std::size_t version,
    std::vector<Waypoint> new_path,
    std::shared_ptr<rmf_traffic::agv::Planner> new_planner);

  rmf_utils::optional<rmf_traffic::schedule::ItineraryVersion> update_timing(
      std::size_t version,
      std::vector<Waypoint> new_path,
      rmf_traffic::agv::Plan new_plan,
      std::shared_ptr<rmf_traffic::agv::Planner> new_planner);

  void update_location(
      std::size_t version,
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      Eigen::Vector3d location,
      std::size_t checkpoint_index);

  rmf_utils::optional<rmf_traffic::agv::Plan::Start> estimate_location() const;

  void new_range(
      const rmf_traffic::blockade::ReservationId reservation_id,
      const rmf_traffic::blockade::ReservedRange& new_range);

  void watch_for_ready(
      std::size_t version,
      std::size_t checkpoint_id);

  bool check_if_ready(
      std::size_t version,
      std::size_t checkpoint_id);

  void check_waiting_delay(
      std::size_t version,
      std::size_t checkpoint_id);

  void send_checkpoints(std::vector<CommandHandle::Checkpoint> checkpoints);

  Data(
      std::shared_ptr<CommandHandle> command_,
      rmf_traffic::blockade::Participant blockade_,
      rmf_traffic::schedule::Participant itinerary_,
      rmf_traffic::agv::VehicleTraits traits_,
      std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
      rxcpp::schedulers::worker worker_,
      std::shared_ptr<rclcpp::Node> node_)
    : command(std::move(command_)),
      itinerary(std::move(itinerary_)),
      blockade(std::move(blockade_)),
      traits(std::move(traits_)),
      schedule(std::move(schedule_)),
      worker(std::move(worker_)),
      node(std::move(node_)),
      profile(std::make_shared<rmf_traffic::Profile>(
                itinerary.description().profile()))
  {
    // Do nothing
  }
};

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::update_path(
    const std::size_t version,
    const std::vector<Waypoint>& new_path)
{
  if (rmf_utils::modular(version).less_than(processing_version))
    return;

  assert(version != processing_version);
  processing_version = version;

  const auto now = rmf_traffic_ros2::convert(node->now());

  plan_itinerary.clear();
  pending_waypoints.clear();
  itinerary.clear();
  ready_check_timer = nullptr;
  waiting_timer = nullptr;
  waiting_for_departure = true;
  ready_checkpoints.reset();
  next_departure_checkpoint = 0;

  if (new_path.empty())
  {
    // Clear out this robot's itinerary because it is done moving
    RCLCPP_INFO(
      node->get_logger(),
      "Traffic light controlled robot [%s] owned by [%s] is being taken off "
      "the schedule after being given a null path.",
      itinerary.description().name().c_str(),
      itinerary.description().owner().c_str());

    planner = nullptr;
    path.clear();
    departure_timing.clear();
    return;
  }
  else if (new_path.size() == 1)
  {
    RCLCPP_ERROR(
          node->get_logger(),
          "Traffic light controlled robot [%s] owned by [%s] was given only "
          "one waypoint. The traffic light controller requires the robot to "
          "start and end at separate locations which are both designated safe "
          "zones for the robot.",
          itinerary.description().name().c_str(),
          itinerary.description().owner().c_str());
    assert(false);
    return;
  }

  for (std::size_t i=1; i < new_path.size(); ++i)
  {
    const auto& wp0 = new_path[i-1];
    const auto& wp1 = new_path[i];

    const auto p0 = wp0.position();
    const auto p1 = wp1.position();

    const double dist = (p1 - p0).norm();
    if (dist < 1e-3 && wp0.map_name() == wp1.map_name())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Traffic light controlled robot [%s] owned by [%s] was given waypoints "
        "[%d, %d] that are too close together [%fm]",
        itinerary.description().name().c_str(),
        itinerary.description().owner().c_str(),
        i-1, i, dist);
      assert(false);
      return;
    }
  }

  std::vector<rmf_traffic::blockade::Writer::Checkpoint> checkpoints;

  rmf_traffic::agv::Graph graph;
  for (std::size_t i=0; i < new_path.size(); ++i)
  {
    const auto& wp = new_path[i];
    graph.add_waypoint(wp.map_name(), wp.position().block<2,1>(0,0))
        .set_passthrough_point(!wp.yield())
        .set_holding_point(wp.yield());

    if (i > 0)
    {
      const auto& last_wp = new_path[i-1];
      rmf_traffic::agv::Graph::Lane::EventPtr event = nullptr;
      if (last_wp.mandatory_delay() > std::chrono::nanoseconds(0))
      {
        // We use DoorOpen for lack of a better placeholder
        event = rmf_traffic::agv::Graph::Lane::Event::make(
              rmf_traffic::agv::Graph::Lane::Wait(last_wp.mandatory_delay()));
      }

      graph.add_lane(rmf_traffic::agv::Graph::Lane::Node(i-1, event), i);
    }

    checkpoints.push_back(
        {wp.position().block<2,1>(0,0), wp.map_name(), wp.yield()});
  }

  auto new_planner = std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Plan::Configuration(graph, traits),
        rmf_traffic::agv::Plan::Options(nullptr));

  rmf_traffic::agv::Plan::Start start{now, 0, new_path.front().position()[2]};
  plan_timing(std::move(start), version, new_path, std::move(new_planner));

  blockade.set(std::move(checkpoints));
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::plan_timing(
    rmf_traffic::agv::Plan::Start start,
    const std::size_t version,
    std::vector<Waypoint> new_path,
    std::shared_ptr<rmf_traffic::agv::Planner> new_planner)
{
  rmf_traffic::agv::Plan::Goal goal(
        new_planner->get_configuration().graph().num_waypoints()-1);

  find_path_service = std::make_shared<services::FindPath>(
    new_planner, rmf_traffic::agv::Plan::StartSet({std::move(start)}),
    std::move(goal), schedule->snapshot(), itinerary.id(), profile);

  find_path_subscription = rmf_rxcpp::make_job<services::FindPath::Result>(
        find_path_service)
      .observe_on(rxcpp::identity_same_worker(worker))
      .subscribe(
        [w = weak_from_this(),
         version,
         new_path = std::move(new_path),
         new_planner](
        const services::FindPath::Result& result)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (version != data->processing_version)
    {
      // This means the path that triggered this service is deprecated.
      return;
    }

    if (!result)
    {
      RCLCPP_ERROR(
        data->node->get_logger(),
        "Failed to find any itinerary for submitted path #%d of robot [%s] "
        "owned by [%s]. This is a critical bug and should be reported to the "
        "rmf_core developers.",
        version,
        data->itinerary.description().name().c_str(),
        data->itinerary.description().owner().c_str());

      return;
    }

    data->current_version = version;

    data->update_timing(
      version,
      std::move(new_path),
      std::move(*result),
      std::move(new_planner));

    data->watch_for_ready(version, 0);
  });
}

namespace {

//==============================================================================
rmf_utils::optional<rmf_traffic::Time> linear_interpolate_time(
    const rmf_traffic::Trajectory::Waypoint& wp0,
    const rmf_traffic::Trajectory::Waypoint& wp1,
    const Eigen::Vector2d& p)
{
  const Eigen::Vector2d p0 = wp0.position().block<2,1>(0,0);
  const Eigen::Vector2d p1 = wp1.position().block<2,1>(0,0);
  if ((p1 - p0).dot(p - p0) < 0)
    return rmf_utils::nullopt;

  if ((p1 - p0).dot(p1 - p) < 0)
    return rmf_utils::nullopt;

  const double v = wp0.velocity().block<2,1>(0,0).norm();
  assert(std::abs(v - wp1.velocity().block<2,1>(0,0).norm()) < 1e-2);

  const double r = (p1 - p0).normalized().dot(p - p0);
  const double s = r/v;

  assert(s <= rmf_traffic::time::to_seconds(wp1.time() - wp0.time()));
  return wp0.time() + rmf_traffic::time::from_seconds(s);
}

//==============================================================================
rmf_utils::optional<rmf_traffic::Time> parabolic_interpolate_time(
    const rmf_traffic::Trajectory::Waypoint& wp0,
    const rmf_traffic::Trajectory::Waypoint& wp1,
    const Eigen::Vector2d& p)
{
  const Eigen::Vector2d p0 = wp0.position().block<2,1>(0,0);
  const Eigen::Vector2d p1 = wp1.position().block<2,1>(0,0);
  if ((p1 - p0).dot(p - p0) < 0)
    return rmf_utils::nullopt;

  if ((p1 - p0).dot(p1 - p) < 0)
    return rmf_utils::nullopt;

  const double r = (p - p0).norm();
  const double v0 = wp0.velocity().block<2,1>(0,0).norm();
  const double v1 = wp1.velocity().block<2,1>(0,0).norm();
  const double dt = rmf_traffic::time::to_seconds(wp1.time() - wp0.time());
  const double a = (v1 - v0)/dt;

  const double D = v0*v0 + 2*a*r;

  const double t_minus = (-v0 - std::sqrt(D))/a;
  if (0 <= t_minus && t_minus <= dt)
    return wp0.time() + rmf_traffic::time::from_seconds(t_minus);

  const double t_plus = (-v0 + std::sqrt(D))/a;
  if (0 <= t_plus && t_plus <= dt)
    return wp0.time() + rmf_traffic::time::from_seconds(t_plus);

  return rmf_utils::nullopt;
}

//==============================================================================
rmf_traffic::Time interpolate_time(
  const rmf_traffic::agv::VehicleTraits& traits,
  const rmf_traffic::agv::Plan::Waypoint& wp0,
  const rmf_traffic::agv::Plan::Waypoint& wp1,
  const Eigen::Vector2d& p)
{
  std::vector<Eigen::Vector3d> positions;
  positions.resize(2, Eigen::Vector3d::Zero());
  positions[0].block<2,1>(0,0) = wp0.position().block<2,1>(0,0);
  positions[1].block<2,1>(0,0) = wp1.position().block<2,1>(0,0);

  const auto interp = rmf_traffic::agv::Interpolate::positions(
        traits, wp0.time(), positions);

  // The interpolation will either be two parabolas or it will be two parabolas
  // with a linear component in the middle.
  assert(interp.size() == 3 || interp.size() == 4);
  const double r = (p - wp0.position().block<2,1>(0,0)).norm();
  if (r < 1e-3)
    return wp0.time();

  const auto end = interp.end();
  if (interp.size() == 3)
  {
    for (auto it = ++interp.begin(); it != end; ++it)
    {
      const auto last = --rmf_traffic::Trajectory::const_iterator(it);
      if (const auto t = parabolic_interpolate_time(*last, *it, p))
        return *t;
    }
  }
  else if(interp.size() == 4)
  {
    std::size_t count = 0;
    for (auto it = ++interp.begin(); it != end; ++it, ++count)
    {
      const auto last = --rmf_traffic::Trajectory::const_iterator(it);
      if (count == 0 || count == 2)
      {
        if (const auto t = parabolic_interpolate_time(*last, *it, p))
          return *t;
      }
      else
      {
        if (const auto t = linear_interpolate_time(*last, *it, p))
          return *t;
      }
    }
  }

  throw std::runtime_error(
    "[rmf_fleet_adapter::agv::TrafficLight] Failed to interpolate an "
    "intermediary waypoint.");
}

//==============================================================================
rmf_utils::optional<rmf_traffic::Time> interpolate_time(
  const rmf_traffic::Time now,
  const rmf_traffic::agv::VehicleTraits& traits,
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  const Eigen::Vector3d& location)
{
  const Eigen::Vector2d p = location.block<2,1>(0,0);

  assert(waypoints.size() > 1);

  const std::size_t N = waypoints.size();
  for (std::size_t i=1; i < waypoints.size(); ++i)
  {
    const auto index = N - i;
    const auto& wp0 = waypoints.at(index-1);
    const auto& wp1 = waypoints.at(index);

    const auto gi_0 = wp0.graph_index();
    const auto gi_1 = wp1.graph_index();

    if (gi_1 && (!gi_0 || (*gi_0 != *gi_1)))
    {
      // Check if the robot is traversing a lane
      const Eigen::Vector2d p0 = wp0.position().block<2,1>(0,0);
      const Eigen::Vector2d p1 = wp1.position().block<2,1>(0,0);
      const Eigen::Vector2d n = p1 - p0;
      const double length = n.norm();
      const Eigen::Vector2d dir = n/length;

      const Eigen::Vector2d r = p - p0;
      const double traversal = r.dot(dir);
      const double deviation = (r - traversal*dir).norm();

      // TODO(MXG): Make this threshold configurable
      const double deviation_threshold = 1.0;

      // If the vehicle has deviated significantly from the path, then we should
      // recompute the timing information.
      if (deviation > deviation_threshold)
        return rmf_utils::nullopt;

      // If the vehicle is behind the lane then we should recompute the timing
      // information.
      if (traversal < 0.0)
        return rmf_utils::nullopt;

      try
      {
        return interpolate_time(traits, wp0, wp1, location.block<2,1>(0,0));
      }
      catch (const std::exception&)
      {
        // TODO(MXG): This is kind of suspicious. Should we escalate the issue
        // at this point?
        return rmf_utils::nullopt;
      }
    }
    else
    {
      // Check if the robot is on the target waypoint
      const Eigen::Vector2d p0 = wp0.position().block<2,1>(0,0);

      // TODO(MXG): Make this threshold configurable
      const double distance_threshold = 0.5;
      if ((p - p0).norm() > distance_threshold)
        continue;

      const double yaw = location[2];
      const auto yaw0 = wp0.position()[2];
      const auto yaw1 = wp1.position()[2];
      const double dyaw = rmf_utils::wrap_to_pi(yaw1 - yaw0);

      const auto t_min = wp0.time();
      const auto t_max = wp1.time();

      // TODO(MXG): Make this threshold configurable
      const double waiting_threshold = 1.0*M_PI/180.0;
      if (std::abs(dyaw) < waiting_threshold)
      {
        // This is a waiting segment

        // TODO(MXG): Make this threshold configurable
        const double yaw_threshold = 20.0*M_PI/180.0;

        if (std::abs(yaw - yaw0) < yaw_threshold)
        {
          // We are waiting

          // If the current time has not yet reached the end of the waiting time
          // then we just return the current time to imply that the robot is
          // on-time. This may leave a false shadow of an itinerary on the
          // schedule, but that should usually be okay.
          if (now <= t_max)
            return now;

          return t_max;
        }
      }
      else
      {
        // This is a turning segment

        // NOTE: For now we'll just approximate turns as constant-velocities
        // while estimating. We could make this estimation more accurate in
        // the future if this turns out to not be good enough.
        const double s = rmf_utils::wrap_to_pi(yaw - yaw0)/dyaw;
        const double dt = rmf_traffic::time::to_seconds(t_max - t_min);
        const auto t = t_min + rmf_traffic::time::from_seconds(s*dt);
        if (now <= t)
          return now;

        return t;
      }
    }
  }

  return rmf_utils::nullopt;
}

//==============================================================================
double calculate_orientation(
    const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1)
{
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}
} // anonymous namespace

//==============================================================================
rmf_utils::optional<rmf_traffic::schedule::ItineraryVersion>
TrafficLight::UpdateHandle::Implementation::Data::update_timing(
    const std::size_t version,
    std::vector<Waypoint> path_,
    rmf_traffic::agv::Plan plan_,
    std::shared_ptr<rmf_traffic::agv::Planner> planner_)
{
  if (version != current_version)
    return rmf_utils::nullopt;

  path = std::move(path_);
  assert(!path.empty());

  // TODO(MXG): Is there a reason to save this as a member variable?
  plan_itinerary = plan_.get_itinerary();

  planner = std::move(planner_);

  pending_waypoints = plan_.get_waypoints();
  std::cout << "Setting " << pending_waypoints.size() << " pending waypoints" << std::endl;

  // Remove all waypoints that don't relate to a graph index
  const auto remove_from = std::remove_if(
        pending_waypoints.begin(), pending_waypoints.end(),
        [](const auto& wp){ return !wp.graph_index().has_value(); });
  pending_waypoints.erase(remove_from, pending_waypoints.end());
  assert(!pending_waypoints.empty());

  const rclcpp::Time initial_t =
      rmf_traffic_ros2::convert(pending_waypoints.front().time());

  // For time that should be considered in the past
  const rclcpp::Time passed_t = initial_t - rclcpp::Duration(3600.0);

  const auto& graph = planner->get_configuration().graph();

  std::optional<std::size_t> current_wp;

  for (std::size_t i=0; i < pending_waypoints.size(); ++i)
  {
    const auto& wp = pending_waypoints[i];

    assert(wp.graph_index().has_value());

    if (!current_wp)
      current_wp = *wp.graph_index();

    if (*wp.graph_index() != *current_wp)
    {
      const std::size_t next_wp = *wp.graph_index();
      for (std::size_t k=*current_wp+1; k < next_wp; ++k)
      {
        // For any intermediate waypoints that will be passed over without
        // stopping, we should try to calculate the time that the robot is
        // expected to pass over it.
        const auto p = graph.get_waypoint(k).get_location();
        const auto t = interpolate_time(traits, pending_waypoints[i-1], wp, p);
        arrival_timing[k] = t;
        departure_timing[k] = rmf_traffic_ros2::convert(t);
        pending_waypoint_index[k] = i;
      }

      current_wp = *wp.graph_index();
      arrival_timing[*current_wp] = wp.time();
    }

    // This might get overriden several times since a single path waypoint might
    // show up several times in a plan. We want to use the last time that is
    // associated with this path waypoint.
    departure_timing[*current_wp] = rmf_traffic_ros2::convert(wp.time());
    pending_waypoint_index[*current_wp] = i;
  }

  std::vector<rmf_traffic::Route> full_itinerary;
  for (auto& r : active_itinerary)
  {
    // Adjust the times of the active itinerary to match any delays that have
    // built up.
    if (!r.trajectory().empty())
      r.trajectory().front().adjust_times(itinerary.delay());
  }
  full_itinerary.insert(
        full_itinerary.end(),
        active_itinerary.begin(),
        active_itinerary.end());

  // Add the pending itinerary so the schedule can see as far ahead as possible.
  full_itinerary.insert(
        full_itinerary.end(),
        plan_itinerary.begin(),
        plan_itinerary.end());

  std::cout << "Still have " << pending_waypoints.size() << " pending waypoints" << std::endl;
  itinerary.set(full_itinerary);
  return itinerary.version();
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::update_location(
    const std::size_t version,
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    const Eigen::Vector3d location,
    const std::size_t checkpoint_index)
{
  const auto now = rmf_traffic_ros2::convert(node->now());

  worker.schedule(
        [w = weak_from_this(),
         version,
         waypoints,
         location,
         now,
         checkpoint_index](const auto&)
  {
    const auto data = w.lock();
    if (!data)
      return;

    if (version != data->current_version)
      return;

    if (checkpoint_index != data->blockade.last_reached())
    {
      std::cout << __LINE__ << ": Participant " << data->itinerary.id()
                << " reached " << checkpoint_index
                << " (was " << data->blockade.last_reached() << ")"
                << ":" << data->blockade.reservation_id().value() << std::endl;
    }
    data->blockade.reached(checkpoint_index);

    assert(checkpoint_index < data->arrival_timing.size());
    const auto expected_time = interpolate_time(
          now, data->traits, waypoints, location);

    if (expected_time)
    {
      const auto new_delay = now - *expected_time;
      const auto time_shift = new_delay - data->itinerary.delay();
      const auto threshold = std::chrono::seconds(1);
      if (time_shift < -threshold || threshold < time_shift)
        data->itinerary.delay(time_shift);

      return;
    }

    // TODO(MXG): If interpolate_time() is changed to use the departure
    RCLCPP_WARN(
      data->node->get_logger(),
      "Failed to compute timing estimate for [%s] owned by [%s] "
      "moving towards path index [%d]",
      data->itinerary.description().name().c_str(),
      data->itinerary.description().owner().c_str(),
      checkpoint_index);
  });
}

//==============================================================================
rmf_utils::optional<rmf_traffic::agv::Plan::Start>
TrafficLight::UpdateHandle::Implementation::Data::estimate_location() const
{
  // Estimate the current position of the robot based on its intended trajectory
  // and the delays that it has reported.

  const auto now = rmf_traffic_ros2::convert(node->now());

  const auto effective_time = now - itinerary.delay();
  const rclcpp::Time effective_time_rcl =
      rmf_traffic_ros2::convert(effective_time);

  if (departure_timing.crbegin()->second < effective_time_rcl)
  {
    // The time that this robot is scheduled for has passed, so there's no way
    // to estimate its position.
    return rmf_utils::nullopt;
  }

  const auto& graph = planner->get_configuration().graph();

  rmf_traffic::agv::Plan::Start start(
        now, 0,
        calculate_orientation(
          path[0].position().block<2,1>(0,0),
          path[1].position().block<2,1>(0,0)));

  if (effective_time_rcl <= departure_timing.begin()->second)
  {
    // The robot has not made progress from its starting point, so we'll just
    // report its original starting conditions.
    return start;
  }

  bool traversing_lane = false;
  for (std::size_t i=1; i < departure_timing.size(); ++i)
  {
    if (effective_time_rcl < departure_timing.at(i))
    {
      start.waypoint(i);

      if (effective_time < arrival_timing.at(i))
      {
        // The vehicle is still moving towards this waypoint.
        const auto* lane = graph.lane_from(i-1, i);
        assert(lane);
        start.lane(lane->index());
        start.orientation(
              calculate_orientation(
                path[i-1].position().block<2,1>(0,0),
                path[i].position().block<2,1>(0,0)));

        traversing_lane = true;
      }

      break;
    }
  }


  for (const auto& route : active_itinerary)
  {
    if (*route.trajectory().start_time() <= effective_time
        && effective_time <= *route.trajectory().finish_time())
    {
      const auto motion =
          rmf_traffic::Motion::compute_cubic_splines(route.trajectory());

      const auto p = motion->compute_position(effective_time);
      start.orientation(p[2]);

      if (traversing_lane)
        start.location(Eigen::Vector2d(p.block<2,1>(0,0)));
    }
  }

  return start;
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::new_range(
    const rmf_traffic::blockade::ReservationId reservation_id,
    const rmf_traffic::blockade::ReservedRange& new_range)
{
  std::cout << "RECEIVED A NEW RANGE FOR [" << itinerary.id() << "]" << std::endl;
  if (reservation_id != blockade.reservation_id())
    return;

  if (new_range.begin == new_range.end)
    return;

  const auto in_range_inclusive = [this](
      const std::size_t plan_index,
      const std::size_t end_path_index) -> bool
  {
    if (pending_waypoints.size() <= plan_index)
    {
      std::cout << " -- plan_index [" << plan_index << "] exceeded pending_waypoints ["
                << pending_waypoints.size() << "]" << std::endl;
      return false;
    }

    std::cout << " -- comparing " << pending_waypoints.at(plan_index)
                 .graph_index().value() << " <= " << end_path_index << std::endl;
    return pending_waypoints.at(plan_index)
        .graph_index().value() <= end_path_index;
  };

  const auto in_range_exclusive = [&](
      const std::size_t plan_index,
      const std::size_t end_path_index) -> bool
  {
    if (pending_waypoints.size() <= plan_index)
      return false;

    std::cout << " -- comparing " << pending_waypoints.at(plan_index)
                 .graph_index().value() << " < " << end_path_index << std::endl;
    return pending_waypoints.at(plan_index)
        .graph_index().value() < end_path_index;
  };

  std::cout << "Pending Waypoints Path Indices:\n";
  for (const auto& p : pending_waypoints)
    std::cout << " -- " << p.graph_index().value() << ": " << p.position().transpose() << std::endl;
  std::cout << std::endl;

  std::vector<CommandHandle::Checkpoint> checkpoints;
  std::size_t next = 0;

  std::size_t current_checkpoint_index = next_departure_checkpoint;
  next_departure_checkpoint = new_range.end;

#ifndef NDEBUG
  if (ready_checkpoints.has_value())
  {
    assert(!ready_checkpoints.value().empty());
    assert(ready_checkpoints.value().back().waypoint_index
           == current_checkpoint_index-1);
  }
#endif

  while (in_range_inclusive(next, current_checkpoint_index))
    ++next;
  --next;

  std::cout << "Initial next: " << next << " | new_range.end: " << new_range.end << std::endl;
  while (in_range_exclusive(next, new_range.end))
  {
    const std::size_t next_checkpoint_index = current_checkpoint_index + 1;
    std::cout << "current_checkpoint_index: " << current_checkpoint_index
              << " | next_checkpoint_index: " << next_checkpoint_index << std::endl;

    std::size_t end_plan_segment_index = next;
    while (in_range_inclusive(end_plan_segment_index, next_checkpoint_index))
      ++end_plan_segment_index;

    assert(end_plan_segment_index > next);
    if (end_plan_segment_index < pending_waypoints.size())
    {
      const std::size_t check_index =
          pending_waypoints[end_plan_segment_index-1].graph_index().value();

      if (check_index <= current_checkpoint_index)
      {
        std::cout << " -- incrementing end_plan_index" << std::endl;
        // We need to make sure that the last plan waypoint we pass along
        // belongs to the next path checkpoint, otherwise the interpolator won't
        // have enough information to infer what the current delay is.
        ++end_plan_segment_index;
      }
    }

    std::cout << " -- end_plan_index: " << end_plan_segment_index << std::endl;

    std::vector<rmf_traffic::agv::Plan::Waypoint> departed_waypoints(
      pending_waypoints.begin() + next,
      pending_waypoints.begin() + end_plan_segment_index);

    assert(!departed_waypoints.empty());

    const std::size_t last_checkpoint_index =
        std::min(
          departed_waypoints.back().graph_index().value(),
          new_range.end);

    std::set<std::size_t> debug_set_inspection;
    for (const auto& wp : departed_waypoints)
      debug_set_inspection.insert(wp.graph_index().value());

    std::cout << "Creating checkpoints " << current_checkpoint_index << " -> "
              << last_checkpoint_index << std::endl;
    for (std::size_t c = current_checkpoint_index;
         c < last_checkpoint_index; ++c)
    {
      std::cout << "For [" << c << "] we are issuing:";
      for (const auto& d : debug_set_inspection)
        std::cout << " " << d;
      std::cout << std::endl;

      auto departed =
          [w = weak_from_this(),
           version = current_version,
           checkpoint_index = c,
           waypoints = departed_waypoints](Eigen::Vector3d location)
      {
        if (const auto data = w.lock())
          data->update_location(version, waypoints, location, checkpoint_index);
      };

      checkpoints.emplace_back(
            CommandHandle::Checkpoint{
              c,
              departure_timing.at(current_checkpoint_index),
              std::move(departed)
            });
    }

    next = end_plan_segment_index-1;
    if (next_checkpoint_index < last_checkpoint_index)
    {
      // If the last_checkpoint_index is greater than the next_checkpoint_index,
      // that means we needed to reach past the end checkpoint for the sake of
      // interpolation, because the next_checkpoint_index gets skipped by the
      // planner (meaning the robot isn't expected to need to stop on that
      // checkpoint).
      //
      // When that happens, all the intermediate checkpoints will already have
      // been added to the checkpoints earlier. Therefore, we should just move
      // ahead to the last_checkpoint_index.
      ++next;
    }

    if (next < pending_waypoints.size())
    {
      current_checkpoint_index =
          pending_waypoints.at(next).graph_index().value();
    }
  }

  if (checkpoints.empty())
  {
    std::cout << "No new checkpoints found" << std::endl;
    return;
  }

  const auto end_plan_it = [&]()
      -> std::vector<rmf_traffic::agv::Plan::Waypoint>::const_iterator
  {
    auto it = ++pending_waypoints.begin();
    while (it != pending_waypoints.end())
    {
      const auto check = it++;
      std::cout << "Checking checkpoint index " << check->graph_index().value() << std::endl;
      if (check->graph_index().value() >= new_range.end)
        return check-1;
    }

    return it;
  }();

  if (end_plan_it != pending_waypoints.end())
    std::cout << "Erasing up to (not including) " << end_plan_it->graph_index().value()
              << " | delta: " << (end_plan_it - pending_waypoints.begin()) << std::endl;
  pending_waypoints.erase(pending_waypoints.begin(), end_plan_it);
  std::cout << "New Pending Waypoints Path Indices:\n";
  for (const auto& p : pending_waypoints)
    std::cout << " -- " << p.graph_index().value() << ": " << p.position().transpose() << std::endl;
  std::cout << std::endl;

  // Cancel this timer that is watching to see if we need to delay our schedule
  // while waiting to depart
  waiting_timer = nullptr;

  if (waiting_for_departure)
  {
    std::cout << "Sending checkpoints right away" << std::endl;
    send_checkpoints(std::move(checkpoints));
  }
  else
  {
    if (!ready_checkpoints.has_value())
      ready_checkpoints = std::vector<CommandHandle::Checkpoint>();

    std::cout << "Appending checkpoints for later" << std::endl;
    // TODO(MXG): We could probably use a move-iterator here
    ready_checkpoints->insert(
          ready_checkpoints->end(),
          checkpoints.begin(),
          checkpoints.end());
  }
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::watch_for_ready(
    const std::size_t version,
    const std::size_t checkpoint_id)
{
  std::cout << "Triggered watch_for_ready for " << itinerary.id()
            << " at checkpoint " << checkpoint_id << std::endl;
  if (version != current_version)
  {
    std::cout << "MISMATCHED VERSION FOR " << itinerary.id()
              << "???: " << version << " vs " << current_version << std::endl;
    return;
  }

  if (checkpoint_id >= path.size()-1)
  {
    // TODO(MXG): We could tell the blockade moderator that we finished here.
    // But we would need to extend the API of blockade::Participant.
  }

  blockade.reached(checkpoint_id);

  waiting_for_departure = true;

  if (ready_checkpoints.has_value())
  {
    std::cout << "Sending queued waypoints for " << itinerary.id() << std::endl;
    send_checkpoints(std::move(ready_checkpoints.value()));
    ready_checkpoints.reset();
  }
  else
  {
    std::cout << "Creating waiting_timer" << std::endl;
    waiting_timer = node->create_wall_timer(
          std::chrono::seconds(1),
          [w = weak_from_this(), version, checkpoint_id]()
    {
      if (const auto data = w.lock())
        data->check_waiting_delay(version, checkpoint_id);
    });
  }

  std::cout << "About to do first ready check" << std::endl;
  if (check_if_ready(version, checkpoint_id))
    return;

  ready_check_timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [w = weak_from_this(), version, checkpoint_id]()
  {
    std::cout << "Triggered ready check timer" << std::endl;
    if (const auto data = w.lock())
    {
      if (data->check_if_ready(version, checkpoint_id))
        data->ready_check_timer = nullptr;
    }
  });
}

//==============================================================================
bool TrafficLight::UpdateHandle::Implementation::Data::check_if_ready(
    std::size_t version,
    std::size_t checkpoint_id)
{
  std::cout << "Checking if ready" << std::endl;
  // Counter-intuitively, we return true for these sanity check cases because we
  // want the caller to quit right away.
  if (version != current_version)
  {
    std::cout << "Wrong version: [" << version << "] vs [" << current_version
              << "]" << std::endl;
    return true;
  }

  if (pending_waypoints.empty())
  {
    std::cout << "NO PENDING WAYPOINTS??? Line: " << __LINE__ << std::endl;
    return true;
  }

  const auto& next_wp = pending_waypoints.front();
  assert(next_wp.graph_index().has_value());

  const auto depart_it = departure_timing.find(checkpoint_id);
  if (depart_it == departure_timing.end())
    return true;

  // TODO(MXG): Make this configurable
  const auto timing_threshold = std::chrono::seconds(1);
  const auto ready_time = depart_it->second;
  const auto now = node->now();

  if (now + timing_threshold <= ready_time)
  {
    std::cout << "Next ready time is in "
              << rmf_traffic::time::to_seconds(
                   (ready_time - now).to_chrono<std::chrono::nanoseconds>())
              << " seconds" << std::endl;
    return false;
  }

  for (auto it = depart_it; it != departure_timing.end(); ++it)
  {
    // TODO(MXG): It would probably be better to reverse iterate here
    if (it->second <= now + timing_threshold)
    {
      std::cout << "Setting [" << it->first << "] to ready for "
                << itinerary.description().name() << "!!" << std::endl;
      blockade.ready(it->first);
    }
  }

  return true;
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::check_waiting_delay(
    const std::size_t version,
    const std::size_t checkpoint_id)
{
  std::cout << "Checking on the waiting delay" << std::endl;
  if (version != current_version)
    return;

  std::cout << "Make it to " << __LINE__ << std::endl;
  if (pending_waypoints.empty())
    return;
  std::cout << "Make it to " << __LINE__ << std::endl;

  assert(pending_waypoints.front().graph_index().has_value());
  if (checkpoint_id != pending_waypoints.front().graph_index().value())
    return;
  std::cout << "Make it to " << __LINE__ << std::endl;

  const auto now = node->now();
  const auto new_delay = now - departure_timing.at(checkpoint_id);
  const auto time_shift = (new_delay - itinerary.delay())
      .to_chrono<std::chrono::nanoseconds>();
  std::cout << "Time shift: " << rmf_traffic::time::to_seconds(time_shift) << std::endl;
  if (time_shift > std::chrono::seconds(1))
    itinerary.delay(time_shift);
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::send_checkpoints(
    std::vector<CommandHandle::Checkpoint> checkpoints)
{
  waiting_for_departure = false;
  const std::size_t checkpoint_id = checkpoints.back().waypoint_index + 1;
  command->receive_checkpoints(
        current_version,
        std::move(checkpoints),
        [w = weak_from_this(),
         version = current_version,
         checkpoint_id]()
  {
    if (const auto data = w.lock())
    {
      data->worker.schedule(
            [w = data->weak_from_this(),
             version,
             checkpoint_id](const auto&)
      {
        if (const auto data = w.lock())
          data->watch_for_ready(version, checkpoint_id);
      });
    }
  });
}

//==============================================================================
class TrafficLight::UpdateHandle::Implementation::Negotiator
    : public rmf_traffic::schedule::Negotiator
{
public:

  Negotiator(std::shared_ptr<Data> data)
    : _data(std::move(data))
  {
    // Do nothing
  }

  void respond(
      const TableViewerPtr& table_viewer,
      const ResponderPtr& responder) final;

private:
  std::weak_ptr<Data> _data;
};

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Negotiator::respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder)
{
  const auto data = _data.lock();
  if (!data || !data->planner || data->pending_waypoints.empty())
  {
    // If we no longer have access to the traffic light data or there is no
    // plan being followed, then we simply forfeit the negotiation.
    return responder->forfeit({});
  }

  rmf_traffic::schedule::StubbornNegotiator(data->itinerary)
      .respond(table_viewer, responder);
}

//==============================================================================
rmf_traffic::blockade::Participant make_blockade(
    rmf_traffic_ros2::blockade::Writer& writer,
    const rmf_traffic::schedule::Participant& itinerary,
    TrafficLight::UpdateHandle::Implementation* impl)
{
  const double radius = itinerary.description().profile()
      .vicinity()->get_characteristic_length();

  auto new_range_cb =
      [impl](
      const rmf_traffic::blockade::ReservationId reservation,
      const rmf_traffic::blockade::ReservedRange& range)
  {
    impl->new_range(reservation, range);
  };

  return writer.make_participant(
        itinerary.id(), radius, std::move(new_range_cb));
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::new_range(
    const rmf_traffic::blockade::ReservationId reservation_id,
    const rmf_traffic::blockade::ReservedRange& new_range)
{
  assert(data);
  data->new_range(reservation_id, new_range);
}

//==============================================================================
TrafficLight::UpdateHandle::Implementation::Implementation(
    std::shared_ptr<CommandHandle> command_,
    rmf_traffic::schedule::Participant itinerary_,
    std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer,
    rmf_traffic::agv::VehicleTraits traits_,
    std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<rclcpp::Node> node_)
  : data(std::make_shared<Data>(
           std::move(command_),
           make_blockade(*blockade_writer, itinerary_, this),
           std::move(itinerary_),
           std::move(traits_),
           std::move(schedule_),
           std::move(worker_),
           std::move(node_)))
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<TrafficLight::UpdateHandle>
TrafficLight::UpdateHandle::Implementation::make(
    std::shared_ptr<CommandHandle> command,
    rmf_traffic::schedule::Participant itinerary,
    std::shared_ptr<rmf_traffic_ros2::blockade::Writer> blockade_writer,
    rmf_traffic::agv::VehicleTraits traits,
    std::shared_ptr<rmf_traffic::schedule::Snappable> schedule,
    rxcpp::schedulers::worker worker,
    std::shared_ptr<rclcpp::Node> node,
    rmf_traffic_ros2::schedule::Negotiation* negotiation)
{
  std::shared_ptr<UpdateHandle> handle = std::make_shared<UpdateHandle>();
  handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
        std::move(command),
        std::move(itinerary),
        std::move(blockade_writer),
        std::move(traits),
        std::move(schedule),
        std::move(worker),
        std::move(node));

  if (negotiation)
  {
    handle->_pimpl->negotiation_license = negotiation->register_negotiator(
          handle->_pimpl->data->itinerary.id(),
          std::make_unique<Negotiator>(handle->_pimpl->data));
  }

  return handle;
}

//==============================================================================
std::size_t TrafficLight::UpdateHandle::follow_new_path(
    const std::vector<Waypoint>& new_path)
{
  const std::size_t version = ++_pimpl->received_version;
  _pimpl->data->worker.schedule(
        [version, new_path, data = _pimpl->data](const auto&)
  {
    data->update_path(version, new_path);
  });

  return version;
}

} // namespace agv
} // namespace rmf_fleet_adapter
