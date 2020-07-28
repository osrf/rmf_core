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

#include <rmf_utils/Modular.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_traffic/Motion.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TrafficLight::UpdateHandle::Implementation::Data
    : std::enable_shared_from_this<Data>
{
public:

  std::shared_ptr<CommandHandle> command;

  rmf_traffic::schedule::Participant itinerary;

  rmf_traffic::agv::VehicleTraits traits;

  std::shared_ptr<rmf_traffic::schedule::Snappable> schedule;

  rxcpp::schedulers::worker worker;

  std::shared_ptr<rclcpp::Node> node;

  std::size_t current_version = 0;

  std::shared_ptr<rmf_traffic::Profile> profile;

  std::vector<Waypoint> path;

  rmf_utils::optional<rmf_traffic::agv::Plan> plan;

  std::shared_ptr<rmf_traffic::agv::Planner> planner;

  std::vector<rmf_traffic::Time> arrival_timing;
  std::vector<rclcpp::Time> departure_timing;

  std::size_t processing_version = 0;

  std::shared_ptr<services::FindPath> find_path_service;
  rxcpp::subscription find_path_subscription;

  void update_path(
      std::size_t version,
      const std::vector<Waypoint>& new_path);

  void update_timing(
      std::size_t version,
      std::vector<Waypoint> path_,
      rmf_traffic::agv::Plan plan_,
      std::shared_ptr<rmf_traffic::agv::Planner> planner_);

  Data(
      std::shared_ptr<CommandHandle> command_,
      rmf_traffic::schedule::Participant itinerary_,
      rmf_traffic::agv::VehicleTraits traits_,
      std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
      rxcpp::schedulers::worker worker_,
      std::shared_ptr<rclcpp::Node> node_)
    : command(std::move(command_)),
      itinerary(std::move(itinerary_)),
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

  const bool clear_itinerary = new_path.empty()
      || (new_path.size() == 1
          && new_path.front().mandatory_delay() <= std::chrono::nanoseconds(0));

  const auto now = rmf_traffic_ros2::convert(node->now());

  if (clear_itinerary)
  {
    // Clear out this robot's itinerary because it is done moving
    RCLCPP_INFO(
      node->get_logger(),
      "Traffic light controlled robot [%s] owned by [%s] is being taken off "
      "the schedule after being given a null path.",
      itinerary.description().name().c_str(),
      itinerary.description().owner().c_str());

    plan = rmf_utils::nullopt;
    planner = nullptr;
    path.clear();
    departure_timing.clear();
    itinerary.clear();
    return;
  }
  else if (new_path.size() == 1)
  {
    // Have the robot sit in place if it was given a single-point trajectory
    // with a mandatory delay
    plan = rmf_utils::nullopt;
    planner = nullptr;

    rmf_traffic::Trajectory sit;
    sit.insert(
      now,
      new_path.front().position(),
      Eigen::Vector3d::Zero());

    sit.insert(
      now + new_path.front().mandatory_delay(),
      new_path.front().position(),
      Eigen::Vector3d::Zero());

    itinerary.set({{new_path.front().map_name(), sit}});
  }

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
              rmf_traffic::agv::Graph::Lane::DoorOpen(
                "", last_wp.mandatory_delay()));
      }

      graph.add_lane(rmf_traffic::agv::Graph::Lane::Node(i-1, event), i);
    }
  }

  auto new_planner = std::make_shared<rmf_traffic::agv::Planner>(
        rmf_traffic::agv::Plan::Configuration(graph, traits),
        rmf_traffic::agv::Plan::Options(nullptr));

  rmf_traffic::agv::Plan::Start start{now, 0, new_path.front().position()[2]};

  find_path_service = std::make_shared<services::FindPath>(
    new_planner, rmf_traffic::agv::Plan::StartSet({std::move(start)}),
    graph.num_waypoints()-1, schedule->snapshot(), itinerary.id(), profile);

  find_path_subscription = rmf_rxcpp::make_job<services::FindPath::Result>(
        find_path_service)
      .observe_on(rxcpp::identity_same_worker(worker))
      .subscribe(
        [w = weak_from_this(), version, new_path, new_planner](
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

  const double r = (p - p0).norm();
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

  assert(false);
  return rmf_utils::nullopt;
}

//==============================================================================
rmf_traffic::Time interpolate_time(
  const rmf_traffic::agv::VehicleTraits& traits,
  const rmf_traffic::agv::Plan::Waypoint& wp0,
  const rmf_traffic::agv::Plan::Waypoint& wp1,
  const Eigen::Vector2d& p)
{
  const auto interp = rmf_traffic::agv::Interpolate::positions(
        traits, wp0.time(), {wp0.position(), wp1.position()});

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

  assert(false);
  throw std::runtime_error(
    "[rmf_fleet_adapter::agv::TrafficLight] Failed to interpolate an "
    "intermediary waypoint.");
}
} // anonymous namespace

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::update_timing(
    const std::size_t version,
    std::vector<Waypoint> path_,
    rmf_traffic::agv::Plan plan_,
    std::shared_ptr<rmf_traffic::agv::Planner> planner_)
{
  if (version != current_version)
    return;

  path = std::move(path_);
  plan = std::move(plan_);
  planner = std::move(planner_);

  assert(!path.empty());

  rclcpp::Time last_t =
      rmf_traffic_ros2::convert(plan->get_waypoints().front().time());

  const auto& waypoints = plan->get_waypoints();
  const auto& graph = planner->get_configuration().graph();

  arrival_timing.resize(path.size(), rmf_traffic_ros2::convert(last_t));
  departure_timing.resize(path.size(), last_t);

  std::size_t current_wp = 0;

  for (std::size_t i=0; i < waypoints.size(); ++i)
  {
    const auto& wp = waypoints[i];

    assert(wp.graph_index());
    if (*wp.graph_index() != current_wp)
    {
      const std::size_t next_wp = *wp.graph_index();
      for (std::size_t k=current_wp+1; k < next_wp; ++k)
      {
        // For any intermediate waypoints that will be passed over without
        // stopping, we should try to calculate the time that the robot is
        // expected to pass over it.
        const auto p = graph.get_waypoint(k).get_location();
        const auto t = interpolate_time(traits, waypoints[i-1], wp, p);
        arrival_timing[k] = t;
        departure_timing[k] = rmf_traffic_ros2::convert(t);
      }

      current_wp = *wp.graph_index();
      arrival_timing[current_wp] = wp.time();
    }

    // This might get overriden several times since a single path waypoint might
    // show up several times in a plan. We want to use the last time that is
    // associated with this path waypoint.
    departure_timing[current_wp] = rmf_traffic_ros2::convert(wp.time());
  }

  command->receive_path_timing(
    current_version, departure_timing,
    [w = weak_from_this(), current_version = current_version](
        const std::size_t path_index, rmf_traffic::Duration remaining_time)
  {
    const auto data = w.lock();
    if (!data)
      return;

    const rmf_traffic::Time estimated_time =
        rmf_traffic_ros2::convert(data->node->now() + remaining_time);

    data->worker.schedule(
          [w = std::weak_ptr<Data>(data),
           current_version,
           path_index,
           estimated_time](const auto&)
    {
      const auto data = w.lock();
      if (!data)
        return;

      if (current_version != data->current_version)
        return;

      assert(path_index < data->arrival_timing.size());

      const auto new_delay = estimated_time - data->arrival_timing[path_index];
      const auto time_shift = new_delay - data->itinerary.delay();
      if (time_shift > std::chrono::seconds(5))
        data->itinerary.delay(time_shift);
    });
  });
}

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
  if (!data || !data->planner || !data->plan)
  {
    // If we no longer have access to the traffic light data or there is no
    // plan being followed, then we simply forfeit the negotiation.
    return responder->forfeit({});
  }

  // FIXME TODO(MXG): Reply to the negotiation here.
  assert(false);
}

//==============================================================================s
TrafficLight::UpdateHandle::Implementation::Implementation(
    std::shared_ptr<CommandHandle> command_,
    rmf_traffic::schedule::Participant itinerary_,
    rmf_traffic::agv::VehicleTraits traits_,
    std::shared_ptr<rmf_traffic::schedule::Snappable> schedule_,
    rxcpp::schedulers::worker worker_,
    std::shared_ptr<rclcpp::Node> node_)
  : data(std::make_shared<Data>(
           std::move(command_),
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
    rmf_traffic::agv::VehicleTraits traits,
    std::shared_ptr<rmf_traffic::schedule::Snappable> schedule,
    rxcpp::schedulers::worker worker,
    std::shared_ptr<rclcpp::Node> node,
    rmf_traffic_ros2::schedule::Negotiation& negotiation)
{
  std::shared_ptr<UpdateHandle> handle = std::make_shared<UpdateHandle>();
  handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
        std::move(command),
        std::move(itinerary),
        std::move(traits),
        std::move(schedule),
        std::move(worker),
        std::move(node));

  handle->_pimpl->negotiation_license = negotiation.register_negotiator(
        handle->_pimpl->data->itinerary.id(),
        std::make_unique<Negotiator>(handle->_pimpl->data));

  return handle;
}

//==============================================================================
std::size_t TrafficLight::UpdateHandle::update_path(
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
