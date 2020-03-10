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

#include "ScheduleManager.hpp"

#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

namespace rmf_fleet_adapter {

namespace {
//==============================================================================
std::vector<rmf_traffic_msgs::msg::Trajectory> convert(
    const std::vector<const rmf_traffic::Trajectory*>& trajectories)
{
  std::vector<rmf_traffic_msgs::msg::Trajectory> output;
  output.reserve(trajectories.size());
  for (const auto& trajectory : trajectories)
    output.emplace_back(rmf_traffic_ros2::convert(*trajectory));

  return output;
}

} // anonymous namespace

//==============================================================================
ScheduleManager::ScheduleManager(
    rclcpp::Node& node,
    rmf_traffic::schedule::Participant participant,
    std::function<void()> revision_callback)
  : _participant(std::move(participant)),
    _revision_callback(std::move(revision_callback))
{

  auto resolve_client =
      node.create_client<rmf_traffic_msgs::srv::TemporaryResolveConflicts>(
        rmf_traffic_ros2::ResolveConflictsSrvName);

  auto conflict_sub =
      node.create_subscription<rmf_traffic_msgs::msg::ScheduleConflict>(
        rmf_traffic_ros2::ScheduleConflictTopicName,
        rclcpp::SystemDefaultsQoS().reliable(),
        [this,
         p = _participant.id()](
        const std::unique_ptr<rmf_traffic_msgs::msg::ScheduleConflict> msg)
  {
    for (const auto m : msg->participants)
    {
      if (m == p)
      {
        this->_have_conflict = true;
        this->_conflict_version = msg->conflict_version;
        this->_revision_callback();
        return;
      }
    }
  });
}

//==============================================================================
void ScheduleManager::push_routes(
    const std::vector<rmf_traffic::Route>& routes,
    std::function<void()> approval_callback)
{
  // TODO(MXG): Be smarter here. If there are no trajectories then erase the
  // current schedule? Or have the robot stand in place?
  std::vector<rmf_traffic::Route> valid_routes;
  valid_routes.reserve(routes.size());
  for (const auto& r : routes)
  {
    if (r.trajectory().size() < 2)
      continue;

    valid_routes.push_back(r);
  }

  // If there are no valid trajectories to push to the schedule, then erase the
  // current trajectories from the schedule and approve.
  // TODO(MXG): Consider putting some debug output here.
  if (valid_routes.empty())
  {
    _participant.clear();
    approval_callback();
    return;
  }

  if (_have_conflict)
  {
    return resolve_trajectories(
          std::move(valid_routes),
          std::move(approval_callback));
  }

  _participant.set(std::move(valid_routes));
  approval_callback();
}

//==============================================================================
void ScheduleManager::push_delay(
    const rmf_traffic::Duration duration,
    const rmf_traffic::Time from_time)
{
  if (_have_conflict)
    return;

  _participant.delay(from_time, duration);
}

//==============================================================================
void ScheduleManager::set_revision_callback(
    std::function<void()> revision_callback)
{
  _revision_callback = std::move(revision_callback);
}

//==============================================================================
rmf_traffic::schedule::ParticipantId ScheduleManager::participant_id() const
{
  return _participant.id();
}

//==============================================================================
void ScheduleManager::resolve_trajectories(
    std::vector<rmf_traffic::Route> routes,
    std::function<void()> approval_callback)
{
  ResolveConflicts::Request request;
  request.participant = _participant.id();
  request.conflict_version = _conflict_version;

  _resolve_client->async_send_request(
        std::make_shared<ResolveConflicts::Request>(request),
        [approval_cb = std::move(approval_callback),
         routes = std::move(routes),
         this](
        const rclcpp::Client<ResolveConflicts>::SharedFuture future)
  {
    const auto response = future.get();

    if (!response->error.empty())
    {
      throw std::runtime_error(
          "Error while attempting to resolve a conflict: " + response->error);
    }

    if (!response->accepted)
      return;

    this->_participant.set(routes);
    approval_cb();
  });
}

//==============================================================================
std::future<ScheduleManager> make_schedule_manager(
    rclcpp::Node& node,
    rmf_traffic_ros2::schedule::Writer& writer,
    rmf_traffic::schedule::ParticipantDescription description,
    std::function<void()> revision_callback)
{
  return std::async(
        std::launch::async,
        [&node,
         &writer,
         description = std::move(description),
         revision_callback = std::move(revision_callback)]() -> ScheduleManager
  {
    return ScheduleManager(
          node,
          writer.make_participant(std::move(description)).get(),
          std::move(revision_callback));
  });
}

//==============================================================================
void async_make_schedule_manager(
    rclcpp::Node& node,
    rmf_traffic_ros2::schedule::Writer& writer,
    rmf_traffic::schedule::ParticipantDescription description,
    std::function<void()> revision_callback,
    std::function<void(ScheduleManager)> ready_callback)
{
  writer.async_make_participant(
        std::move(description),
        [&node,
         revision_callback = std::move(revision_callback),
         ready_callback = std::move(ready_callback)](
        rmf_traffic::schedule::Participant participant)
  {
    ready_callback(
          ScheduleManager{
            node,
            std::move(participant),
            std::move(revision_callback)
          });
  });
}

} // namespace rmf_fleet_adapter
