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
void ScheduleManager::push_trajectories(
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
      throw std::runtime_error(response->error);

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
  auto resolve_client =
      node.create_client<rmf_traffic_msgs::srv::TemporaryResolveConflicts>(
        rmf_traffic_ros2::ResolveConflictsSrvName);


  return std::async(
        std::launch::async,
        [&node,
         &writer,
         resolve_client = std::move(resolve_client),
         description = std::move(description),
         revision_callback = std::move(revision_callback)]() -> ScheduleManager
  {
    auto participant_future = writer.make_participant(std::move(description));
    participant_future.wait();

    auto participant = participant_future.get();

    auto conflict_sub =
        node.create_subscription<rmf_traffic_msgs::msg::ScheduleConflict>(
          rmf_traffic_ros2::ScheduleConflictTopicName,
          rclcpp::SystemDefaultsQoS().reliable(),
          [revision_callback = std::move(revision_callback),
           p = participant.id()](
          const std::unique_ptr<rmf_traffic_msgs::msg::ScheduleConflict> msg)
    {
      for (const auto m : msg->participants)
      {
        if (m == p)
        {
          // TODO(MXG): Fix this. ScheduleManager needs to be informed that a
          // conflict is active
          revision_callback();
          return;
        }
      }
    });

    return ScheduleManager(
          std::move(participant),
          std::move(resolve_client),
          std::move(conflict_sub));
  });
}

} // namespace rmf_fleet_adapter
