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

#ifndef SRC__FULL_CONTROL__SCHEDULEMANAGER_HPP
#define SRC__FULL_CONTROL__SCHEDULEMANAGER_HPP

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/schedule/Version.hpp>

#include "Listener.hpp"

#include <rmf_traffic_ros2/schedule/Writer.hpp>

#include <rmf_traffic_msgs/srv/temporary_resolve_conflicts.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict.hpp>

#include <rclcpp/node.hpp>

#include <unordered_set>

namespace rmf_fleet_adapter {

//==============================================================================
// TODO(MXG): Move this into rmf_traffic_ros2 as a generalized utility class.
// Consider renaming it to ScheduleParticipant.
class ScheduleManager
{
public:

  ScheduleManager(
      rclcpp::Node& node,
      rmf_traffic::schedule::Participant participant,
      std::function<void()> revision_callback);

  using TrajectorySet = std::vector<rmf_traffic::Trajectory>;

  void push_routes(
      const std::vector<rmf_traffic::Route>& routes,
      std::function<void()> approval_callback);

  void push_delay(
      const rmf_traffic::Duration duration,
      const rmf_traffic::Time from_time);

  void set_revision_callback(
      std::function<void()> revision_callback);

  rmf_traffic::schedule::ParticipantId participant_id() const;

private:

  void resolve_trajectories(
      std::vector<rmf_traffic::Route> routes,
      std::function<void()> approval_callback);

  rmf_traffic::schedule::Participant _participant;

  using ResolveConflicts = rmf_traffic_msgs::srv::TemporaryResolveConflicts;
  using ScheduleConflict = rmf_traffic_msgs::msg::ScheduleConflict;

  rclcpp::Client<ResolveConflicts>::SharedPtr _resolve_client;

  rclcpp::Subscription<rmf_traffic_msgs::msg::ScheduleConflict>::SharedPtr _conflict_sub;

  std::function<void()> _revision_callback;
  rmf_traffic::schedule::Version _conflict_version;
  bool _have_conflict = false;
};

//==============================================================================
std::future<ScheduleManager> make_schedule_manager(
    rclcpp::Node& node,
    rmf_traffic_ros2::schedule::Writer& writer,
    rmf_traffic::schedule::ParticipantDescription description,
    std::function<void()> revision_callback);

//==============================================================================
void async_make_schedule_manager(
    rclcpp::Node& node,
    rmf_traffic_ros2::schedule::Writer& writer,
    rmf_traffic::schedule::ParticipantDescription description,
    std::function<void()> revision_callback,
    std::function<void(ScheduleManager manager)> ready_callback);

} // namespace rmf_fleet_adapter


#endif // SRC__FULL_CONTROL__SCHEDULEMANAGER_HPP
