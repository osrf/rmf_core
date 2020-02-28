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

#include "ScheduleNode.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>
#include <rmf_traffic_ros2/schedule/Patch.hpp>

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_traffic_schedule {

//==============================================================================
std::unordered_set<rmf_traffic::schedule::Version> get_conflicts(
    const rmf_traffic::schedule::Viewer::View& view)
{
  // TODO(MXG): Make this function more efficient by only checking the latest
  // unchecked changes against the ones that came before them, and then
  // appending that list onto the conflicts of the previous version.

  std::unordered_set<rmf_traffic::schedule::Version> conflicts;
  for (auto v0 = view.begin(); v0 != view.end(); ++v0)
  {
    auto v1 = ++rmf_traffic::schedule::Viewer::View::const_iterator{v0};
    for (; v1 != view.end(); ++v1)
    {
      if (v0->participant == v1->participant)
        continue;

      if (v0->route.map() != v1->route.map())
        continue;

      if (rmf_traffic::DetectConflict::between(
            v0->description.profile(),
            v0->route.trajectory(),
            v1->description.profile(),
            v1->route.trajectory()))
      {
        conflicts.insert(v0->participant);
        conflicts.insert(v1->participant);
      }
    }
  }

  return conflicts;
}

//==============================================================================
ScheduleNode::ScheduleNode()
  : Node("rmf_traffic_schedule_node")
{
  // TODO(MXG): As soon as possible, all of these services should be made
  // multi-threaded so they can be parallel processed.

  register_query_service =
      create_service<RegisterQuery>(
        rmf_traffic_ros2::RegisterQueryServiceName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const RegisterQuery::Request::SharedPtr request,
            const RegisterQuery::Response::SharedPtr response)
        { this->register_query(request_header, request, response); });

  unregister_query_service =
      create_service<UnregisterQuery>(
        rmf_traffic_ros2::UnregisterQueryServiceName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const UnregisterQuery::Request::SharedPtr request,
            const UnregisterQuery::Response::SharedPtr response)
        { this->unregister_query(request_header, request, response); });

  mirror_update_service =
      create_service<MirrorUpdate>(
        rmf_traffic_ros2::MirrorUpdateServiceName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const MirrorUpdate::Request::SharedPtr request,
            const MirrorUpdate::Response::SharedPtr response)
        { this->mirror_update(request_header, request, response); });

  mirror_wakeup_publisher =
      create_publisher<MirrorWakeup>(
        rmf_traffic_ros2::MirrorWakeupTopicName,
        rclcpp::SystemDefaultsQoS());

  conflict_publisher =
      create_publisher<ScheduleConflict>(
        rmf_traffic_ros2::ScheduleConflictTopicName,
        rclcpp::SystemDefaultsQoS());

  conflict_check_quit = false;
  conflict_check_thread = std::thread(
        [&]()
  {
    rmf_traffic::schedule::Mirror mirror;
    const auto query_all = rmf_traffic::schedule::query_all();
    Version last_checked_version = 0;

    while (rclcpp::ok() && !conflict_check_quit)
    {
      rmf_utils::optional<rmf_traffic::schedule::Patch> next_patch;

      // Use this scope to minimize how long we lock the database for
      {
        std::unique_lock<std::mutex> lock(database_mutex);
        conflict_check_cv.wait_for(lock, std::chrono::milliseconds(100), [&]()
        {
          return (database.latest_version() > last_checked_version)
              && !conflict_check_quit;
        });

        if (database.latest_version() == last_checked_version
            || conflict_check_quit)
        {
          // This is a casual wakeup to check if we're supposed to quit yet
          continue;
        }

        next_patch = database.changes(query_all, last_checked_version);

        // TODO(MXG): Check whether the database really needs to remain locked
        // during this update.
        try
        {
          mirror.update(*next_patch);
          last_checked_version = next_patch->latest_version();
        }
        catch(const std::exception& e)
        {
          RCLCPP_ERROR(get_logger(), e.what());
          continue;
        }
      }

      // TODO(MXG): If we only look at the participants that have changed since
      // the last patch, this could be much more efficient.

      const auto view = mirror.query(rmf_traffic::schedule::query_all());

      const auto conflicts = get_conflicts(view);
      if (!conflicts.empty())
      {
        {
          std::unique_lock<std::mutex> lock(active_conflicts_mutex);
          active_conflicts.insert(
                std::make_pair(last_checked_version, conflicts));
        }

        ScheduleConflict msg;
        for (const auto c : conflicts)
          msg.indices.push_back(c);

        msg.version = last_checked_version;

        conflict_publisher->publish(std::move(msg));
      }
      else
      {
        bool had_conflicts = false;

        {
          std::unique_lock<std::mutex> lock(active_conflicts_mutex);
          had_conflicts = !active_conflicts.empty();
          active_conflicts.clear();
        }

        if (had_conflicts)
        {
          ScheduleConflict msg;
          msg.version = last_checked_version;
          conflict_publisher->publish(std::move(msg));
        }
      }
    }
  });
}

//==============================================================================
ScheduleNode::~ScheduleNode()
{
  conflict_check_quit = true;
  if (conflict_check_thread.joinable())
    conflict_check_thread.join();
}

//==============================================================================
void ScheduleNode::register_query(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const RegisterQuery::Request::SharedPtr& request,
    const RegisterQuery::Response::SharedPtr& response)
{
  uint64_t query_id = last_query_id;
  uint64_t attempts = 0;
  do
  {
    ++query_id;
    ++attempts;
    if(attempts == std::numeric_limits<uint64_t>::max())
    {
      // I suspect a computer would run out of RAM before we reach this point,
      // but there's no harm in double-checking.
      response->error = "No more space for additional queries to be registered";
      RCLCPP_ERROR(
            get_logger(),
            "[ScheduleNode::register_query] " + response->error);
      return;
    }
  } while(registered_queries.find(query_id) != registered_queries.end());

  last_query_id = query_id;
  registered_queries.insert(
        std::make_pair(query_id, rmf_traffic_ros2::convert(request->query)));

  response->query_id = query_id;
  RCLCPP_INFO(
        get_logger(),
        "[" + std::to_string(query_id) + "] Registered query");
}

//==============================================================================
void ScheduleNode::unregister_query(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const UnregisterQuery::Request::SharedPtr& request,
    const UnregisterQuery::Response::SharedPtr& response)
{
  const auto it = registered_queries.find(request->query_id);
  if(it == registered_queries.end())
  {
    response->error = "No query found with the id ["
        + std::to_string(request->query_id) + "]";
    response->confirmation = false;

    RCLCPP_WARN(
          get_logger(),
          "[ScheduleNode::unregister_query] " + response->error);
    return;
  }

  registered_queries.erase(it);
  response->confirmation = true;

  RCLCPP_INFO(
        get_logger(),
        "[" + std::to_string(request->query_id) + "] Unregistered query");
}

//==============================================================================
void ScheduleNode::mirror_update(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const MirrorUpdate::Request::SharedPtr& request,
    const MirrorUpdate::Response::SharedPtr& response)
{
  const auto query_it = registered_queries.find(request->query_id);
  if(query_it == registered_queries.end())
  {
    response->error = "Unrecognized query_id: "
        + std::to_string(request->query_id);
    RCLCPP_WARN(
          get_logger(),
          "[ScheduleNode::mirror_update] " + response->error);
    return;
  }

  rmf_utils::optional<rmf_traffic::schedule::Version> version;
  if (!request->initial_request)
    version = request->latest_mirror_version;

  response->patch =
      rmf_traffic_ros2::convert(database.changes(query_it->second, version));
}

//==============================================================================
void ScheduleNode::wakeup_mirrors()
{
  rmf_traffic_msgs::msg::MirrorWakeup msg;
  msg.latest_version = database.latest_version();
  mirror_wakeup_publisher->publish(msg);

  conflict_check_cv.notify_all();
}

} // namespace rmf_traffic_schedule
