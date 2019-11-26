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

#include <rmf_traffic/Conflict.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_traffic_schedule {

//==============================================================================
std::unordered_set<rmf_traffic::schedule::Version> get_conflicts(
    const rmf_traffic::schedule::Viewer::View& view)
{
  std::unordered_set<rmf_traffic::schedule::Version> conflicts;
  for (auto v0 = view.begin(); v0 != view.end(); ++v0)
  {
    auto v1 = ++rmf_traffic::schedule::Viewer::View::const_iterator{v0};
    for (; v1 != view.end(); ++v1)
    {
      if (!rmf_traffic::DetectConflict::between(
            v0->trajectory, v1->trajectory, true).empty())
      {
        conflicts.insert(v0->id);
        conflicts.insert(v1->id);
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

  submit_trajectories_service =
      create_service<rmf_traffic_msgs::srv::SubmitTrajectories>(
        rmf_traffic_ros2::SubmitTrajectoriesSrvName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const SubmitTrajectories::Request::SharedPtr request,
            const SubmitTrajectories::Response::SharedPtr response)
        { this->submit_trajectories(request_header, request, response); });

  replace_trajectories_service =
      create_service<ReplaceTrajectories>(
        rmf_traffic_ros2::ReplaceTrajectoriesSrvName,
        [=](const request_id_ptr request_header,
            const ReplaceTrajectories::Request::SharedPtr request,
            const ReplaceTrajectories::Response::SharedPtr response)
        { this->replace_trajectories(request_header, request, response); });

  delay_trajectories_service =
      create_service<DelayTrajectories>(
        rmf_traffic_ros2::DelayTrajectoriesSrvName,
        [=](const request_id_ptr request_header,
            const DelayTrajectories::Request::SharedPtr request,
            const DelayTrajectories::Response::SharedPtr response)
        { this->delay_trajectories(request_header, request, response); });

  erase_trajectories_service =
      create_service<EraseTrajectories>(
        rmf_traffic_ros2::EraseTrajectoriesSrvName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const EraseTrajectories::Request::SharedPtr request,
            const EraseTrajectories::Response::SharedPtr response)
        { this->erase_trajectories(request_header, request, response); });

  resolve_trajectories_service =
      create_service<ResolveTrajectories>(
        rmf_traffic_ros2::ResolveTrajectoriesSrvName,
        [=](const std::shared_ptr<rmw_request_id_t> request_header,
            const ResolveTrajectories::Request::SharedPtr request,
            const ResolveTrajectories::Response::SharedPtr response)
        { this->resolve_trajectories(request_header, request, response); });

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

    Version last_checked_version = 0;

    while (rclcpp::ok() && !conflict_check_quit)
    {
      const auto next_query =
          rmf_traffic::schedule::make_query(last_checked_version);
      rmf_utils::optional<rmf_traffic::schedule::Database::Patch> next_patch;

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

        next_patch = database.changes(next_query);

        // TODO(MXG): Check whether the database really needs to remain locked
        // during this update.
        mirror.update(*next_patch);
      }

      last_checked_version = mirror.latest_version();

      const auto view = mirror.query(
            rmf_traffic::schedule::query_everything());

      const auto conflicts = get_conflicts(view);
      if (!conflicts.empty())
      {
        {
          std::unique_lock<std::mutex> lock(active_conflicts_mutex);
          active_conflicts.insert(
                std::make_pair(mirror.latest_version(), conflicts));
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
void insert_conflicts(
    const std::vector<uint64_t>& conflicts,
    rmf_traffic_msgs::srv::SubmitTrajectories::Response& response)
{
  if(!conflicts.empty())
  {
    response.accepted = false;
    response.conflicts = conflicts;
  }
}

//==============================================================================
void ScheduleNode::submit_trajectories(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const SubmitTrajectories::Request::SharedPtr& request,
    const SubmitTrajectories::Response::SharedPtr& response)
{
  response->accepted = true;
  response->current_version = database.latest_version();
  response->original_version = response->current_version;
  response->error.clear();

  std::vector<rmf_traffic::Trajectory> requested_trajectories;
  requested_trajectories.reserve(request->trajectories.size());
  std::vector<uint64_t> conflicting_indices;
  conflicting_indices.reserve(request->trajectories.size());
  for(std::size_t i=0; i < request->trajectories.size(); ++i)
  {
    const auto& msg = request->trajectories[i];

    bool valid_trajectory = true;
    rmf_traffic::Trajectory requested_trajectory =
        [&]() -> rmf_traffic::Trajectory
    {
      try
      {
        return rmf_traffic_ros2::convert(msg);
      }
      catch(const std::exception& e)
      {
        valid_trajectory = false;
        response->accepted = false;
        response->error = e.what();
      }

      return {""};
    }();

    if(!valid_trajectory)
      return;

    if(requested_trajectory.size() < 2)
    {
      response->error = "Invalid trajectory at index [" + std::to_string(i)
          + "]: Only [" + std::to_string(requested_trajectory.size())
          + "] segments; minimum is 2.";
      RCLCPP_ERROR(
            get_logger(),
            "[ScheduleNode::submit_trajectory] " + response->error);
      return;
    }

    const auto view = database.query(rmf_traffic::schedule::make_query(
                     {requested_trajectory.get_map_name()},
                     requested_trajectory.start_time(),
                     requested_trajectory.finish_time()));

    for(const auto& v : view)
    {
      // TODO(MXG) Since we already put these time limits in the query, do we
      // really need to check them again here?
      if(*requested_trajectory.finish_time() < *v.trajectory.start_time())
        continue;

      if(*v.trajectory.finish_time() < *requested_trajectory.start_time())
        continue;

      const auto conflicts = rmf_traffic::DetectConflict::narrow_phase(
            requested_trajectory, v.trajectory, true);
      if (!conflicts.empty())
        conflicting_indices.push_back(i);
    }

    requested_trajectories.emplace_back(std::move(requested_trajectory));
  }

  insert_conflicts(conflicting_indices, *response);
  if(!response->accepted)
    return;

  // TODO(MXG): Should there be constraints on trajectory submissions to avoid
  // this kind of check? Like each submission can only refer to one vehicle at
  // a time, and therefore we should never need to test these trajectories for
  // conflicts with each other?
  for(std::size_t i=0; i < requested_trajectories.size()-1; ++i)
  {
    for(std::size_t j=i+1; j < requested_trajectories.size(); ++j)
    {
      const auto conflicts = rmf_traffic::DetectConflict::between(
            requested_trajectories[i],
            requested_trajectories[j], true);
      if (!conflicts.empty())
        conflicting_indices.push_back(i);
    }
  }

  insert_conflicts(conflicting_indices, *response);
  if(!response->accepted)
    return;

  {
    std::unique_lock<std::mutex> lock(database_mutex);
    for(auto&& request : requested_trajectories)
      database.insert(std::move(request));
  }

  response->current_version = database.latest_version();
  wakeup_mirrors();

  RCLCPP_DEBUG(
        get_logger(),
        "Received trajectory [" + std::to_string(response->current_version)
        + "]");
}

//==============================================================================
void ScheduleNode::replace_trajectories(
    const request_id_ptr& /*request_header*/,
    const ReplaceTrajectories::Request::SharedPtr& request,
    const ReplaceTrajectories::Response::SharedPtr& response)
{
  response->original_version = database.latest_version();
  response->current_version = response->original_version;
  if (request->replace_ids.size() == 0)
  {
    RCLCPP_WARN(
          get_logger(),
          "Replace trajectory request has no replace IDs in it!");
  }

  std::vector<rmf_traffic::Trajectory> trajectories;
  trajectories.reserve(request->trajectories.size());
  for (std::size_t i=0; i < request->trajectories.size(); ++i)
  {
    try
    {
      trajectories.emplace_back(
            rmf_traffic_ros2::convert(request->trajectories[i]));
    }
    catch(const std::exception& e)
    {
      response->error = std::string()
          + "Failed to convert trajectory at index [" + std::to_string(i)
          + "] with exception: " + e.what();
      RCLCPP_WARN(get_logger(), response->error);
      return;
    }
  }

  std::size_t index=0;

  {
    std::unique_lock<std::mutex> lock(database_mutex);
    while (index < request->replace_ids.size() &&
           index < request->trajectories.size())
    {
      database.replace(request->replace_ids[index],
                       std::move(trajectories[index]));
      ++index;
    }

    for (; index < request->trajectories.size(); ++index)
      database.insert(std::move(trajectories[index]));

    response->latest_trajectory_version = database.latest_version();

    for (; index < request->replace_ids.size(); ++index)
      database.erase(request->replace_ids[index]);
  }

  response->current_version = database.latest_version();
  wakeup_mirrors();
}

//==============================================================================
void ScheduleNode::delay_trajectories(
    const request_id_ptr& /*request_header*/,
    const DelayTrajectories::Request::SharedPtr& request,
    const DelayTrajectories::Response::SharedPtr& response)
{
  response->original_version = database.latest_version();
  response->current_version = response->original_version;

  const auto from_time = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(request->from_time));

  if (request->delay < 0)
  {
    response->error = std::string()
        + "The schedule does not (yet) support negative delays ["
        + std::to_string(request->delay) + "]";
    RCLCPP_WARN(get_logger(), response->error);
    return;
  }

  const auto delay = std::chrono::nanoseconds(request->delay);

  {
    std::unique_lock<std::mutex> lock(database_mutex);
    for (const rmf_traffic::schedule::Version id : request->delay_ids)
      database.delay(id, from_time, delay);
  }

  wakeup_mirrors();
}

//==============================================================================
void ScheduleNode::erase_trajectories(
    const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
    const EraseTrajectories::Request::SharedPtr& request,
    const EraseTrajectories::Response::SharedPtr& response)
{
  {
    std::unique_lock<std::mutex> lock(database_mutex);
    for(const uint64_t id : request->erase_ids)
      database.erase(id);
  }

  response->version = database.latest_version();
  wakeup_mirrors();
}

//==============================================================================
void ScheduleNode::resolve_trajectories(
    const std::shared_ptr<rmw_request_id_t>& request_header,
    const ResolveTrajectories::Request::SharedPtr& request,
    const ResolveTrajectories::Response::SharedPtr& response)
{
  TODO(MXG): Implement this function;
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

  auto query = rmf_traffic::schedule::make_query(
        request->latest_mirror_version);
  query.spacetime() = query_it->second;

  response->patch = rmf_traffic_ros2::convert(database.changes(query));
}

//==============================================================================
void ScheduleNode::wakeup_mirrors() const
{
  rmf_traffic_msgs::msg::MirrorWakeup msg;
  msg.latest_version = database.latest_version();
  mirror_wakeup_publisher->publish(msg);
}

} // namespace rmf_traffic_schedule
