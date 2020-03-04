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

#ifndef SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
#define SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP

#include <rmf_traffic/schedule/Database.hpp>

#include <rclcpp/node.hpp>

#include <rmf_traffic_msgs/msg/mirror_wakeup.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict.hpp>

#include <rmf_traffic_msgs/msg/itinerary_clear.hpp>
#include <rmf_traffic_msgs/msg/itinerary_delay.hpp>
#include <rmf_traffic_msgs/msg/itinerary_erase.hpp>
#include <rmf_traffic_msgs/msg/itinerary_extend.hpp>
#include <rmf_traffic_msgs/msg/itinerary_set.hpp>

#include <rmf_traffic_msgs/msg/schedule_inconsistency.hpp>

#include <rmf_traffic_msgs/srv/mirror_update.hpp>
#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/mirror_update.h>
#include <rmf_traffic_msgs/srv/unregister_query.hpp>



#include <rmf_traffic_msgs/srv/temporary_resolve_conflicts.hpp>



#include <unordered_map>

namespace rmf_traffic_schedule {

//==============================================================================
class ScheduleNode : public rclcpp::Node
{
public:

  ScheduleNode();

  ~ScheduleNode();

private:

  using request_id_ptr = std::shared_ptr<rmw_request_id_t>;

  using RegisterQuery = rmf_traffic_msgs::srv::RegisterQuery;
  using RegisterQueryService = rclcpp::Service<RegisterQuery>;

  void register_query(
      const request_id_ptr& request_header,
      const RegisterQuery::Request::SharedPtr& request,
      const RegisterQuery::Response::SharedPtr& response);

  RegisterQueryService::SharedPtr register_query_service;


  using UnregisterQuery = rmf_traffic_msgs::srv::UnregisterQuery;
  using UnregisterQueryService = rclcpp::Service<UnregisterQuery>;

  void unregister_query(
      const request_id_ptr& request_header,
      const UnregisterQuery::Request::SharedPtr& request,
      const UnregisterQuery::Response::SharedPtr& response);

  UnregisterQueryService::SharedPtr unregister_query_service;


  using MirrorUpdate = rmf_traffic_msgs::srv::MirrorUpdate;
  using MirrorUpdateService = rclcpp::Service<MirrorUpdate>;

  void mirror_update(
      const request_id_ptr& request_header,
      const MirrorUpdate::Request::SharedPtr& request,
      const MirrorUpdate::Response::SharedPtr& response);

  MirrorUpdateService::SharedPtr mirror_update_service;


  using MirrorWakeup = rmf_traffic_msgs::msg::MirrorWakeup;
  using MirrorWakeupPublisher = rclcpp::Publisher<MirrorWakeup>;
  MirrorWakeupPublisher::SharedPtr mirror_wakeup_publisher;


  using ScheduleConflict = rmf_traffic_msgs::msg::ScheduleConflict;
  using ScheduleConflictPublisher = rclcpp::Publisher<ScheduleConflict>;
  ScheduleConflictPublisher::SharedPtr conflict_publisher;


  using ItinerarySet = rmf_traffic_msgs::msg::ItinerarySet;
  void itinerary_set(const ItinerarySet& set);
  rclcpp::Subscription<ItinerarySet>::SharedPtr itinerary_set_sub;

  using ItineraryExtend = rmf_traffic_msgs::msg::ItineraryExtend;
  void itinerary_extend(const ItineraryExtend& extend);
  rclcpp::Subscription<ItineraryExtend>::SharedPtr itinerary_extend_sub;

  using ItineraryDelay = rmf_traffic_msgs::msg::ItineraryDelay;
  void itinerary_delay(const ItineraryDelay& delay);
  rclcpp::Subscription<ItineraryDelay>::SharedPtr itinerary_delay_sub;

  using ItineraryErase = rmf_traffic_msgs::msg::ItineraryErase;
  void itinerary_erase(const ItineraryErase& erase);
  rclcpp::Subscription<ItineraryErase>::SharedPtr itinerary_erase_sub;

  using ItineraryClear = rmf_traffic_msgs::msg::ItineraryClear;
  void itinerary_clear(const ItineraryClear& clear);
  rclcpp::Subscription<ItineraryClear>::SharedPtr itinerary_clear_sub;

  using InconsistencyMsg = rmf_traffic_msgs::msg::ScheduleInconsistency;
  rclcpp::Publisher<InconsistencyMsg>::SharedPtr inconsistency_pub;
  void publish_inconsistencies(rmf_traffic::schedule::ParticipantId id);

  void wakeup_mirrors();

  using TemporaryResolveConflicts = rmf_traffic_msgs::srv::TemporaryResolveConflicts;
  void temp_resolve_conflicts(
      const request_id_ptr& request_header,
      const TemporaryResolveConflicts::Request::SharedPtr& request,
      const TemporaryResolveConflicts::Response::SharedPtr& response);
  rclcpp::Service<TemporaryResolveConflicts>::SharedPtr temp_resolve_conflicts_srv;

  // TODO(MXG): Consider using libguarded instead of a database_mutex
  std::mutex database_mutex;
  rmf_traffic::schedule::Database database;

  using QueryMap =
      std::unordered_map<uint64_t, rmf_traffic::schedule::Query>;
  // TODO(MXG): Have a way to make query registrations expire after they have
  // not been used for some set amount of time (e.g. 24 hours? 48 hours?).
  std::size_t last_query_id = 0;
  QueryMap registered_queries;

  // TODO(MXG): Make this a separate node
  std::thread conflict_check_thread;
  std::condition_variable conflict_check_cv;
  std::atomic_bool conflict_check_quit;

  using Version = rmf_traffic::schedule::Version;
  struct ConflictInfo
  {
    ConflictInfo(std::unordered_set<Version> ids)
    : participants(std::move(ids))
    {
      // Do nothing
    }

    std::unordered_set<rmf_traffic::schedule::ParticipantId> participants;
  };

  using ConflictMap = std::map<Version, ConflictInfo>;
  ConflictMap active_conflicts;
  std::mutex active_conflicts_mutex;
  Version next_conflict_version = 0;
};

} // namespace rmf_traffic_schedule

#endif // SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
