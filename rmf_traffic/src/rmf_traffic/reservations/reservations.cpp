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

#include <rmf_traffic/reservations/reservations.hpp>
#include <unordered_map>
#include <mutex>
#include <map>

namespace rmf_traffic {
namespace reservations {
//==============================================================================
Reservation::Reservation(
    uint64_t unique_id,
    std::optional<rmf_traffic::Duration> duration,
    rmf_traffic::agv::Graph::Waypoint waypoint,
    rmf_traffic::schedule::ParticipantId participantId):
      _unique_id(unique_id),
      _duration(duration),
      _waypoint(waypoint),
      _participantId(participantId)
{

}


//==============================================================================
class ReservationSystem::Implementation
{
public:
  
  //============================================================================
  std::optional<Reservation> reserve(
    const rmf_traffic::schedule::ParticipantId participantId,
    const rmf_traffic::Time time,
    const std::vector<rmf_traffic::agv::Graph::Waypoint>& vertices,
    std::optional<rmf_traffic::Duration> duration)
  {
    //const std::lock_guard<std::mutex> lock(_mutex);
    for(auto waypoint: vertices)
    {
      if(is_free(waypoint, time, duration))
      {
        return {make_reservation(participantId, time, waypoint, duration)};
      }
    }

    return std::nullopt;
  }

  //==============================================================================
  Reservation make_reservation(
    const rmf_traffic::schedule::ParticipantId participantId,
    const rmf_traffic::Time time,
    const rmf_traffic::agv::Graph::Waypoint& waypoint,
    std::optional<rmf_traffic::Duration> duration)
  {
    Reservation reservation(
      _reservation_counter,
      duration,
      waypoint,
      participantId);

    _schedule[waypoint.index()].insert({time, reservation});
    _reservations[_reservation_counter] = time;
    _reservation_by_participant[participantId].insert(_reservation_counter); 
    _reservation_counter++;

    return reservation;
  }

  //==============================================================================
  void cancel_reservation(Reservation res)
  {
    const std::lock_guard<std::mutex> lock(_mutex);
    auto uid = res._unique_id;
    auto time = _reservations[uid];
    _schedule[res._waypoint.index()].erase(time);
    _reservations.erase(uid);
    _reservation_by_participant[res._participantId].erase(uid);
  }

  //==============================================================================
  bool is_free(
    rmf_traffic::agv::Graph::Waypoint waypoint,
    rmf_traffic::Time start_time,
    std::optional<rmf_traffic::Duration> duration)
  {

    auto waypoint_schedule = _schedule.find(waypoint.index());

    if(waypoint_schedule == _schedule.end())
    {
      _schedule.insert({waypoint.index(), {}});
      return true;
    }

    if(duration.has_value())
    {
      auto end_time = start_time+duration.value();
      auto end_slot = waypoint_schedule->second.lower_bound(end_time);
      auto start_slot = waypoint_schedule->second.upper_bound(start_time);

      if (end_slot == start_slot)
      {
        /// Since interval is in the same slot all we need to do
        /// is check if there is a previous reservation. If so,
        /// does the previous reservation exceed the duration
        if (start_slot == waypoint_schedule->second.begin())
        {
          /// No previous reservation safe to reserve
          return true;
        }
        else
        {
          /// Has previous reservation
          auto last_reservation = std::prev(start_slot);
          if (!last_reservation->second._duration.has_value())
          {
            /// Last reservation had infinite time
            return false;
          }
          else 
          {
            auto last_reservation_end_time = 
              last_reservation->first + 
              last_reservation->second._duration.value();

            return last_reservation_end_time <= start_time;
          }
        }
        
      }

      return false;
    }
    else
    {
      auto start_slot = waypoint_schedule->second.upper_bound(start_time);
      if(start_slot != waypoint_schedule->second.end())
      {
        /// A slot that is after the requested time exists hence we may not reserve it
        return false;
      }
      if(start_slot == waypoint_schedule->second.begin())
      {
        /// No booked slots, may reserve the entire time period
        return true;
      }

      // Check previous slot.
      auto last_reservation = std::prev(start_slot);
      if (!last_reservation->second._duration.has_value())
      {
        // Last reservation had infinite time
        return false;
      }
      else 
      {
        auto last_reservation_end_time = 
          last_reservation->first + 
          last_reservation->second._duration.value();

        return last_reservation_end_time <= start_time;
      }
    }
  }

  Implementation()
  {

  }

private:
  uint64_t _reservation_counter;

  // this is a hash map with the waypoint as a key and
  // a map. The map contains starting times of each reservation
  std::unordered_map<
    uint64_t,
    std::map<rmf_traffic::Time, Reservation>> _schedule;

  std::unordered_map<uint64_t, rmf_traffic::Time> _reservations;
  std::unordered_map<rmf_traffic::schedule::ParticipantId, 
    std::unordered_set<uint64_t>> _reservation_by_participant;

  std::mutex _mutex;
};

std::optional<Reservation> ReservationSystem::reserve(
  const rmf_traffic::schedule::ParticipantId participantId,
  const rmf_traffic::Time time,
  const std::vector<rmf_traffic::agv::Graph::Waypoint>& vertices,
  std::optional<rmf_traffic::Duration> duration)
{
  return _pimpl->reserve(participantId, time, vertices, duration);
}

void ReservationSystem::cancel_reservation(Reservation res)
{
  _pimpl->cancel_reservation(res);
}

ReservationSystem::ReservationSystem() :
 _pimpl(rmf_utils::make_unique_impl<Implementation>())
{

}

}
}