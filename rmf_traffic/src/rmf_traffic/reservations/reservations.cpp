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
class Reservation::Implementation
{
public:
  ReservationId _unique_id;
  std::optional<rmf_traffic::Duration> _duration;
  std::string _waypoint;

  Implementation(
    ReservationId unique_id,
    std::optional<rmf_traffic::Duration> duration,
    std::string waypoint):
      _unique_id(unique_id),
      _duration(duration),
      _waypoint(waypoint)
  {

  }
};

//==============================================================================
Reservation::Reservation(
    ReservationId unique_id,
    std::optional<rmf_traffic::Duration> duration,
    std::string waypoint):
    _pimpl(rmf_utils::make_impl<Reservation::Implementation>(
      unique_id,
      duration,
      waypoint
    ))
{

}

//==============================================================================
ReservationId Reservation::reservation_id() const
{
  return _pimpl->_unique_id;
}

//==============================================================================
const std::string Reservation::waypoint() const
{
  return _pimpl->_waypoint;  
}

//==============================================================================
const std::optional<rmf_traffic::Duration> Reservation::duration() const
{
  return _pimpl->_duration;  
}

//==============================================================================
class ReservationSystem::Implementation
{
public:
  
  //============================================================================
  std::optional<Reservation> reserve(
    const rmf_traffic::Time time,
    const std::vector<std::string>& vertices,
    std::optional<rmf_traffic::Duration> duration)
  {
    const std::lock_guard<std::mutex> lock(_mutex);
    for(auto waypoint: vertices)
    {
      if(is_free(waypoint, time, duration))
      {
        return {make_reservation(time, waypoint, duration)};
      }
    }

    return std::nullopt;
  }

  //==============================================================================
  Reservation make_reservation(
    const rmf_traffic::Time time,
    const std::string& waypoint,
    std::optional<rmf_traffic::Duration> duration)
  {
    Reservation reservation(
      _reservation_counter,
      duration,
      waypoint);

    _schedule[waypoint].insert({time, reservation});
    _reservations[_reservation_counter] = time;
    _location_by_reservation_id.insert({_reservation_counter, waypoint});
    _reservation_counter++;

    return reservation;
  }

  //==============================================================================
  void cancel_reservation(ReservationId reservation_id)
  {
    const std::lock_guard<std::mutex> lock(_mutex);
    
    // Lookup the reservation
    auto reservation = _reservations.find(reservation_id);
    if(reservation == _reservations.end())
    {
      throw std::runtime_error("Could not find reservation with id "
        + std::to_string(reservation_id));
    }
    auto time = reservation->second;

    auto location = _location_by_reservation_id.find(reservation_id)->second;
    _schedule[location].erase(time);
    _reservations.erase(reservation_id);
    _location_by_reservation_id.erase(reservation_id);

  }

  //==============================================================================
  bool is_free(
    std::string waypoint,
    rmf_traffic::Time start_time,
    std::optional<rmf_traffic::Duration> duration)
  {

    auto waypoint_schedule = _schedule.find(waypoint);

    if(waypoint_schedule == _schedule.end())
    {
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
          if (!last_reservation->second.duration().has_value())
          {
            /// Last reservation had infinite time
            return false;
          }
          else 
          {
            auto last_reservation_end_time = 
              last_reservation->first + 
              last_reservation->second.duration().value();

            return last_reservation_end_time <= start_time;
          }
        }
        
      }

      return false;
    }
    else
    {
      /// This branch is taken for indefinite reservations. 
      /// The key insight is that the indefinite reservation must 
      /// be after the last reservation. Hence we just neeed to check 
      /// if the last reservation ends after the requested start time.
      /// This results in O(1) lookup time
      if(waypoint_schedule->second.size() == 0)
      {
        return true;
      }
      else
      {
        auto last_reservation = waypoint_schedule->second.rbegin();
        if(!last_reservation->second.duration().has_value())
        {
          // Last reservation was indefinite reservation. Can't 
          // insert second indefinite reservation
          return false;
        }
        auto end_time = last_reservation->first 
          + *last_reservation->second.duration();
        return end_time <= start_time;
      }
    }
  }

  Implementation()
  {

  }

private:
  ReservationId _reservation_counter;

  // this is a hash map with the waypoint as a key and
  // a map. The map contains starting times of each reservation
  std::unordered_map<
    std::string,
    std::map<rmf_traffic::Time, Reservation>> _schedule;

  std::unordered_map<ReservationId, rmf_traffic::Time> _reservations;
  std::unordered_map<ReservationId,
    std::string> _location_by_reservation_id;

  std::mutex _mutex;
};

std::optional<Reservation> ReservationSystem::reserve(
  const rmf_traffic::Time time,
  const std::vector<std::string>& vertices,
  std::optional<rmf_traffic::Duration> duration)
{
  return _pimpl->reserve(time, vertices, duration);
}

void ReservationSystem::cancel_reservation(uint64_t res)
{
  _pimpl->cancel_reservation(res);
}

ReservationSystem::ReservationSystem() :
 _pimpl(rmf_utils::make_unique_impl<Implementation>())
{

}

} //end namespace reservation
} //end namespace rmf_traffic
