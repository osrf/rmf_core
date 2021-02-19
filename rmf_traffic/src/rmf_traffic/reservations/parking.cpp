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

#include <rmf_traffic/reservations/parking.hpp>
#include <unordered_map>
#include <mutex>
#include <map>

namespace rmf_traffic {
namespace reservations {

class ParkingReservation::Implementation {
public:
  Implementation(
    std::shared_ptr<ReservationSystem> reservation_system):
    _reservation_system(reservation_system)
  {
    // Do nothing
  }

  std::optional<Reservation> reserve(
    const rmf_traffic::schedule::ParticipantId participantId,
    const rmf_traffic::Time time,
    const std::vector<rmf_traffic::agv::Graph::Waypoint>& vertices,
    std::optional<rmf_traffic::Duration> duration = std::nullopt)
  {
    for(auto& vert: vertices)
    {
      if(!vert.is_parking_spot())
      {
        throw std::runtime_error(
          "A vertex which was not a parking spot was passed to ParkingReservation");
      }
    }

    return _reservation_system->reserve(
      participantId,
      time,
      vertices,
      duration
      );
  }

private:
  std::shared_ptr<ReservationSystem> _reservation_system;
}

//=========================================================================
ParkingReservation::ParkingReservation(
  std::shared_ptr<ReservationSystem> reservation_system):
  _pimpl(reservation_system)
{
  // Do nothing
}

//=========================================================================
std::optional<Reservation> ParkingReservation::reserve(
    const rmf_traffic::schedule::ParticipantId participantId,
    const rmf_traffic::Time time,
    const std::vector<rmf_traffic::agv::Graph::Waypoint>& vertices,
    std::optional<rmf_traffic::Duration> duration = std::nullopt)
{
  return _pimpl->reserve(particpantId, time, vertices, duration);
}


}
}