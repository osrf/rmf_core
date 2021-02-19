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

#ifndef RMF_TRAFFIC__PARKING_HPP
#define RMF_TRAFFIC__PARKING_HPP

#include <memory>
#include <rmf_traffic/reservations/reservations.hpp>

namespace rmf_traffic {
namespace reservations {

//=============================================================================
/// A simple wrapper around the class for parking items
/// 
class ParkingReservation
{
public:
  ParkingReservation(
    std::shared_ptr<ReservationSystem> reservation_system);

  
  std::optional<Reservation> reserve(
    const rmf_traffic::schedule::ParticipantId participantId,
    const rmf_traffic::Time time,
    const std::vector<rmf_traffic::agv::Graph::Waypoint>& vertices,
    std::optional<rmf_traffic::Duration> duration = std::nullopt);

  class Implementation;

private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} //end namespace reservations
} //end namespace rmf_traffic
#endif