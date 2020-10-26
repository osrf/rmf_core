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

#ifndef SRC__RMF_TRAFFIC__BLOCKADE__INTERNAL_PARTICIPANT_HPP
#define SRC__RMF_TRAFFIC__BLOCKADE__INTERNAL_PARTICIPANT_HPP

#include <rmf_traffic/blockade/Participant.hpp>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
class Participant::Implementation
{
public:

  static Participant make(
      ParticipantId participant_id,
      double radius,
      std::shared_ptr<Writer> writer,
      std::shared_ptr<RectificationRequesterFactory> rectifier_factory);

  void check(const Status& status);

  void check();

  Implementation(
      ParticipantId id,
      double radius,
      std::shared_ptr<Writer> writer);

  ~Implementation();

private:
  friend class Participant;

  void _send_reservation();
  void _send_ready();
  void _send_reached();

  ParticipantId _id;
  double _radius;
  std::shared_ptr<Writer> _writer;
  std::unique_ptr<RectificationRequester> _rectification;

  Writer::Reservation _current_reservation;
  std::optional<ReservationId> _reservation_id;
  std::optional<CheckpointId> _last_ready;
  CheckpointId _last_reached;

};

} // namespace blockade
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__BLOCKADE__INTERNAL_PARTICIPANT_HPP
