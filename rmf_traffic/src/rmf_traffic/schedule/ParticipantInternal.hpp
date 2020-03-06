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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__PARTICIPANTINTERNAL_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__PARTICIPANTINTERNAL_HPP

#include "Modular.hpp"

#include <rmf_traffic/schedule/Participant.hpp>

#include <map>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Participant::Implementation
{
public:

  static Participant make(
      ParticipantDescription description,
      Writer& writer,
      RectificationRequesterFactory* rectifier_factory);

  void retransmit(
      const std::vector<Rectifier::Range>& from,
      ItineraryVersion last_known);

  ItineraryVersion current_version() const;

  // Note: It would be better if this constructor were private, but we need to
  // make it public so it can be used by rmf_utils::make_unique_impl
  Implementation(
      ParticipantId id,
      ParticipantDescription description,
      Writer& writer);

  ~Implementation();

private:
  friend class Participant;

  Writer::Input make_input(std::vector<Route> itinerary);
  ItineraryVersion get_next_version();

  const ParticipantId _id;
  const ParticipantDescription _description;
  Writer& _writer;
  std::unique_ptr<RectificationRequester> _rectification;

  using ChangeHistory =
      std::map<RouteId, std::function<void()>, ModularLess<RouteId>>;

  Writer::Input itinerary;

  ChangeHistory _change_history;
  RouteId _last_route_id = std::numeric_limits<RouteId>::max();
  ItineraryVersion _version = std::numeric_limits<ItineraryVersion>::max();
};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__PARTICIPANTINTERNAL_HPP
