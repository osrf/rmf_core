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

#include <rmf_traffic/schedule/Negotiator.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class SimpleResponder::Implementation
{
public:

  std::shared_ptr<schedule::Negotiation> negotiation;
  schedule::ParticipantId for_participant;
  std::vector<schedule::ParticipantId> to_accommodate;

};

//==============================================================================
SimpleResponder::SimpleResponder(
    std::shared_ptr<schedule::Negotiation> negotiation,
    schedule::ParticipantId for_participant,
    std::vector<schedule::ParticipantId> to_accommodate)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(negotiation),
               for_participant,
               std::move(to_accommodate)
             }))
{
  // Do nothing
}

//==============================================================================
void SimpleResponder::submit(
    std::vector<Route> itinerary,
    std::function<void()> /*approval_callback*/) const
{
  _pimpl->negotiation->submit(
        _pimpl->for_participant,
        _pimpl->to_accommodate,
        std::move(itinerary));
}

//==============================================================================
void SimpleResponder::reject() const
{
  _pimpl->negotiation->reject(_pimpl->to_accommodate);
}

} // namespace schedule
} // namespace rmf_traffic
