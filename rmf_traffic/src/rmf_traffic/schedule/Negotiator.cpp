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
SimpleResponder::SimpleResponder(
  std::shared_ptr<schedule::Negotiation> negotiation,
  std::vector<schedule::ParticipantId> sequence)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(negotiation),
        sequence.back(),
        std::move(sequence)
      }))
{
  _pimpl->to_accommodate.pop_back();
}

//==============================================================================
void SimpleResponder::submit(std::vector<Route> itinerary,
  std::function<UpdateVersion()> /*approval_callback*/) const
{
  const auto table = _pimpl->negotiation->table(
    _pimpl->for_participant, _pimpl->to_accommodate);

  if (table)
  {
    table->submit(
      std::move(itinerary),
      table->version() ? *table->version()+1 : 0);
  }
}

//==============================================================================
void SimpleResponder::reject(
    const Negotiation::Alternatives& alternatives) const
{
  const auto parent = _pimpl->negotiation->table(_pimpl->to_accommodate);
  if (parent)
    parent->reject(*parent->version(), _pimpl->for_participant, alternatives);
}

//==============================================================================
void SimpleResponder::forfeit(const std::vector<ParticipantId>&) const
{
  const auto table = _pimpl->negotiation->table(
        _pimpl->for_participant, _pimpl->to_accommodate);

  if (table)
    table->forfeit(table->version()? *table->version() : 0);
}

} // namespace schedule
} // namespace rmf_traffic
