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

  schedule::Negotiation::TablePtr table;
  schedule::Version table_version;
  schedule::Negotiation::TablePtr parent;
  rmf_utils::optional<schedule::Version> parent_version;

  std::vector<schedule::ParticipantId>* report_blockers;

  Implementation(
      schedule::Negotiation::TablePtr table_,
      std::vector<schedule::ParticipantId>* report_blockers_)
    : table(std::move(table_)),
      report_blockers(report_blockers_)
  {
    table_version = table->version();

    parent = table->parent();
    if (parent)
    {
      assert(parent->version());
      parent_version = parent->version();
    }
  }
};

//==============================================================================
SimpleResponder::SimpleResponder(
  const Negotiation::TablePtr& table,
  std::vector<schedule::ParticipantId>* report_blockers)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation(table, report_blockers)))
{
  // Do nothing
}

//==============================================================================
void SimpleResponder::submit(std::vector<Route> itinerary,
  std::function<UpdateVersion()> /*approval_callback*/) const
{
  _pimpl->table->submit(std::move(itinerary), _pimpl->table_version+1);
}

//==============================================================================
void SimpleResponder::reject(
    const Negotiation::Alternatives& alternatives) const
{
  if (_pimpl->parent)
  {
    _pimpl->parent->reject(
          *_pimpl->parent_version,
          _pimpl->table->participant(),
          alternatives);
  }

  // TODO(MXG): Should we throw an exception if the if-statement fails?
}

//==============================================================================
void SimpleResponder::forfeit(const std::vector<ParticipantId>& blockers) const
{
  if (_pimpl->report_blockers)
    *_pimpl->report_blockers = blockers;

  _pimpl->table->forfeit(_pimpl->table_version);
}

} // namespace schedule
} // namespace rmf_traffic
