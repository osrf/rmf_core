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

#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/DetectConflict.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class ScheduleRouteValidator::Implementation
{
public:

  const schedule::Viewer* viewer;
  schedule::ParticipantId participant;
  Profile profile;
  mutable rmf_traffic::schedule::Query query;

};

//==============================================================================
ScheduleRouteValidator::ScheduleRouteValidator(
    const schedule::Viewer& viewer,
    schedule::ParticipantId participant_id,
    Profile profile)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               &viewer,
               participant_id,
               std::move(profile),
               rmf_traffic::schedule::query_all()
             }))
{
  _pimpl->query.spacetime().query_timespan({});
}

//==============================================================================
ScheduleRouteValidator& ScheduleRouteValidator::schedule_viewer(
    const schedule::Viewer& viewer)
{
  _pimpl->viewer = &viewer;
  return *this;
}

//==============================================================================
const schedule::Viewer& ScheduleRouteValidator::schedule_viewer() const
{
  return *_pimpl->viewer;
}

//==============================================================================
ScheduleRouteValidator& ScheduleRouteValidator::participant(
    const schedule::ParticipantId p)
{
  _pimpl->participant = p;
  return *this;
}

//==============================================================================
schedule::ParticipantId ScheduleRouteValidator::participant() const
{
  return _pimpl->participant;
}

//==============================================================================
bool ScheduleRouteValidator::valid(const Route& route) const
{
  _pimpl->query.spacetime().timespan()->clear_maps();
  _pimpl->query.spacetime().timespan()->add_map(route.map());

  _pimpl->query.spacetime().timespan()->set_lower_time_bound(
        *route.trajectory().start_time());

  _pimpl->query.spacetime().timespan()->set_upper_time_bound(
        *route.trajectory().finish_time());

  const auto view = _pimpl->viewer->query(_pimpl->query);
  for (const auto& v : view)
  {
    if (v.participant == _pimpl->participant)
      continue;

    if (rmf_traffic::DetectConflict::between(
          _pimpl->profile,
          route.trajectory(),
          v.description.profile(),
          v.route.trajectory()))
      return false;
  }

  return true;
}

//==============================================================================
std::unique_ptr<RouteValidator> ScheduleRouteValidator::clone() const
{
  return std::make_unique<ScheduleRouteValidator>(*this);
}

//==============================================================================
class NegotiatingRouteValidator::Implementation
{
public:

  const schedule::Negotiation::Table* table;
  Profile profile;

  mutable schedule::Query query;
};

//==============================================================================
NegotiatingRouteValidator::NegotiatingRouteValidator(
    const schedule::Negotiation::Table& table,
    Profile profile)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               &table,
               std::move(profile),
               schedule::query_all()
             }))
{
  _pimpl->query.spacetime().query_timespan({});
}

//==============================================================================
bool NegotiatingRouteValidator::valid(const Route& route) const
{
  _pimpl->query.spacetime().timespan()->clear_maps();
  _pimpl->query.spacetime().timespan()->add_map(route.map());

  _pimpl->query.spacetime().timespan()->set_lower_time_bound(
        *route.trajectory().start_time());

  _pimpl->query.spacetime().timespan()->set_upper_time_bound(
        *route.trajectory().finish_time());

  const auto view = _pimpl->table->query(_pimpl->query.spacetime());
  for (const auto& v : view)
  {
    if (rmf_traffic::DetectConflict::between(
          _pimpl->profile,
          route.trajectory(),
          v.description.profile(),
          v.route.trajectory()))
    {
      return false;
    }
  }

  return true;
}

//==============================================================================
std::unique_ptr<RouteValidator> NegotiatingRouteValidator::clone() const
{
  return std::make_unique<NegotiatingRouteValidator>(*this);
}

} // namespace agv
} // namespace rmf_traffic
