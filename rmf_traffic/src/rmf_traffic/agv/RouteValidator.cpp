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
rmf_utils::optional<RouteValidator::Conflict>
ScheduleRouteValidator::find_conflict(const Route& route) const
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

    if (const auto time = rmf_traffic::DetectConflict::between(
        _pimpl->profile,
        route.trajectory(),
        v.description.profile(),
        v.route.trajectory()))
    {
      return Conflict{v.participant, *time};
    }
  }

  return rmf_utils::nullopt;
}

//==============================================================================
std::unique_ptr<RouteValidator> ScheduleRouteValidator::clone() const
{
  return std::make_unique<ScheduleRouteValidator>(*this);
}

//==============================================================================
class NegotiatingRouteValidator::Generator::Implementation
{
public:
  struct Data
  {
    // TODO(MXG): This should be changed to a Table::View
    const schedule::Negotiation::Table* table;
    Profile profile;
  };

  std::shared_ptr<Data> data;
  std::vector<schedule::ParticipantId> alternative_sets;

  Implementation(
      const schedule::Negotiation::Table& table,
      Profile profile)
  : data(std::make_shared<Data>(
           Data{
             &table,
             std::move(profile)
           }))
  {
    const auto& rollouts = table.rollouts();
    alternative_sets.reserve(rollouts.size());
    for (const auto& r : rollouts)
      alternative_sets.push_back(r.first);
  }
};

//==============================================================================
NegotiatingRouteValidator::Generator::Generator(
  const schedule::Negotiation::Table& table,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(table, std::move(profile)))
{
  // Do nothing
}

//==============================================================================
class NegotiatingRouteValidator::Implementation
{
public:

  std::shared_ptr<Generator::Implementation::Data> data;
  std::vector<schedule::Negotiation::Table::Rollout> rollouts;
  rmf_utils::optional<schedule::ParticipantId> masked = rmf_utils::nullopt;

  static NegotiatingRouteValidator make(
    std::shared_ptr<Generator::Implementation::Data> data,
    std::vector<schedule::Negotiation::Table::Rollout> rollouts)
  {
    NegotiatingRouteValidator output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{
            std::move(data),
            std::move(rollouts)
          });

    return output;
  }
};

//==============================================================================
NegotiatingRouteValidator NegotiatingRouteValidator::Generator::begin() const
{
  std::vector<schedule::Negotiation::Table::Rollout> rollouts;
  for (const auto& r : _pimpl->data->table->rollouts())
    rollouts.push_back({r.first, 0});

  return NegotiatingRouteValidator::Implementation::make(
        _pimpl->data, std::move(rollouts));
}

//==============================================================================
const std::vector<schedule::ParticipantId>&
NegotiatingRouteValidator::Generator::alternative_sets() const
{
  return _pimpl->alternative_sets;
}

//==============================================================================
std::size_t NegotiatingRouteValidator::Generator::alternative_count(
    schedule::ParticipantId participant) const
{
  return _pimpl->data->table->rollouts().at(participant).size();
}

//==============================================================================
NegotiatingRouteValidator& NegotiatingRouteValidator::mask(
    schedule::ParticipantId id)
{
  _pimpl->masked = id;
  return *this;
}

//==============================================================================
NegotiatingRouteValidator& NegotiatingRouteValidator::remove_mask()
{
  _pimpl->masked = rmf_utils::nullopt;
  return *this;
}

//==============================================================================
NegotiatingRouteValidator NegotiatingRouteValidator::next(
    schedule::ParticipantId id) const
{
  auto rollouts = _pimpl->rollouts;
  const auto it = std::find_if(
        rollouts.begin(), rollouts.end(), [&](
        const schedule::Negotiation::Table::Rollout& r)
  {
    return r.participant == id;
  });

  if (it == rollouts.end())
  {
    std::string error = "[NegotiatingRouteValidator::next] Requested next "
        "alternative for " + std::to_string(id) + " but the only options are [";

    for (const auto r : rollouts)
      error += " " + std::to_string(r.participant);

    error += " ]";

    throw std::runtime_error(error);
  }

  it->alternative += 1;

  return _pimpl->make(_pimpl->data, std::move(rollouts));
}

//==============================================================================
const std::vector<schedule::Negotiation::Table::Rollout>&
NegotiatingRouteValidator::rollouts() const
{
  return _pimpl->rollouts;
}

//==============================================================================
NegotiatingRouteValidator::operator bool() const
{
  return !end();
}

//==============================================================================
bool NegotiatingRouteValidator::end() const
{
  for (const auto& r : _pimpl->rollouts)
  {
    const auto num_alternatives =
        _pimpl->data->table->rollouts().at(r.participant).size();

    if (num_alternatives <= r.alternative)
      return true;
  }

  return false;
}

//==============================================================================
rmf_utils::optional<RouteValidator::Conflict>
NegotiatingRouteValidator::find_conflict(const Route& route) const
{
  // TODO(MXG): Consider if we can reduce the amount of heap allocation that's
  // needed here.
  auto query = schedule::make_query(
    {route.map()},
    route.trajectory().start_time(),
    route.trajectory().finish_time());

  const auto view = _pimpl->data->table->query(
        query.spacetime(), _pimpl->rollouts);

  for (const auto& v : view)
  {
    if (_pimpl->masked && (*_pimpl->masked == v.participant))
      continue;

    // NOTE(MXG): There is no need to check the map, because the query will
    // filter out all itineraries that are not on this map.
    if (const auto time = rmf_traffic::DetectConflict::between(
        _pimpl->data->profile,
        route.trajectory(),
        v.description.profile(),
        v.route.trajectory()))
    {
      return Conflict{v.participant, *time};
    }
  }

  for (const auto& r : _pimpl->rollouts)
  {
    if (_pimpl->masked && (*_pimpl->masked == r.participant))
      continue;

    const auto& last_route =
        _pimpl->data->table->rollouts()
        .at(r.participant)
        .at(r.alternative).back();

    if (route.map() != last_route->map())
      continue;

    const auto& last_wp = last_route->trajectory().back();

    if (*route.trajectory().finish_time() < last_wp.time())
      continue;

    const auto& description =
        _pimpl->data->table->viewer()->get_participant(r.participant);
    assert(description);

    // The end_cap trajectory represents the last known position of the
    // rollout's alternative. This prevents the negotiator from using a
    // pathological strategy like waiting until the other participant vanishes.
    Trajectory end_cap;
    end_cap.insert(
          last_wp.time(),
          last_wp.position(),
          Eigen::Vector3d::Zero());

    end_cap.insert(
          *route.trajectory().finish_time() + std::chrono::seconds(10),
          last_wp.position(),
          Eigen::Vector3d::Zero());

    if (const auto time = rmf_traffic::DetectConflict::between(
          _pimpl->data->profile,
          route.trajectory(),
          description->profile(),
          end_cap))
    {
      return Conflict{r.participant, *time};
    }
  }

  return rmf_utils::nullopt;
}

//==============================================================================
std::unique_ptr<RouteValidator> NegotiatingRouteValidator::clone() const
{
  return std::make_unique<NegotiatingRouteValidator>(*this);
}

//==============================================================================
NegotiatingRouteValidator::NegotiatingRouteValidator()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_traffic
