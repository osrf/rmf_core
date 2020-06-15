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

  std::shared_ptr<const schedule::Viewer> shared_viewer;
  const schedule::Viewer* viewer;
  schedule::ParticipantId participant;
  Profile profile;

};

//==============================================================================
ScheduleRouteValidator::ScheduleRouteValidator(
  const schedule::Viewer& viewer,
  schedule::ParticipantId participant_id,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        nullptr,
        &viewer,
        participant_id,
        std::move(profile)
      }))
{
  // Do nothing
}

//==============================================================================
ScheduleRouteValidator::ScheduleRouteValidator(
  std::shared_ptr<const schedule::Viewer> viewer,
  schedule::ParticipantId participant_id,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        viewer,
        viewer.get(),
        participant_id,
        std::move(profile)
      }))
{
  // Do nothing
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
  // TODO(MXG): Should we use a mutable Spacetime instance to avoid the
  // allocation here?
  schedule::Query::Spacetime spacetime;
  spacetime.query_timespan()
      .all_maps(false)
      .add_map(route.map())
      .set_lower_time_bound(*route.trajectory().start_time())
      .set_upper_time_bound(*route.trajectory().finish_time());

  const auto view = _pimpl->viewer->query(
        spacetime, schedule::Query::Participants::make_all());

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
    schedule::Negotiation::Table::ViewerPtr viewer;
    Profile profile;
  };

  std::shared_ptr<const Data> data;
  std::vector<schedule::ParticipantId> alternative_sets;

  Implementation(
      schedule::Negotiation::Table::ViewerPtr viewer,
      Profile profile)
  : data(std::make_shared<Data>(
           Data{
             std::move(viewer),
             std::move(profile)
           }))
  {
    const auto& alternatives = data->viewer->alternatives();
    alternative_sets.reserve(alternatives.size());
    for (const auto& r : alternatives)
      alternative_sets.push_back(r.first);
  }
};

//==============================================================================
NegotiatingRouteValidator::Generator::Generator(
  schedule::Negotiation::Table::ViewerPtr viewer,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
           std::move(viewer), std::move(profile)))
{
  // Do nothing
}

//==============================================================================
NegotiatingRouteValidator::Generator::Generator(
  schedule::Negotiation::Table::ViewerPtr viewer)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             viewer,
             viewer->get_description(viewer->participant_id())->profile()))
{
  // Do nothing
}

//==============================================================================
class NegotiatingRouteValidator::Implementation
{
public:

  std::shared_ptr<const Generator::Implementation::Data> data;
  schedule::Negotiation::VersionedKeySequence rollouts;
  rmf_utils::optional<schedule::ParticipantId> masked = rmf_utils::nullopt;

  static NegotiatingRouteValidator make(
    std::shared_ptr<const Generator::Implementation::Data> data,
    schedule::Negotiation::VersionedKeySequence rollouts)
  {
    NegotiatingRouteValidator output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{
            std::move(data),
            std::move(rollouts)
          });

    return output;
  }

  static rmf_utils::clone_ptr<NegotiatingRouteValidator> make_ptr(
      std::shared_ptr<const Generator::Implementation::Data> data,
      schedule::Negotiation::VersionedKeySequence rollout)
  {
    return rmf_utils::make_clone<NegotiatingRouteValidator>(
          make(std::move(data), std::move(rollout)));
  }
};

//==============================================================================
std::vector<rmf_utils::clone_ptr<NegotiatingRouteValidator>>
NegotiatingRouteValidator::Generator::all() const
{
  const std::size_t N_alts = _pimpl->data->viewer->alternatives().size();
  if (0 == N_alts)
    return {rmf_utils::make_clone<NegotiatingRouteValidator>(begin())};

  std::vector<std::vector<schedule::Version>> version_queue;
  std::vector<schedule::Version> current_versions;
  current_versions.reserve(N_alts);
  std::vector<schedule::Version> end_versions;
  current_versions.reserve(N_alts);
  schedule::Negotiation::VersionedKeySequence keys;
  keys.reserve(N_alts);
  for (const auto& alts : _pimpl->data->viewer->alternatives())
  {
    current_versions.push_back(0);
    end_versions.push_back(alts.second->size());
    keys.push_back({alts.first, 0});
  }

  assert(current_versions.size() == N_alts);
  assert(end_versions.size() == N_alts);
  assert(keys.size() == N_alts);

  while (true) // A break statement provides the exit condition
  {
    for (std::size_t i=0; i < N_alts-1; ++i)
    {
      if (current_versions[i] >= end_versions[i])
      {
        for (std::size_t j=0; j <= i; ++j)
          current_versions[j] = 0;

        ++current_versions[i+1];
        continue;
      }
    }

    if (current_versions.back() >= end_versions.back())
      break;

    version_queue.push_back(current_versions);
    ++current_versions[0];
  }

  std::vector<rmf_utils::clone_ptr<NegotiatingRouteValidator>> validators;
  validators.reserve(version_queue.size());

  for (const auto& versions : version_queue)
  {
    for (std::size_t i=0; i < N_alts; ++i)
      keys[i].version = versions[i];

    validators.emplace_back(
          NegotiatingRouteValidator::Implementation::make_ptr(
            _pimpl->data, keys));
  }

  return validators;
}

//==============================================================================
NegotiatingRouteValidator NegotiatingRouteValidator::Generator::begin() const
{
  schedule::Negotiation::VersionedKeySequence rollouts;
  for (const auto& r : _pimpl->data->viewer->alternatives())
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
  return _pimpl->data->viewer->alternatives().at(participant)->size();
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
        const schedule::Negotiation::VersionedKey& key)
  {
    return key.participant == id;
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

  it->version += 1;

  return _pimpl->make(_pimpl->data, std::move(rollouts));
}

//==============================================================================
const schedule::Negotiation::VersionedKeySequence&
NegotiatingRouteValidator::alternatives() const
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
        _pimpl->data->viewer->alternatives().at(r.participant)->size();

    if (num_alternatives <= r.version)
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
  schedule::Query::Spacetime spacetime;
  spacetime.query_timespan()
      .all_maps(false)
      .add_map(route.map())
      .set_lower_time_bound(*route.trajectory().start_time())
      .set_upper_time_bound(*route.trajectory().finish_time());

  const auto view = _pimpl->data->viewer->query(spacetime, _pimpl->rollouts);

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
        _pimpl->data->viewer->alternatives()
        .at(r.participant)
        ->at(r.version).back();

    if (route.map() != last_route->map())
      continue;

    const auto& last_wp = last_route->trajectory().back();

    if (*route.trajectory().finish_time() < last_wp.time())
      continue;

    const auto& description =
        _pimpl->data->viewer->get_description(r.participant);
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
