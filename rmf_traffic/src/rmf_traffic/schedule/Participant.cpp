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

#include "ParticipantInternal.hpp"
#include "debug_Participant.hpp"
#include "RectifierInternal.hpp"

#include <sstream>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
ItineraryVersion Participant::Debug::get_itinerary_version(const Participant& p)
{
  return p._pimpl->current_version();
}

//==============================================================================
Participant Participant::Implementation::make(
  ParticipantDescription description,
  std::shared_ptr<Writer> writer,
  std::shared_ptr<RectificationRequesterFactory> rectifier_factory)
{
  const auto registration = writer->register_participant(description);

  Participant participant;
  participant._pimpl = rmf_utils::make_unique_impl<Implementation>(
    registration, std::move(description), std::move(writer));

  if (rectifier_factory)
  {
    participant._pimpl->_rectification =
      rectifier_factory->make(
      Rectifier::Implementation::make(*participant._pimpl), registration.id());
  }

  return participant;
}

//==============================================================================
void Participant::Implementation::retransmit(
  const std::vector<Rectifier::Range>& ranges,
  const ItineraryVersion last_known_version)
{
  if (rmf_utils::modular(current_version()).less_than(last_known_version))
  {
    std::stringstream str;
    str << "[Participant::Implementation::retransmit] Remote database has a "
        << "higher version number [" << last_known_version << "] than ["
        << current_version() << "] the version number of the local "
        << "participant [" << _id << ":" << _description.owner() << "/"
        << _description.name() << "]. This indicates that the remote database "
        << "is receiving participant updates from a conflicting source.";

    throw std::runtime_error(str.str());
  }

  for (const auto& range : ranges)
  {
    assert(rmf_utils::modular(range.lower).less_than_or_equal(range.upper));

    // We use lower_bound because it is possible that some of this range has
    // been truncated because of a nullifying change. Therefore we should accept
    // the lowest change in the map if the lower end of this range is gone.
    const auto begin_it = _change_history.lower_bound(range.lower);

    const auto end_it = _change_history.find(range.upper);
    if (end_it == _change_history.end())
    {
      // These changes are no longer relevant. The inconsistency should get
      // fixed by either one of the later ranges or by fixing the tail at the
      // end of this function.
      continue;
    }

    // The _change_map will always contain complete sequences with no gaps, so
    // if end_it was found, then begin_it should be an entry less than or equal
    // to end_it.
    assert(begin_it->first <= end_it->first);

    for (auto it = begin_it; it->first <= end_it->first; ++it)
      it->second();
  }

  // In case the database doesn't have the most recent changes, we will
  // retransmit them.
  const auto tail_begin = _change_history.upper_bound(last_known_version);
  for (auto it = tail_begin; it != _change_history.end(); ++it)
    it->second();
}

//==============================================================================
ItineraryVersion Participant::Implementation::current_version() const
{
  return _version;
}

//==============================================================================
Participant::Implementation::Implementation(
  const Writer::Registration& registration,
  ParticipantDescription description,
  std::shared_ptr<Writer> writer)
: _id(registration.id()),
  _version(registration.itinerary_version()),
  _description(std::move(description)),
  _writer(writer)
{
  // Do nothing
}

//==============================================================================
Participant::Implementation::~Implementation()
{
  // Unregister the participant during destruction
  _writer->unregister_participant(_id);
}

//==============================================================================
Writer::Input Participant::Implementation::make_input(
  std::vector<Route> itinerary)
{
  Writer::Input input;
  input.reserve(itinerary.size());

  for (std::size_t i = 0; i < itinerary.size(); ++i)
  {
    if (itinerary[i].trajectory().size() < 2)
      continue;

    input.emplace_back(
      Writer::Item{
        ++_last_route_id,
        std::make_shared<Route>(std::move(itinerary[i]))
      });
  }

  return input;
}

//==============================================================================
ItineraryVersion Participant::Implementation::get_next_version()
{
  return ++_version;
}

//==============================================================================
RouteId Participant::set(std::vector<Route> itinerary)
{
  // TODO(MXG): Consider issuing an exception or warning when a single-point
  // trajectory is submitted.
  const auto r_it = std::remove_if(itinerary.begin(), itinerary.end(),
                 [](const auto& r){ return r.trajectory().size() < 2; });
  itinerary.erase(r_it, itinerary.end());

  const RouteId initial_route_id = _pimpl->_last_route_id;
  if (itinerary.empty())
  {
    // This situation is more efficient to express as a clear() command
    clear();
    return initial_route_id;
  }

  _pimpl->_change_history.clear();
  _pimpl->_cumulative_delay = std::chrono::seconds(0);

  auto input = _pimpl->make_input(std::move(itinerary));

  _pimpl->_current_itinerary = input;

  const ItineraryVersion itinerary_version = _pimpl->get_next_version();
  const ParticipantId id = _pimpl->_id;
  auto change =
    [this, input = std::move(input), itinerary_version, id]()
    {
      this->_pimpl->_writer->set(id, input, itinerary_version);
    };

  _pimpl->_change_history[itinerary_version] = change;
  change();

  return initial_route_id;
}

//==============================================================================
RouteId Participant::extend(const std::vector<Route>& additional_routes)
{
  const RouteId initial_route_id = _pimpl->_last_route_id;
  if (additional_routes.empty())
    return initial_route_id;

  auto input = _pimpl->make_input(std::move(additional_routes));

  _pimpl->_current_itinerary.reserve(
    _pimpl->_current_itinerary.size() + input.size());

  for (const auto& item : input)
    _pimpl->_current_itinerary.push_back(item);

  const ItineraryVersion itinerary_version = _pimpl->get_next_version();
  const ParticipantId id = _pimpl->_id;
  auto change =
    [this, input = std::move(input), itinerary_version, id]()
    {
      this->_pimpl->_writer->extend(id, input, itinerary_version);
    };

  _pimpl->_change_history[itinerary_version] = change;
  change();

  return initial_route_id;
}

//==============================================================================
void Participant::delay(Duration delay)
{
  bool no_delays = true;
  for (auto& item : _pimpl->_current_itinerary)
  {
    if (item.route->trajectory().size() > 0)
    {
      no_delays = false;

      auto new_trajectory = item.route->trajectory();
      new_trajectory.front().adjust_times(delay);

      item.route = std::make_shared<Route>(
            item.route->map(), std::move(new_trajectory));
    }
  }

  if (no_delays)
  {
    // We don't need to make any changes, because there are no waypoints to move
    return;
  }

  _pimpl->_cumulative_delay += delay;

  const ItineraryVersion itinerary_version = _pimpl->get_next_version();
  const ParticipantId id = _pimpl->_id;
  auto change =
    [this, delay, itinerary_version, id]()
    {
      this->_pimpl->_writer->delay(id, delay, itinerary_version);
    };

  _pimpl->_change_history[itinerary_version] = change;
  change();
}

//==============================================================================
rmf_traffic::Duration Participant::delay() const
{
  return _pimpl->_cumulative_delay;
}

//==============================================================================
void Participant::erase(const std::unordered_set<RouteId>& input_routes)
{
  const auto remove_it = std::remove_if(
    _pimpl->_current_itinerary.begin(),
    _pimpl->_current_itinerary.end(),
    [&](const Writer::Item& item)
    {
      return input_routes.count(item.id) > 0;
    });

  if (remove_it == _pimpl->_current_itinerary.end())
  {
    // None of the requested IDs are in the current itinerary, so there is
    // nothing to remove
    return;
  }

  std::vector<RouteId> routes;
  routes.reserve(input_routes.size());
  for (auto it = remove_it; it != _pimpl->_current_itinerary.end(); ++it)
    routes.push_back(it->id);

  _pimpl->_current_itinerary.erase(remove_it, _pimpl->_current_itinerary.end());

  const ItineraryVersion itinerary_version = _pimpl->get_next_version();
  const ParticipantId id = _pimpl->_id;
  auto change =
    [this, routes = std::move(routes), itinerary_version, id]()
    {
      this->_pimpl->_writer->erase(id, routes, itinerary_version);
    };

  _pimpl->_change_history[itinerary_version] = change;
  change();
}

//==============================================================================
void Participant::clear()
{
  if (_pimpl->_current_itinerary.empty())
  {
    // There is nothing to clear, so we can skip this change
    return;
  }

  _pimpl->_current_itinerary.clear();

  const ItineraryVersion itinerary_version = _pimpl->get_next_version();
  const ParticipantId id = _pimpl->_id;
  auto change =
    [this, itinerary_version, id]()
    {
      this->_pimpl->_writer->erase(id, itinerary_version);
    };

  _pimpl->_change_history[itinerary_version] = change;
  change();
}

//==============================================================================
RouteId Participant::last_route_id() const
{
  return _pimpl->_last_route_id;
}

//==============================================================================
const Writer::Input& Participant::itinerary() const
{
  return _pimpl->_current_itinerary;
}

//==============================================================================
ItineraryVersion Participant::version() const
{
  return _pimpl->_version;
}

//==============================================================================
const ParticipantDescription& Participant::description() const
{
  return _pimpl->_description;
}

//==============================================================================
ParticipantId Participant::id() const
{
  return _pimpl->_id;
}

//==============================================================================
Participant::Participant()
{
  // Do nothing
}

//==============================================================================
Participant make_participant(ParticipantDescription description,
  std::shared_ptr<Writer> writer,
  std::shared_ptr<RectificationRequesterFactory> rectifier_factory)
{
  if (!writer)
  {
    throw std::runtime_error(
          "[rmf_traffic::schedule::make_participant] A nullptr was given for "
          "the `writer` argument. This is illegal.");
  }

  return Participant::Implementation::make(
    std::move(description),
    std::move(writer),
    std::move(rectifier_factory));
}

} // namespace schedule
} // namespace rmf_traffic
