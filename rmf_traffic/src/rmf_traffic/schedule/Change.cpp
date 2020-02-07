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

#include "ChangeInternal.hpp"

#include <rmf_utils/optional.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Change::Implementation
{
public:

  // TODO(MXG): This implementation could be changed to use a std::variant
  Mode mode;
  Set set;
  Extend extend;
  Delay delay;
  Erase erase;

  ParticipantId participant;
  Version schedule_version;

  Implementation(
      Mode mode_,
      ParticipantId participant_,
      Version schedule_version_)
  : mode(mode_),
    participant(participant_),
    schedule_version(schedule_version_)
  {
    // Do nothing
  }

  static Change make(
      Mode mode_,
      ParticipantId participant_,
      Version schedule_version_)
  {
    Change change;
    change._pimpl =
        rmf_utils::make_impl<Implementation>(
          mode_, participant_, schedule_version_);

    return change;
  }

  template<typename T, typename... Args>
  static rmf_utils::impl_ptr<typename T::Implementation> make_impl(
      Args&&... args)
  {
    return rmf_utils::make_impl<typename T::Implementation>(
          typename T::Implementation{std::forward<Args>(args)...});
  }

};

//==============================================================================
Change::Set::Set()
{
  // Do nothing
}

//==============================================================================
class Change::Set::Implementation
{
public:

  // TODO(MXG): This could be a shared_ptr to an itinerary to improve
  // performance for the schedule database
  Itinerary itinerary;

};

//==============================================================================
Change::Extend::Extend()
{
  // Do nothing
}

//==============================================================================
class Change::Extend::Implementation
{
public:

  // TODO(MXG): This could be an aliased shared_ptr to an alias's route to
  // improve performance for the schedule database.
  ConstRoutePtr route;

};

//==============================================================================
Change::Delay::Delay()
{
  // Do nothing
}

//==============================================================================
Change::Erase::Erase()
{
  // Do nothing
}

//==============================================================================
class Change::Erase::Implementation
{
public:

  /// True if the entire itinerary should be erased
  bool entire_itinerary;

  /// The indices of the routes that should be erased if entire_itinerary is
  /// false
  std::vector<RouteId> routes;

};

//==============================================================================
Change Change::make_set(
    ParticipantId participant,
    Itinerary itinerary,
    Version schedule_version)
{
  Change change =
      Implementation::make(Mode::Set, participant, schedule_version);

  change._pimpl->set._pimpl =
      Implementation::make_impl<Set>(std::move(itinerary));

  return change;
}

//==============================================================================
Change Change::make_extend(
    ParticipantId participant,
    Route route)
{
  Change change =
      Implementation::make(Mode::Extend, participant);

  change._pimpl->extend._pimpl =
      Implementation::make_impl<Extend>(
        std::make_shared<const Route>(std::move(route)));

  return change;
}

//==============================================================================
Change Change::make_delay(
    ParticipantId participant,
    Time from,
    Duration duration,
    Version schedule_version)
{
  Change change =
      Implementation::make(Mode::Delay, participant, schedule_version);

  change._pimpl->delay._pimpl =
      Implementation::make_impl<Delay>(from, duration);

  return change;
}

//==============================================================================
Change Change::make_erase(ParticipantId participant, Version schedule_version)
{
  Change change =
      Implementation::make(Mode::Erase, participant, schedule_version);

  change._pimpl->erase._pimpl =
      Implementation::make_impl<Erase>(true, std::vector<RouteId>());

  return change;
}

//==============================================================================
Change Change::make_erase(
    ParticipantId participant,
    std::vector<RouteId> routes,
    Version schedule_version)
{
  Change change =
      Implementation::make(Mode::Erase, participant, schedule_version);

  change._pimpl->erase._pimpl =
      Implementation::make_impl<Erase>(false, std::move(routes));

  return change;
}

//==============================================================================
Change::Mode Change::get_mode() const
{
  return _pimpl->mode;
}

//==============================================================================
ParticipantId Change::participant() const
{
  return _pimpl->participant;
}

//==============================================================================
const Change::Set* Change::set() const
{
  if (Mode::Set == _pimpl->mode)
    return nullptr;

  return &_pimpl->set;
}

//==============================================================================
const Change::Extend* Change::extend() const
{
  if (Mode::Extend == _pimpl->mode)
    return nullptr;

  return &_pimpl->extend;
}

//==============================================================================
const Change::Delay* Change::delay() const
{
  if (Mode::Delay == _pimpl->mode)
    return nullptr;

  return &_pimpl->delay;
}

//==============================================================================
const Change::Erase* Change::erase() const
{
  if (Mode::Erase == _pimpl->mode)
    return nullptr;

  return &_pimpl->erase;
}

//==============================================================================
Change::Change()
{
  // Do nothing
}

} // namespace schedule
} // namespace rmf_traffic
