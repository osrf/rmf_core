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

#include <rmf_traffic/schedule/Change.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Change::Implementation
{
public:

  // TODO(MXG): This implementation could be changed to use a std::variant
  Mode mode;
  Put put;
  Post post;
  Delay delay;
  Erase erase;

  ParticipantId participant;

  Implementation(Mode mode_, ParticipantId participant_)
    : mode(mode_),
      participant(participant_)
  {
    // Do nothing
  }

  static Change make(Mode mode_, ParticipantId participant_)
  {
    Change change;
    change._pimpl = rmf_utils::make_impl<Implementation>(mode_, participant_);
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
Change::Put::Put()
{
  // Do nothing
}

//==============================================================================
class Change::Put::Implementation
{
public:

  // TODO(MXG): This could be a shared_ptr to an itinerary to improve
  // performance for the schedule database
  Itinerary itinerary;

};

//==============================================================================
Change::Post::Post()
{
  // Do nothing
}

//==============================================================================
class Change::Post::Implementation
{
public:

  // TODO(MXG): This could be an aliased shared_ptr to an alias's route to
  // improve performance for the schedule database.
  Route route;

};

//==============================================================================
Change::Delay::Delay()
{
  // Do nothing
}

//==============================================================================
class Change::Delay::Implementation
{
public:

  /// The time that the delay began
  Time from;

  /// The duration of the delay
  Duration duration;

};

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
Change Change::make_put(ParticipantId participant, Itinerary itinerary)
{
  Change change = Implementation::make(Mode::Put, participant);
  change._pimpl->put._pimpl =
      Implementation::make_impl<Put>(std::move(itinerary));

  return change;
}

//==============================================================================
Change Change::make_post(ParticipantId participant, Route route)
{
  Change change = Implementation::make(Mode::Post, participant);
  change._pimpl->post._pimpl =
      Implementation::make_impl<Post>(std::move(route));

  return change;
}

//==============================================================================
Change Change::make_delay(
    ParticipantId participant,
    Time from,
    Duration duration)
{
  Change change = Implementation::make(Mode::Delay, participant);
  change._pimpl->delay._pimpl =
      Implementation::make_impl<Delay>(from, duration);

  return change;
}

//==============================================================================
Change Change::make_erase(ParticipantId participant)
{
  Change change = Implementation::make(Mode::Erase, participant);
  change._pimpl->erase._pimpl =
      Implementation::make_impl<Erase>(true, std::vector<RouteId>());

  return change;
}

//==============================================================================
Change Change::make_erase(
    ParticipantId participant,
    std::vector<RouteId> routes)
{
  Change change = Implementation::make(Mode::Erase, participant);
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
const Change::Put* Change::put() const
{
  if (Mode::Put == _pimpl->mode)
    return nullptr;

  return &_pimpl->put;
}

//==============================================================================
const Change::Post* Change::post() const
{
  if (Mode::Post == _pimpl->mode)
    return nullptr;

  return &_pimpl->post;
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
