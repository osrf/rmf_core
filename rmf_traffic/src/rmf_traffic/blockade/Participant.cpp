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

#include "internal_Participant.hpp"
#include "internal_Rectifier.hpp"

#include <sstream>


#include <iostream>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
Participant Participant::Implementation::make(
    const ParticipantId participant_id,
    const double radius,
    std::shared_ptr<Writer> writer,
    std::shared_ptr<RectificationRequesterFactory> rectifier_factory)
{
  Participant participant;
  participant._pimpl = rmf_utils::make_unique_impl<Implementation>(
        participant_id, radius, std::move(writer));

  if (rectifier_factory)
  {
    participant._pimpl->_rectification =
        rectifier_factory->make(
        Rectifier::Implementation::make(*participant._pimpl), participant_id);
  }

  return participant;
}

//==============================================================================
void Participant::Implementation::check(const Status& status)
{
  if (status.reservation != _reservation_id)
  {
    _send_reservation();

    if (_last_ready)
      _send_ready();

    // We intentionally leave out _last_reached for now, because a blockade
    // participant is expected to sit at the first waypoint until its
    // reservation is recognized by the moderator and departure is granted.
    // Therefore, the participant should not have reached anything past 0 if the
    // moderator does not yet know about the new path.

    return;
  }

  if (status.last_ready != _last_ready)
  {
    if (status.last_ready.has_value() && !_last_ready.has_value())
    {
      // The moderator thinks we are ready for some checkpoints, but we are not
      // ready for any. Releasing checkpoint 0 will tell the moderator that we
      // are not ready for any of these.
      _send_release(0);
    }
    else
    {
      assert(_last_ready.has_value());
      if (status.last_ready.has_value() && *status.last_ready > *_last_ready)
      {
        _send_release(_last_ready.value() + 1);
      }
      else
      {
        // We will reach this condition if:
        // 1. The moderator incorrectly thinks we are not ready for any
        //    departures, or
        // 2. The moderator does not know about our latest ready departure
        _send_ready();
      }
    }
  }

  if (status.last_reached != _last_reached)
    _send_reached();
}

//==============================================================================
void Participant::Implementation::check()
{
  if (!_reservation_id)
    return;

  _send_reservation();
}

//==============================================================================
Participant::Implementation::Implementation(
    const ParticipantId id,
    const double radius,
    std::shared_ptr<Writer> writer)
  : _id(id),
    _writer(std::move(writer)),
    _reservation_id(std::nullopt)
{
  _current_reservation.radius = radius;
}

//==============================================================================
Participant::Implementation::~Implementation()
{
  if (_reservation_id)
    _writer->cancel(_id);
}

//==============================================================================
void Participant::Implementation::_send_reservation()
{
  assert(_current_reservation.path.size() > 1);
  _writer->set(_id, _reservation_id.value(), _current_reservation);
}

//==============================================================================
void Participant::Implementation::_send_ready()
{
  _writer->ready(_id, _reservation_id.value(), _last_ready.value());
}

//==============================================================================
void Participant::Implementation::_send_release(CheckpointId checkpoint)
{
  _writer->release(_id, _reservation_id.value(), checkpoint);
}

//==============================================================================
void Participant::Implementation::_send_reached()
{
  _writer->reached(_id, _reservation_id.value(), _last_reached);
}

//==============================================================================
void Participant::radius(const double new_radius)
{
  _pimpl->_current_reservation.radius = new_radius;
}

//==============================================================================
double Participant::radius() const
{
  return _pimpl->_current_reservation.radius;
}

//==============================================================================
void Participant::set(std::vector<Writer::Checkpoint> path)
{
  _pimpl->_current_reservation.path = std::move(path);

  if (_pimpl->_reservation_id)
    ++*_pimpl->_reservation_id;
  else
    _pimpl->_reservation_id = 1;

  _pimpl->_last_ready = std::nullopt;
  _pimpl->_last_reached = 0;

  _pimpl->_send_reservation();
}

//==============================================================================
const std::vector<Writer::Checkpoint>& Participant::path() const
{
  return _pimpl->_current_reservation.path;
}

//==============================================================================
void Participant::ready(CheckpointId checkpoint)
{
  if (_pimpl->_current_reservation.path.size()-1 <= checkpoint)
  {
    // TODO(MXG): Should we consider throwing an exception here?
    checkpoint = _pimpl->_current_reservation.path.size() - 2;
  }

  if (_pimpl->_last_ready.has_value() && checkpoint <= *_pimpl->_last_ready)
    return;

  _pimpl->_last_ready = checkpoint;
  _pimpl->_send_ready();
}

//==============================================================================
void Participant::release(CheckpointId checkpoint)
{
  if (!_pimpl->_last_ready.has_value())
    return;

  if (_pimpl->_last_ready.value() < checkpoint)
    return;

  if (checkpoint > 0)
    _pimpl->_last_ready = checkpoint - 1;
  else
    _pimpl->_last_ready.reset();

  _pimpl->_send_release(checkpoint);
}

//==============================================================================
std::optional<CheckpointId> Participant::last_ready() const
{
  return _pimpl->_last_ready;
}

//==============================================================================
void Participant::reached(CheckpointId checkpoint)
{
  if (checkpoint <= _pimpl->_last_reached)
    return;

  _pimpl->_last_reached = checkpoint;
  _pimpl->_send_reached();
}

//==============================================================================
void Participant::cancel()
{
  if (_pimpl->_reservation_id)
  {
    _pimpl->_writer->cancel(_pimpl->_id, *_pimpl->_reservation_id);
    _pimpl->_reservation_id = std::nullopt;
  }
}

//==============================================================================
CheckpointId Participant::last_reached() const
{
  return _pimpl->_last_reached;
}

//==============================================================================
ParticipantId Participant::id() const
{
  return _pimpl->_id;
}

//==============================================================================
std::optional<ReservationId> Participant::reservation_id() const
{
  return _pimpl->_reservation_id;
}

//==============================================================================
Participant::Participant()
{
  // Do nothing
}

//==============================================================================
Participant make_participant(
    const ParticipantId participant_id,
    const double radius,
    std::shared_ptr<Writer> writer,
    std::shared_ptr<RectificationRequesterFactory> rectifier_factory)
{
  if (!writer)
  {
    throw std::runtime_error(
          "[rmf_traffic::blockade::make_participant] A nullptr was given for "
          "the `writer` argument. This is illegal.");
  }

  return Participant::Implementation::make(
        participant_id, radius, std::move(writer),
        std::move(rectifier_factory));
}

} // namespace blockade
} // namespace rmf_traffic
