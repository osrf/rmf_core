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

#ifndef RMF_TRAFFIC__SCHEDULE__RECTIFIER_HPP
#define RMF_TRAFFIC__SCHEDULE__RECTIFIER_HPP

#include <rmf_traffic/schedule/Itinerary.hpp>
#include <rmf_traffic/schedule/ParticipantDescription.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// The Rectifier class provides an interface for telling a Participant to
/// rectify an inconsistency in the information received by a database. This
/// rectification protocol is important when the schedule is being managed over
/// an unreliable network.
///
/// The Rectifier class can be used by a RectifierRequester to ask a participent
/// to retransmit a range of its past itinerary changes.
///
/// Only the Participant class is able to create a Rectifier instance. Users of
/// rmf_traffic cannot instantiate a Rectifier.
class Rectifier
{
public:

  /// A range of itinerary change IDs that is currently missing from a database.
  /// All IDs from lower to upper are missing, including lower and upper
  /// themselves.
  ///
  /// It is undefined behavior if the value given to upper is less than the
  /// value given to upper.
  struct Range
  {
    /// The ID of the first itinerary change in this range that is missing
    ItineraryVersion lower;

    /// The ID of the last itinerary change in this range that is missing
    ItineraryVersion upper;
  };

  /// Ask the participant to retransmit the specified range of its itinerary
  /// changes.
  ///
  /// \param[in] ranges
  ///   The ranges of missing Itinerary IDs
  ///
  /// \param[in] last_known_version
  ///   The last ItineraryVersion known upstream.
  void retransmit(
      const std::vector<Range>& ranges,
      ItineraryVersion last_known_version);

  /// Get the current ItineraryVersion of the Participant.
  ItineraryVersion current_version() const;

  class Implementation;
private:
  Rectifier();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// RectificationRequester is a pure abstract class which should be implemented
/// for any middlewares that intend to act as transport layers for the
/// scheduling system.
///
/// Classes that derive from RectificationRequester do not need to implement any
/// interfaces, but they should practice RAII. The lifecycle of the
/// RectificationRequester will be tied to the Participant that it was created
/// for.
///
/// When a schedule database reports an inconsistency for the participant tied
/// to a RectificationRequester instance, the instance should call
/// Rectifier::retransmit() on the Rectifier that was assigned to it.
class RectificationRequester
{
public:

  /// This destructor is pure virtual to ensure that a derived class is
  /// instantiated.
  virtual ~RectificationRequester() = 0;
};

//==============================================================================
/// The RectificationRequesterFactory is a pure abstract interface class which
/// should be implemented for any middlewares that intend to act as transport
/// layers for the scheduling system.
class RectificationRequesterFactory
{
public:

  virtual std::unique_ptr<RectificationRequester> make(
      Rectifier rectifier,
      ParticipantId participant_id) = 0;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__RECTIFIER_HPP
