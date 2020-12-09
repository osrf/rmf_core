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

#ifndef RMF_TRAFFIC__BLOCKADE__RECTIFIER_HPP
#define RMF_TRAFFIC__BLOCKADE__RECTIFIER_HPP

#include <rmf_traffic/blockade/Status.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
/// The Rectifier class provides an interface for telling a Participant to
/// rectify an inconsistency in the information received by a moderator. This
/// rectification protocol is important when the blockades are being managed
/// over an unreliable network.
///
/// The Rectifier class can be used by a RectifierRequester to ask a participant
/// to retransmit a range of its past status changes.
///
/// Only the Participant class is able to create a Rectifier instance. Users of
/// rmf_traffic cannot instantiate a Rectifier.
class Rectifier
{
public:

  /// Check that the given status is up to date, and retransmit if any
  /// information is out of sync.
  void check(const Status& status);

  /// Check that there should not be a status for this participant. If that is
  /// a mistake and this participant *should* have a status, then retransmit the
  /// necessary information.
  void check();

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
/// Rectifier::check() on the Rectifier that was assigned to it.
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
/// layers for the blockade system.
class RectificationRequesterFactory
{
public:

  /// Create a RectificationRequester to be held by a Participant
  ///
  /// \param[in] rectifier
  ///   This rectifier can be used by the RectificationRequester to ask the
  ///   participant to check its status.
  ///
  /// \param[in] participant_id
  ///   The ID of the participant that will hold onto this
  ///   RectificationRequester. This is the same participant that the rectifier
  ///   will request for checks.
  virtual std::unique_ptr<RectificationRequester> make(
    Rectifier rectifier,
    ParticipantId participant_id) = 0;

  // virtual destructor
  virtual ~RectificationRequesterFactory() = default;

};

//==============================================================================
// Forward declaration for ModeratorRectificationRequesterFactory
class Moderator;

//==============================================================================
/// This class provides a simple implementation of a
/// RectificationRequesterFactory that just hooks directly into a Moderator
/// instance and issues rectification requests when told to based on the current
/// inconsistencies in the Database.
class ModeratorRectificationRequesterFactory
    : public RectificationRequesterFactory
{
public:

  /// Constructor
  ///
  /// \param[in] moderator
  ///   The moderator object that this will rectify for.
  ModeratorRectificationRequesterFactory(std::shared_ptr<Moderator> moderator);

  // Documentation inherited
  std::unique_ptr<RectificationRequester> make(
      Rectifier rectifier,
      ParticipantId participant_id) final;

  /// Call this function to instruct all the RectificationRequesters produced by
  /// this factory to perform their rectifications.
  void rectify();

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};


} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__RECTIFIER_HPP
