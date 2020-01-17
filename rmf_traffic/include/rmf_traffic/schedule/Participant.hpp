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

#ifndef RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP
#define RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace schedule {

using ParticipantId = uint64_t;

//==============================================================================
class Participant
{
public:

  /// Enumeration for responsiveness.
  //
  // TODO(MXG): Should this be replaced with a boolean? Do we anticipate the
  // possibility of other categories of responsiveness that cannot be
  // encapsulated by the simple Responsive category?
  enum class Rx : uint16_t
  {
    /// This responsiveness type is illegal and will always be rejected by the
    /// schedule verifier. Having this movement type implies a major bug in the
    /// code and should be reported immediately.
    Unspecified = 0,

    /// The participant will not respond to any conflicts.
    Unresponsive,

    /// The participant will try to respond to conflicts.
    Responsive,
  };

  /// Constructor
  ///
  /// \param[in] name
  ///   The name of the object participating in the schedule.
  ///
  /// \param[in] owner
  ///   The name of the "owner" of this participant. This does not currently
  ///   have a formal definition, but for most vehicles it should be the name of
  ///   the fleet that the vehicle belongs to.
  ///
  /// \param[in] responsiveness
  ///   What category of responsiveness this participant has. A Responsive
  ///   participant might be able to react to a conflict or a request for
  ///   accommodations.
  ///
  /// \param[in] footprint
  ///   An estimate of the space that this participant occupies.
  ///
  /// \param[in] vicinity
  ///   An estimate of the vicinity around the participant in which the presence
  ///   of other traffic would disrupt its operations.
  ///
  Participant(
      std::string name,
      std::string owner,
      Rx responsiveness,
      geometry::ConstFinalConvexShapePtr footprint,
      geometry::ConstFinalConvexShapePtr vicinity);

  /// Set the name of the participant.
  Participant& name(std::string value);

  /// Get the name of the participant.
  const std::string& name() const;

  /// Set the name of the "owner" of the participant.
  Participant& owner(std::string value);

  /// Get the name of the "owner" of the participant.
  const std::string& owner() const;

  /// Set the responsiveness of the participant.
  Participant& responsiveness(Rx value);

  /// Get the responsiveness of the participant.
  Rx responsiveness() const;

  /// Set the footprint of the participant.
  Participant& footprint(geometry::ConstFinalConvexShapePtr shape);

  /// Get the footprint of the participant.
  const geometry::ConstFinalConvexShapePtr& footprint() const;

  /// Set the vicinity of this participant.
  Participant& vicinity(geometry::ConstFinalConvexShapePtr shape);

  /// Get the vicinity of this participant.
  const geometry::ConstFinalConvexShapePtr& vicinity() const;

private:
  class Implementation;
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_traffic
} // namespace schedule

#endif // RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP
