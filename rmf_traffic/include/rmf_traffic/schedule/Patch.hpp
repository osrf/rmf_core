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

#ifndef RMF_TRAFFIC__SCHEDULE__PATCH_HPP
#define RMF_TRAFFIC__SCHEDULE__PATCH_HPP

#include <rmf_traffic/schedule/Change.hpp>

#include <rmf_traffic/detail/bidirectional_iterator.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A container of Database changes
class Patch
{
public:

  template<typename E, typename I, typename F>
  using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

  class Participant
  {
  public:

    /// Constructor
    ///
    /// \param[in] id
    ///   The ID of the participant that is being changed
    ///
    /// \param[in] erasures
    ///   The information about which routes to erase
    ///
    /// \param[in] delays
    ///   The information about what delays have occurred
    ///
    /// \param[in] additions
    ///   The information about which routes to add
    Participant(
        ParticipantId id,
        Change::Erase erasures,
        std::vector<Change::Delay> delays,
        Change::Add additions);

    /// The ID of the participant that this set of changes will patch.
    ParticipantId participant_id() const;

    /// The route erasures to perform.
    ///
    /// These erasures should be performed before any other changes.
    const Change::Erase& erasures() const;

    /// The sequence of delays to apply.
    ///
    /// These delays should be applied in sequential order after the erasures
    /// are performed, and before any additions are performed.
    const std::vector<Change::Delay>& delays() const;

    /// The set of additions to perfom.
    ///
    /// These additions should be applied after all other changes.
    const Change::Add& additions() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class IterImpl;
  using const_iterator = base_iterator<const Participant, IterImpl, Patch>;

  /// Constructor. Mirrors should evaluate the fields of the Patch class in the
  /// order of these constructor arguments.
  ///
  /// \param[in] removed_participants
  ///   Information about which participants have been unregistered since the
  ///   last update.
  ///
  /// \param[in] new_participants
  ///   Information about which participants have been registered since the last
  ///   update.
  ///
  /// \param[in] changes
  ///   Information about how the participants have changed since the last
  ///   update.
  ///
  /// \param[in] cull
  ///   Information about how the database has culled old data since the last
  ///   update.
  ///
  /// \param[in] latest_version
  ///   The lastest version of the database that this Patch represents.
  Patch(
      std::vector<Change::UnregisterParticipant> removed_participants,
      std::vector<Change::RegisterParticipant> new_participants,
      std::vector<Participant> changes,
      rmf_utils::optional<Change::Cull> cull,
      Version latest_version);

  // TODO(MXG): Consider using a persistent reliable topic to broadcast the
  // active participant information instead of making it part of the patch.
  // Ideally this information would not need to change frequently, so it doesn't
  // necessarily need to be in the Patch scheme. The addition and loss of
  // participants is significant enough that we should guarantee it's always
  // transmitted correctly.
  //
  // The current scheme makes an assumption that remote mirrors will always
  // either sync up before unregistered participant information is culled, or
  // else they will perform a complete refresh. This might be a point of
  // vulnerability if a remote mirror is not being managed correctly.

  /// Get a list of which participants have been unregistered. This should be
  /// evaluated first in the patch.
  const std::vector<Change::UnregisterParticipant>& unregistered() const;

  /// Get a list of new participants that have been registered. This should be
  /// evaluated after the unregistered participants.
  const std::vector<Change::RegisterParticipant>& registered() const;

  /// Returns an iterator to the first element of the Patch.
  const_iterator begin() const;

  /// Returns an iterator to the element following the last element of the
  /// Patch. This iterator acts as a placeholder; attempting to dereference it
  /// results in undefined behavior.
  const_iterator end() const;

  /// Get the number of elements in this Patch.
  std::size_t size() const;

  /// Get the cull information for this patch if a cull has occurred.
  const Change::Cull* cull() const;

  /// Get the latest version of the Database that informed this Patch.
  Version latest_version() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule

namespace detail {

extern template class bidirectional_iterator<
    const schedule::Patch::Participant,
    schedule::Patch::IterImpl,
    schedule::Patch
>;

}

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__PATCH_HPP
