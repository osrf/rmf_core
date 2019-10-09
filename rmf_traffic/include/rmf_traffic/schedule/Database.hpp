/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
#define RMF_TRAFFIC__SCHEDULE__DATABASE_HPP

#include <rmf_traffic/schedule/Viewer.hpp>

#include <rmf_utils/macros.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A class that maintains a database of scheduled Trajectories. This class is
/// intended to be used only for the canonical RMF traffic schedule database.
/// All local mirror copy
///
/// The Viewer API can be queried to find Trajectories that match certain
/// criteria.
///
/// You can also retrieve update patches from a database. To apply those patches
/// to a downstream Viewer, it is strongly advised to use the
/// rmf_traffic::schedule::Mirror class.
class Database : public Viewer
{
public:

  /// A class that describes a change within the database
  class Change
  {
  public:

    /// Enumeration for what type of change has occurred
    enum class Mode : uint16_t
    {
      /// A Trajectory was inserted
      Insert,

      /// A pause was introduced to a Trajectory
      Interrupt,

      /// A delay was introduced to a Trajectory
      Delay,

      /// A Trajectory was replaced by a new one
      Replace,

      /// A Trajectory was erased
      Erase,

      /// Some Trajectories were culled
      Cull,
    };

    /// Make an insertion change
    ///
    /// \param[in] trajectory
    ///   A pointer to the Trajectory that was inserted for this change, or a
    ///   nullptr if this insertion is voided later.
    ///
    /// \param[in] id
    ///   The ID of this insertion.
    static Change make_insert(
        const Trajectory* trajectory,
        std::size_t id);

    /// Make an interruption change
    ///
    /// \param[in] original_id
    ///   The original ID of the Trajectory that will be interrupted
    ///
    /// \param[in] interruption_trajectory
    ///   The trajectory that is being inserted as an interruption
    ///
    /// \param[in] delay
    ///   The additional delay following the interruption.
    ///
    /// \param[in] id
    ///   The ID of the modified Trajectory
    static Change make_interrupt(
        std::size_t original_id,
        const Trajectory* interruption_trajectory,
        Duration delay,
        std::size_t id);

    /// Make a delay change
    ///
    /// \param[in] original_id
    ///   The original ID of the Trajectory that will be delayed
    ///
    /// \param[in] from
    ///   The point in time where the delay originates
    ///
    /// \param[in] delay
    ///   The duration of the delay (how far back the Segments that come after
    ///   `from` should be pushed).
    ///
    /// \param[in] id
    ///   The ID of the modified Trajectory
    static Change make_delay(
        std::size_t original_id,
        Time from,
        Duration delay,
        std::size_t id);

    /// Make a replacement change
    ///
    /// \param[in] original_id
    ///   The original ID of the Trajectory that is being replaced
    ///
    /// \param[in] trajectory
    ///   A pointer to the Trajectory that was inserted for this change, or a
    ///   nullptr if this replacement is voided later.
    ///
    /// \param[in] id
    ///   The ID of this replacement.
    static Change make_replace(
        std::size_t original_id,
        const Trajectory* trajectory,
        std::size_t id);

    /// Make an erasure change
    ///
    /// \param[in] original_id
    ///   The ID of the Trajectory that is erased.
    ///
    /// \param[in] id
    ///   The ID of this erasure.
    static Change make_erase(
        std::size_t original_id,
        std::size_t id);

    /// Make a culling
    ///
    /// \param[in] culled
    ///   The set of IDs that were culled.
    ///
    /// \param[in] id
    ///   The ID of this culling
    static Change make_cull(
        std::vector<std::size_t> culled,
        std::size_t id);

    /// The API for an insertion
    class Insert
    {
    public:

      /// A pointer to the Trajectory that was inserted.
      ///
      /// If this returns a nullptr, then that implies that this insertion is
      /// void because the Patch will contain a Replace or Erase Change that
      /// nullifies it.
      const Trajectory* trajectory() const;

      class Implementation;
    private:
      Insert();
      RMF_UTILS__DEFAULT_COPY_MOVE(Insert);
      friend class Change;
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// The API for an interruption
    class Interrupt
    {
    public:

      /// The ID of the Trajectory that was interrupted.
      std::size_t original_id() const;

      /// A pointer to the Trajectory that was inserted.
      ///
      /// If this returns a nullptr, then that implies this interruption is void
      /// because the Patch will contain a Replace or Erase Change that
      /// nullifies it.
      const Trajectory* interruption() const;

      /// The length of the delay that follows the interruption.
      Duration delay() const;

      class Implementation;
    private:
      Interrupt();
      RMF_UTILS__DEFAULT_COPY_MOVE(Interrupt);
      friend class Change;
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// The API for a delay
    class Delay
    {
    public:

      /// The ID of the Trajectory that was delayed.
      std::size_t original_id() const;

      /// The time that the delay began.
      Time from() const;

      /// The duration of the delay.
      Duration duration() const;

      class Implementation;
    private:
      Delay();
      RMF_UTILS__DEFAULT_COPY_MOVE(Delay);
      friend class Change;
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// The API for a replacement
    class Replace
    {
    public:

      /// The ID of the Trajectory that was replaced
      std::size_t original_id() const;

      /// A pointer to the Trajectory that replaced it.
      ///
      /// If this returns a nullptr, then that implies that this replacement is
      /// void because the Patch will contain a Replace or Erase Change that
      /// nullifies it.
      const Trajectory* trajectory() const;

      class Implementation;
    private:
      Replace();
      RMF_UTILS__DEFAULT_COPY_MOVE(Replace);
      friend class Change;
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// The API for an erasure
    class Erase
    {
    public:

      /// The ID of the Trajectory that was erased.
      std::size_t original_id() const;

      class Implementation;
    private:
      Erase();
      RMF_UTILS__DEFAULT_COPY_MOVE(Erase);
      friend class Change;
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    class Cull
    {
    public:

      /// The set of IDs that have been culled from the schedule.
      const std::vector<std::size_t>& culled_ids() const;

      class Implementation;
    private:
      Cull();
      RMF_UTILS__DEFAULT_COPY_MOVE(Cull);
      friend class Change;
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// Get the type of Change
    Mode get_mode() const;

    /// Get the version ID that this change refers to
    std::size_t id() const;

    /// Get the Insert interface if this is an Insert type change. Otherwise
    /// this returns a nullptr.
    const Insert* insert() const;

    /// Get the Interrupt interface if this is an Interrupt type change.
    /// Otherwise this returns a nullptr.
    const Interrupt* interrupt() const;

    /// Get the Delay interface if this is a Delay type change. Otherwise this
    /// returns a nullptr.
    const Delay* delay() const;

    /// Get the Replace interface if this is a Replace type change. Otherwise
    /// this returns a nullptr.
    const Replace* replace() const;

    /// Get the Erase interface if this is an Erase type change. Otherwise this
    /// returns a nullptr.
    const Erase* erase() const;

    /// Get the Cull interface if this is a Cull type change. Otherwise this
    /// returns a nullptr.
    const Cull* cull() const;

    class Implementation;
  private:
    Change();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A container of Database changes
  class Patch
  {
  public:

    template<typename E, typename I, typename F>
    using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

    class IterImpl;
    using const_iterator = base_iterator<const Change, IterImpl, Patch>;

    Patch(std::vector<Change> changes, std::size_t latest_version);

    /// Returns an iterator to the first element of the Patch.
    const_iterator begin() const;

    /// Returns an iterator to the element following the last element of the
    /// Patch. This iterator acts as a placeholder; attempting to dereference it
    /// results in undefined behavior.
    const_iterator end() const;

    /// Get the number of elements in this Patch.
    std::size_t size() const;

    /// Get the latest version of the Database that informed this Patch.
    std::size_t latest_version() const;

    class Implementation;
  private:
    Patch();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Get the changes in this Database that match the given Query parameters.
  Patch changes(const Query& parameters) const;

  /// Insert a Trajectory into this database.
  ///
  /// \return The database id for this new Trajectory.
  std::size_t insert(Trajectory trajectory);

  /// Interrupt a trajectory by inserting another Trajectory inside of it.
  ///
  /// This will add each Segment of the input Trajectory into the targeted
  /// Trajectory entry in the database. All Segments in the original Trajectory
  /// that come after the start time of the interruption Trajectory will be
  /// pushed back in time by the whole duration of the interruption Trajectory.
  /// They will then be pushed back further by the duration of the `delay`
  /// argument.
  ///
  /// \param[in] id
  ///   The ID of the Trajectory to add the interruption to.
  ///
  /// \param[in] interruption_trajectory
  ///   The trajectory that should be inserted as an interruption of the
  ///   original trajectory.
  ///
  /// \param[in] delay
  ///   Additional delay that should follow the interruption. The total time
  ///   that the remaining Segments will be delayed from their original timing
  ///   is the duration of the `interruption_trajectory` plus the duration of
  ///   this `delay` argument.
  ///
  /// \return The updated ID for this modified Trajectory.
  ///
  /// \sa delay()
  std::size_t interrupt(
      std::size_t id,
      Trajectory interruption_trajectory,
      Duration delay);

  /// Add a delay to the Trajectory from the specified Time.
  ///
  /// Nothing about the Trajectory will be changed except that Segments which
  /// come after the specified time will be pushed back by the specified delay.
  ///
  /// \note This can create distortions in the Trajectory Segment that leads
  /// up to the `from` Time, so use with caution. This is primarily intended to
  /// make corrections to live Trajectories based on incoming state information.
  ///
  /// \note Unlike interrupt(), this will not introduce any new Segments to the
  /// Trajectory.
  ///
  /// \param[in] id
  ///   The ID of the Trajectory to delay.
  ///
  /// \param[in] from
  ///   All Trajectory Segments that end after this time point will be pushed
  ///   back by the delay.
  ///
  /// \param[in] delay
  ///   This is the duration of time to delay all qualifying Trajectory Segments
  ///
  /// \return The updated ID for this modified Trajectory
  ///
  /// \sa interrupt()
  std::size_t delay(
      std::size_t id,
      Time from,
      Duration delay);

  /// Replace an existing Trajectory with a new one. This is used for revising
  /// plans.
  ///
  /// \param[in] previous_id
  ///   The id of the previous Trajectory that is being replaced.
  ///
  /// \param[in] trajectory
  ///   The new trajectory to replace the old one with.
  ///
  /// \return The updated ID of the revised trajectory.
  std::size_t replace(std::size_t previous_id, Trajectory trajectory);

  /// Erase a Trajectory from this database.
  ///
  /// \return the new version of this database.
  std::size_t erase(std::size_t id);

  /// Throw away all Trajectories up to the specified time.
  ///
  /// \param[in] time
  ///   All Trajectories that finish before this time will be culled from the
  ///   Database. Their data will be completely deleted from this Database
  ///   object.
  ///
  /// \return The new version of the schedule database. If nothing was culled,
  /// this version number will remain the same.
  std::size_t cull(Time time);

};

} // namespace schedule

namespace detail {

extern template class bidirectional_iterator<
    const schedule::Database::Change,
    schedule::Database::Patch::IterImpl,
    schedule::Database::Patch
>;

}

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
