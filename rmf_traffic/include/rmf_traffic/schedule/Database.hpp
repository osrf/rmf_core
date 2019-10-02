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
    enum class Mode
    {
      /// A Trajectory was inserted
      Insert,

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
        Trajectory* trajectory,
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
        Trajectory* trajectory,
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
      std::vector<std::size_t> culled_ids() const;

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
    using iterator = base_iterator<Change, IterImpl, Patch>;
    using const_iterator = base_iterator<const Change, IterImpl, Patch>;

    /// Returns an iterator to the first element of the Patch.
    const_iterator begin() const;

    /// Returns an iterator to the element following the last element of the
    /// Patch. This iterator acts as a placeholder; attempting to dereference it
    /// results in undefined behavior.
    const_iterator end() const;

    /// Get the number of elements in this Patch.
    std::size_t size() const;

    /// Get the latest version that this Patch knows of.
    std::size_t latest_version() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Get the changes in this Database that match the given Query parameters.
  Patch changes(Query parameters) const;

  /// Get the oldest version number inside this Database.
  std::size_t oldest_version() const;

  /// Get the latest version number of this Database.
  std::size_t latest_version() const;

  /// Insert a Trajectory into this database.
  ///
  /// \return The database id for this new Trajectory.
  std::size_t insert(Trajectory trajectory);


  /// Replace an existing Trajectory with a new one. This is used for revising
  /// plans.
  ///
  /// \param[in] previous_id
  ///   The id of the previous Trajectory that is being replaced.
  ///
  /// \param[in] trajectory
  ///   The new trajectory to replace the old one with.
  ///
  /// \return The updated id of the revised trajectory.
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
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
