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
#include <rmf_traffic/schedule/Patch.hpp>

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

  /// Initialize a Database
  Database();

  /// Get the changes in this Database that match the given Query parameters.
  Patch changes(const Query& parameters) const;

  /// Insert a Trajectory into this database.
  ///
  /// \return The database id for this new Trajectory.
  Version insert(Trajectory trajectory);

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
  Version interrupt(
      Version id,
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
  Version delay(
      Version id,
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
  Version replace(Version previous_id, Trajectory trajectory);

  /// Erase a Trajectory from this database.
  ///
  /// \return the new version of this database.
  Version erase(Version id);

  /// Throw away all Trajectories up to the specified time.
  ///
  /// \param[in] time
  ///   All Trajectories that finish before this time will be culled from the
  ///   Database. Their data will be completely deleted from this Database
  ///   object.
  ///
  /// \return The new version of the schedule database. If nothing was culled,
  /// this version number will remain the same.
  Version cull(Time time);

};

} // namespace schedule

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
