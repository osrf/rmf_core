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
#include <rmf_traffic/schedule/Writer.hpp>

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
class Database : public Viewer, public Writer
{
public:

  //============================================================================
  // Writer API
  //============================================================================

  /// Documentation inherited from Writer
  void put(
      ParticipantId participant,
      Itinerary itinerary,
      bool retransmission) final;

  /// Documentation inherited from Writer
  void post(
      ParticipantId participant,
      Itinerary itinerary,
      bool retransmission) final;

  /// Documentation inherited from Writer
  void delay(
      ParticipantId participant,
      Time from,
      Duration delay,
      ItineraryVersion version,
      bool retransmission) final;

  /// Documentation inherited from Writer
  void erase(
      ParticipantId participant,
      ItineraryVersion version,
      bool retransmission) final;

  /// Documentation inherited from Writer
  void erase(
      ParticipantId participant,
      const std::vector<ItineraryVersion>& routes,
      ItineraryVersion version,
      bool retransmission) final;

  /// Documentation inherited from Writer
  ParticipantId register_participant(
      ParticipantDescription participant_info) final;

  /// Documentation inherited from Writer
  void unregister_participant(ParticipantId participant) final;


  //============================================================================
  // Viewer API
  //============================================================================

  /// Documentation inherited from Viewer
  View query(const Query& parameters) const final;

  /// Documentation inherited from Viewer
  const std::unordered_set<ParticipantId>& participant_ids() const final;

  /// Documentation inherited from Viewer
  rmf_utils::optional<const ParticipantDescription&> get_participant(
      std::size_t participant_id) const final;

  /// Documentation inherited from Viewer
  rmf_utils::optional<Itinerary> get_itinerary(
      std::size_t participant_id) const final;

  /// Documentation inherited from Viewer
  Version oldest_version() const final;

  /// Documentation inherited from Viewer
  Version latest_version() const final;


  //============================================================================
  // Database API
  //============================================================================

  /// Initialize a Database
  Database();

  /// Get the changes in this Database that match the given Query parameters.
  /// If a version number is specified, then the returned Patch will reflect the
  /// changes that occurred from the specified version to the current version of
  /// the schedule.
  ///
  /// To get a consistent reflection of the schedule when specifying a base
  /// version, it is important that the query parameters are not changed in
  /// between calls.
  ///
  /// \param[in] parameters
  ///   The parameters describing what types of schedule entries the mirror
  ///   cares about.
  ///
  /// \param[in] after
  ///   Specify that only changes which come after this version number are
  ///   desired. If you give a nullopt for this argument, then all changes will
  ///   be provided.
  ///
  /// \return A Patch of schedule changes that are relevant to the specified
  /// query parameters.
  Patch changes(
      const Query& parameters,
      rmf_utils::optional<Version> after) const;

  /// Throw away all itineraries up to the specified time.
  ///
  /// \param[in] time
  ///   All Trajectories that finish before this time will be culled from the
  ///   Database. Their data will be completely deleted from this Database
  ///   object.
  ///
  /// \return The new version of the schedule database. If nothing was culled,
  /// this version number will remain the same.
  Version cull(Time time);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
