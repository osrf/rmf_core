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

#ifndef RMF_TRAFFIC__SCHEDULE__VIEWER_HPP
#define RMF_TRAFFIC__SCHEDULE__VIEWER_HPP

#include <rmf_traffic/detail/bidirectional_iterator.hpp>

#include <rmf_traffic/schedule/Query.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Itinerary.hpp>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/macros.hpp>
#include <rmf_utils/optional.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A pure abstract interface class that allows users to query for itineraries
/// that are in a schedule.
///
/// This class cannot be instantiated directly. To get a Viewer, you must
/// instantiate an rmf_traffic::schedule::Database or an
/// rmf_traffic::schedule::Mirror object.
class Viewer
{
public:

  /// A read-only view of some Trajectories in a Database or Mirror.
  ///
  /// It is undefined behavior to modify a Database or patch a Mirror while
  /// reading Trajectories from this view. The user of this class is responsible
  /// for managing access to reads vs access to writes.
  class View
  {
  public:

    template<typename E, typename I, typename F>
    using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

    // TODO(MXG): Replace this with a PIMPL class
    struct Element
    {
      const ParticipantId participant;
      const RouteId route_id;
      const Route& route;
      const ParticipantDescription& description;
    };

    class IterImpl;
    using const_iterator = base_iterator<const Element, IterImpl, View>;
    using iterator = const_iterator;

    /// Returns an iterator to the first element of the View
    const_iterator begin() const;

    /// Returns an iterator to the element following the last element of the
    /// View.
    const_iterator end() const;

    /// Returns the number of elements in this View.
    std::size_t size() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Query this Viewer to get a View of the Trajectories inside of it that
  /// match the Query parameters.
  virtual View query(const Query& parameters) const = 0;

  /// Alternative signature for query()
  virtual View query(
    const Query::Spacetime& spacetime,
    const Query::Participants& participants) const = 0;

  // TODO(MXG): Consider providing an iterator-style API to view participant IDs
  // and participant descriptions.

  /// Get the set of active participant IDs.
  virtual const std::unordered_set<ParticipantId>& participant_ids() const = 0;

  /// Get the information of the specified participant if it is available.
  /// If a participant with the specified ID is not registered with the
  /// schedule, then this will return a nullptr.
  virtual std::shared_ptr<const ParticipantDescription> get_participant(
    ParticipantId participant_id) const = 0;

  /// Get the latest version number of this Database.
  virtual Version latest_version() const = 0;

  // Virtual destructor
  virtual ~Viewer() = default;

  // The Debug class is for internal testing use only. Its definition is not
  // visible to downstream users.
  class Debug;
  class Implementation;
};

//==============================================================================
/// A pure abstract interface class that extends Viewer to allow users to
/// explicitly request the itinerary of a specific participant.
///
/// \note This interface class is separate from Viewer because it is not
/// generally needed by the traffic planning or negotiation systems, and the
/// Snapshot class can perform better if it does not need to provide this
/// function.
class ItineraryViewer : public virtual Viewer
{
public:

  /// Get the itinerary of a specific participant if it is available. If a
  /// participant with the specified ID is not registered with the schedule or
  /// has never submitted an itinerary, then this will return a nullopt.
  virtual rmf_utils::optional<Itinerary> get_itinerary(
    std::size_t participant_id) const = 0;

  // Virtual destructor
  virtual ~ItineraryViewer() = default;
};

} // namespace schedule

namespace detail {

extern template class bidirectional_iterator<
    const schedule::Viewer::View::Element,
    schedule::Viewer::View::IterImpl,
    schedule::Viewer::View
>;

} // namespace detail
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__VIEWER_HPP
