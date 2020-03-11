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

#ifndef RMF_TRAFFIC__AGV__ROUTEVALIDATOR_HPP
#define RMF_TRAFFIC__AGV__ROUTEVALIDATOR_HPP

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Negotiation.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
/// The RouteValidator class provides an interface for identifying whether a
/// given route can be considered valid.
class RouteValidator
{
public:

  /// Returns true if the given route can be considered valid, false otherwise.
  virtual bool valid(const Route& route) const = 0;

  virtual ~RouteValidator() = default;
};

//==============================================================================
class ScheduleRouteValidator : public RouteValidator
{
public:

  /// Constructor
  ///
  /// \warning You are expected to maintain the lifetime of the schedule
  /// viewer for as long as this ScheduleRouteValidator instance is alive. This
  /// object will only retain a reference to the viewer, not a copy of it.
  ///
  /// \param[in] viewer
  ///   The schedule viewer which will be used to check for conflicts
  ///
  /// \param[in] participant
  ///   The ID of the participant whose route is being validated. Any routes for
  ///   this participant on the schedule will be ignored while validating.
  ScheduleRouteValidator(
      const schedule::Viewer& viewer,
      schedule::ParticipantId participant);

  /// Change the schedule viewer to use for planning.
  ///
  /// \warning The Options instance will store a reference to the viewer; it
  /// will not store a copy. Therefore you are responsible for keeping the
  /// schedule viewer alive while this Options class is being used.
  ScheduleRouteValidator& schedule_viewer(const schedule::Viewer& viewer);

  /// Get a const reference to the schedule viewer that will be used for
  /// planning. It is undefined behavior to call this function is called after
  /// the schedule viewer has been destroyed.
  const schedule::Viewer& schedule_viewer() const;

  /// Set the ID of the participant that is being validated.
  ScheduleRouteValidator& participant(schedule::ParticipantId p);

  /// Get the ID of the participant that is being validated.
  schedule::ParticipantId participant() const;

  // Documentation inherited
  bool valid(const Route& route) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class NegotiatingRouteValidator : public RouteValidator
{
public:

  /// Constructor
  ///
  /// \param[in] table
  ///   The Negotiation::Table that the route must be valid on.
  NegotiatingRouteValidator(const schedule::Negotiation::Table& table);

  // Documentation inherited
  bool valid(const Route& route) const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__ROUTEVALIDATOR_HPP
