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

#include <rmf_utils/clone_ptr.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
/// The RouteValidator class provides an interface for identifying whether a
/// given route can be considered valid.
class RouteValidator
{
public:

  using ParticipantId = schedule::ParticipantId;
  using Route = rmf_traffic::Route;

  struct Conflict
  {
    ParticipantId participant;
    Time time;
  };

  /// If the specified route has a conflict with another participant, this will
  /// return the participant ID for the first conflict that gets identified.
  /// Otherwise it will return a nullopt.
  ///
  /// \param[in] route
  ///   The route that is being checked.
  //
  // TODO(MXG): It would be better to implement this as a coroutine generator so
  // that clients of this interface can decide whether they only care about the
  // first conflict or if they want to know all of the conflicts. This is not a
  // high priority because in the vast majority of cases when a conflict happens
  // it will only be with one participant. And since this is only meant to
  // provide a hint about which participant is causing conflicts, it is okay if
  // other participants are ignored.
  virtual rmf_utils::optional<Conflict> find_conflict(
      const Route& route) const = 0;

  /// Create a clone of the underlying RouteValidator object.
  virtual std::unique_ptr<RouteValidator> clone() const = 0;

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
  ///
  /// \param[in] profile
  ///   The profile for the participant. This is not inferred from the
  ///   viewer because the viewer might not be synced with the schedule by the
  ///   time this validator is being used.
  ScheduleRouteValidator(
    const schedule::Viewer& viewer,
    schedule::ParticipantId participant_id,
    Profile profile);

  /// Constructor
  ///
  /// This constructor will assume that the profile for the participant can be
  /// found inside the viewer.
  ///
  /// \param[in] viewer
  ///   The schedule viewer which will be used to check for conflicts. The
  ///   reference to the viewer will be kept alive.
  ///
  /// \param[in] participant_id
  ///   The ID for the participant that is being validated.
  ScheduleRouteValidator(
    std::shared_ptr<const schedule::Viewer> viewer,
    schedule::ParticipantId participant_id);

  /// Constructor
  ///
  /// This constructor will use the profile given to it for the participant that
  /// is being planned for. This is safe to use, even if the participant is not
  /// registered in the schedule yet.
  ///
  /// \param[in] viewer
  ///   The schedule viewer which will be used ot check for conflicts. The
  ///   reference to the viewer will be kept alive.
  ///
  /// \param[in] participant_id
  ///   The ID for the participant that is being validated.
  ///
  /// \param[in] profile
  ///   The profile for the participant.
  ScheduleRouteValidator(
    std::shared_ptr<const schedule::Viewer> viewer,
    schedule::ParticipantId participant_id,
    Profile profile);

  /// Make the ScheduleRouteValidator as a clone_ptr
  template<typename... Args>
  static rmf_utils::clone_ptr<ScheduleRouteValidator> make(Args&&... args)
  {
    return rmf_utils::make_clone<ScheduleRouteValidator>(
          std::forward<Args>(args)...);
  }

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

  // TODO(MXG): Make profile setters and getters

  // Documentation inherited
  rmf_utils::optional<Conflict> find_conflict(const Route& route) const final;

  // Documentation inherited
  std::unique_ptr<RouteValidator> clone() const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class NegotiatingRouteValidator : public RouteValidator
{
public:

  /// The Generator class begins the creation of NegotiatingRouteValidator
  /// instances. NegotiatingRouteValidator may be able to brach in multiple
  /// dimensions because of the rollout alternatives that are provided during a
  /// rejection.
  class Generator
  {
  public:

    /// Constructor
    ///
    /// \param[in] table
    ///   The Negotiating Table that the generated validators are concerned with
    ///
    /// \param[in] profile
    ///   The profile of the participant whose routes are being validated.
    Generator(
        schedule::Negotiation::Table::ViewerPtr viewer,
        rmf_traffic::Profile profile);

    /// Start with a NegotiatingRouteValidator that will use all the most
    /// preferred alternatives from every participant.
    NegotiatingRouteValidator begin() const;

    /// Get the set of participants who have specified what their available
    /// rollouts are.
    const std::vector<schedule::ParticipantId>& alternative_sets() const;

    /// Get the number of alternative rollouts for the specified participant.
    /// This function will throw an excpetion if participant does not offer an
    /// alternative set.
    std::size_t alternative_count(schedule::ParticipantId participant) const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Mask the given Participant so that conflicts with it will be ignored. In
  /// the current implementation, only one participant can be masked at a time.
  ///
  /// \param[in] id
  ///   The ID of a participant whose conflicts should be ignored when checking
  ///   for collisions.
  NegotiatingRouteValidator& mask(schedule::ParticipantId id);

  /// Remove any mask that has been applied using the mask() function.
  NegotiatingRouteValidator& remove_mask();

  /// Get a NegotiatingRouteValidator for the next rollout alternative offered
  /// by the given participant.
  NegotiatingRouteValidator next(schedule::ParticipantId id) const;

  /// Get the set of child Table alternatives used by this
  /// NegotiatingRouteValidator.
  const schedule::Negotiation::VersionedKeySequence& alternatives() const;

  /// Implicitly cast this validator instance to true if it can be used as a
  /// validator. If it cannot be used as a validator, return false. This will
  /// have the opposite value of end().
  operator bool() const;

  /// Return true if this validator object has gone past the end of its limits.
  /// Return false if it can still be used as a validator.
  bool end() const;

  // Documentation inherited
  rmf_utils::optional<Conflict> find_conflict(const Route& route) const final;

  // Documentation inherited
  std::unique_ptr<RouteValidator> clone() const final;

  class Implementation;
private:
  NegotiatingRouteValidator();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__ROUTEVALIDATOR_HPP
