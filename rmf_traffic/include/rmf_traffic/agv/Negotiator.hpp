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

#ifndef RMF_TRAFFIC__AGV__NEGOTIATOR_HPP
#define RMF_TRAFFIC__AGV__NEGOTIATOR_HPP

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Negotiator
{
public:

  /// A class to specify user-defined options for the Negotiator.
  class Options
  {
  public:

    /// Constructor
    ///
    /// \param[in] min_hold_time
    ///   The minimum amount of time that the planner should spend waiting at
    ///   holding points. See Planner::Options for more information.
    Options(
        Duration min_hold_time = Planner::Options::DefaultMinHoldingTime);

    /// Set the minimum amount of time to spend waiting at holding points
    Options& minimum_holding_time(Duration holding_time);

    /// Get the minimum amount of time to spend waiting at holding points
    Duration minimum_holding_time() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A pure abstract interface class that allows the Negotiator to respond to
  /// other Negotiators.
  class Responder
  {
  public:

    /// The negotiator will call this function when it has an itinerary to
    /// submit in response to a negotiation.
    virtual void submit(std::vector<Route> itinerary) const = 0;

    /// The negotiator will call this function if it has decided to reject an
    /// attempt to negotiate.
    virtual void reject() const = 0;

    // Virtual destructor
    virtual ~Responder() = default;
  };

  /// Constructor
  ///
  /// \param[in] planner_configuration
  ///   The configuration that will be used by the planner underlying this
  ///   Negotiator.
  ///
  /// \param[in] options
  ///   Additional options that will be used by the Negotiator.
  Negotiator(
      Planner::Start start,
      Planner::Goal goal,
      Planner::Configuration planner_configuration,
      const Options& options = Options());

  /// Have the Negotiator respond to an attempt to negotiate.
  ///
  /// \param[in] table
  ///   The Negotiation::Table that is being used for the negotiation.
  ///
  /// \param[in] responder
  ///   The Responder instance that the negotiator should use when a response is
  ///   ready.
  ///
  /// \param[in] interrupt_flag
  ///   A pointer to a flag that can be used to interrupt the planning if it has
  ///   been running for too long. If the planner should run indefinitely, then
  ///   pass a nullptr.
  void respond(
      std::shared_ptr<const schedule::Negotiation::Table> table,
      const Responder& responder,
      const bool* interrupt_flag = nullptr) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class SimpleResponder : public Negotiator::Responder
{
public:

  /// Constructor
  ///
  /// \param[in] negotiation
  ///   The Negotiation that this SimpleResponder is tied to
  ///
  /// \param[in] for_participant
  ///   The participant that will respond using this SimpleResponder
  ///
  /// \param[in] to_accommodate
  ///   The participants that will be accommodated by the response
  SimpleResponder(
      std::shared_ptr<schedule::Negotiation> negotiation,
      schedule::ParticipantId for_participant,
      std::vector<schedule::ParticipantId> to_accommodate);

  // Documentation inherited
  void submit(std::vector<Route> itinerary) const final;

  // Documentation inherited
  void reject() const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__NEGOTIATOR_HPP
