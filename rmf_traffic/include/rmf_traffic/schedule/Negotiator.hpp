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

#ifndef RMF_TRAFFIC__SCHEDULE__NEGOTIATOR_HPP
#define RMF_TRAFFIC__SCHEDULE__NEGOTIATOR_HPP

#include <rmf_traffic/schedule/Negotiation.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A pure abstract interface class that facilitates negotiating a resolution to
/// a schedule conflict. An example implementation of this class can be found
/// as rmf_traffic::agv::Negotiator.
class Negotiator
{
public:

  /// A pure abstract interface class that allows the Negotiator to respond to
  /// other Negotiators.
  class Responder
  {
  public:

    /// The negotiator will call this function when it has an itinerary to
    /// submit in response to a negotiation.
    ///
    /// \param[in] itinerary
    ///   The itinerary that is being proposed
    ///
    /// \param[in] approval_callback
    ///   This callback will get triggered if this submission gets approved.
    ///   Pass in a nullptr if a callback is not necessary.
    virtual void submit(
        std::vector<Route> itinerary,
        std::function<void()> approval_callback = nullptr) const = 0;

    /// The negotiator will call this function if it has decided to reject an
    /// attempt to negotiate.
    virtual void reject() const = 0;

    // Virtual destructor
    virtual ~Responder() = default;
  };

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
  ///   A pointer to a flag that can be used to interrupt the negotiator if it
  ///   has been running for too long. If the planner should run indefinitely,
  ///   then pass a nullptr.
  virtual void respond(
      std::shared_ptr<const schedule::Negotiation::Table> table,
      const Responder& responder,
      const bool* interrupt_flag = nullptr) = 0;

  virtual ~Negotiator() = default;
};

//==============================================================================
/// A simple implementation of a Negotiator::Responder. It simply passes the
/// result along to the Negotiation.
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
  // NOTE: approval_callback does not get used
  void submit(
      std::vector<Route> itinerary,
      std::function<void()> approval_callback = nullptr) const final;

  // Documentation inherited
  void reject() const final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__NEGOTIATOR_HPP
