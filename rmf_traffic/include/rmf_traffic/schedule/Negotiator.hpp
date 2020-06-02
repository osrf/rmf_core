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

  using TableViewerPtr = Negotiation::Table::ViewerPtr;

  /// A pure abstract interface class that allows the Negotiator to respond to
  /// other Negotiators.
  class Responder
  {
  public:

    using ParticipantId = rmf_traffic::schedule::ParticipantId;
    using ItineraryVersion = rmf_traffic::schedule::ItineraryVersion;
    using UpdateVersion = rmf_utils::optional<ItineraryVersion>;
    using ApprovalCallback = std::function<UpdateVersion()>;
    using Alternatives = Negotiation::Alternatives;

    /// The negotiator will call this function when it has an itinerary to
    /// submit in response to a negotiation.
    ///
    /// \param[in] itinerary
    ///   The itinerary that is being proposed
    ///
    /// \param[in] approval_callback
    ///   This callback will get triggered if this submission gets approved.
    ///   The return value of the callback should be the itinerary version of
    ///   the participant update that will follow the resolution of this
    ///   negotiation (or a nullopt if no update will be performed). Pass in a
    ///   nullptr if an approval callback is not necessary.
    virtual void submit(
      std::vector<Route> itinerary,
      ApprovalCallback approval_callback = nullptr) const = 0;

    /// The negotiator will call this function if it has decided to reject an
    /// attempt to negotiate. It must supply a set of alternatives for the
    /// parent negotiator to consider for its next proposal.
    virtual void reject(const Alternatives& alternatives) const = 0;

    /// The negotiator will call this function if it cannot find any feasible
    /// proposal or alternative that can be accommodated by the parent.
    ///
    /// \param[in] blockers
    ///   Give the set of schedule participants that are blocking a solution
    ///   from being found.
    virtual void forfeit(const std::vector<ParticipantId>& blockers) const = 0;

    // Virtual destructor
    virtual ~Responder() = default;
  };

  using ResponderPtr = std::shared_ptr<const Responder>;

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
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder) = 0;

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
  /// \param[in] table
  ///   The table
  ///
  /// \param[in] report_blockers
  ///   If the blockers should be reported when a forfeit is given, provide a
  ///   pointer to a vector of ParticipantIds.
  SimpleResponder(
    const Negotiation::TablePtr& table,
    std::vector<schedule::ParticipantId>* report_blockers = nullptr);

  template<typename... Args>
  static std::shared_ptr<SimpleResponder> make(Args&&... args)
  {
    return std::make_shared<SimpleResponder>(std::forward<Args>(args)...);
  }

  // Documentation inherited
  // NOTE: approval_callback does not get used
  void submit(
    std::vector<Route> itinerary,
    std::function<UpdateVersion()> approval_callback = nullptr) const final;

  // Documentation inherited
  void reject(
      const Negotiation::Alternatives& alternatives) const final;

  // Documentation inherited
  void forfeit(const std::vector<ParticipantId>& blockers) const final;

  /// Get the blockers that were reported by the Negotiator, if a forfeit
  /// was given.
  const std::vector<ParticipantId>& blockers() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__NEGOTIATOR_HPP
