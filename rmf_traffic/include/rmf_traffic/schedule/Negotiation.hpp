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

#ifndef RMF_TRAFFIC__SCHEDULE__NEGOTIATION_HPP
#define RMF_TRAFFIC__SCHEDULE__NEGOTIATION_HPP

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Negotiation
{
public:

  // TODO(MXG): Add an API that allows a multi-participant planner to propose
  // globally optimal itineraries.

  /// Constructor
  ///
  /// \warning You are expected to maintain the lifetime of the schedule
  /// viewer for as long as this Negotiation instance is alive. This object
  /// will only retain a reference to the viewer, not a copy of it.
  ///
  /// \param[in] viewer
  ///   A reference to the schedule viewer that represents the most up-to-date
  ///   schedule.
  ///
  /// \param[in] participants
  ///   The participants who are involved in the schedule negotiation.
  ///
  Negotiation(
      const Viewer& viewer,
      std::vector<ParticipantId> participants);

  /// Add a new participant to the negotiation. This participant will become
  /// involved in the negotiation, and must give its consent for any agreement
  /// to be finalized.
  void add_participant(ParticipantId p);

  /// Submit a proposal for a participant that accommodates some of the other
  /// participants in the negotiation (or none if an empty vector is given for
  /// the to_accommodate argument).
  ///
  /// \warning A runtime error may be thrown if the proposal hierarchy specified
  /// by to_accommodate has not been submitted yet.
  ///
  /// \param[in] for_participant
  ///   The participant whose itinerary is being submitted.
  ///
  /// \param[in] to_accommodate
  ///   The participants whose submissions are being accommodated. When
  ///   submitting the ideal itinerary for a participant, this vector should be
  ///   empty.
  ///
  /// \param[in] itinerary
  ///   The itinerary that is being submitted by this participant.
  void submit(
      ParticipantId for_participant,
      std::vector<ParticipantId> to_accommodate,
      std::vector<Route> itinerary);

  /// Returns true if at least one proposal is available that has the consent of
  /// every participant.
  bool ready() const;

  /// Returns true if all possible proposals have been received and are ready to
  /// be evaluated.
  bool complete() const;

  struct Submission
  {
    ParticipantId participant;
    Itinerary itinerary;
  };

  using Proposal = std::vector<Submission>;

  /// A pure abstract interface class for choosing the best proposal.
  class Evaluator
  {
  public:

    /// Given a set of proposals, choose the one that is the "best". It is up to
    /// the implementation of the Evaluator to decide how to rank proposals.
    virtual std::size_t choose(
        const std::vector<const Proposal*>& proposals) const = 0;

    virtual ~Evaluator() = default;
  };

  /// Evaluate the proposals that are available.
  const Proposal* evaluate(const Evaluator& evaluator) const;

  /// The Negotiation::Table class gives a view of what the other negotiation
  /// participants have proposed.
  ///
  /// A Table instance is meant to be viewed by a specific participant and
  /// displays the proposals of other participants for a specific hierarchies of
  /// accommodations. See the documentation of Negotiation::table().
  ///
  /// Alongside the views of the other Negotiation participants, the View
  /// provided by the Table instance will show the itineraries of schedule
  /// participants that are not part of the Negotiation. That way the external
  /// itineraries can also be accounted for when planning a submission based on
  /// this Table.
  class Table
  {
  public:

    /// View this table with the given parameters.
    Viewer::View query(const Query& parameters) const;

    class Implementation;
  private:
    Table();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Get a Negotiation::Table that provides a view into what participants are
  /// proposing.
  ///
  /// \param[in] for_participant
  ///   The participant that is supposed to be viewing this Table. The
  ///   itineraries of this participant will be left off of the Table.
  ///
  /// \param[in] to_accommodate
  ///   The set of participants who are being accommodated at this Table. The
  ///   ordering of the participants in this set is hierarchical where each
  ///   participant is accommodating all of the participants that come before
  ///   it.
  const Table* table(
      ParticipantId for_participant,
      const std::vector<ParticipantId>& to_accommodate) const;

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// An implementation of an evaluator that chooses the proposal that minimizes
/// net delays in completing the itineraries.
class QuickestFinishEvaluator : public Negotiation::Evaluator
{
public:

  // Documentation inherited
  std::size_t choose(
      const std::vector<const Negotiation::Proposal*>& proposals) const final;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__NEGOTIATION_HPP
