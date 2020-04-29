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

#include <unordered_map>

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

  /// Get the participants that are currently involved in this negotiation.
  const std::unordered_set<ParticipantId>& participants() const;

  /// Add a new participant to the negotiation. This participant will become
  /// involved in the negotiation, and must give its consent for any agreement
  /// to be finalized.
  void add_participant(ParticipantId p);

  /// Returns true if at least one proposal is available that has the consent of
  /// every participant.
  bool ready() const;

  /// Returns true if all possible proposals have been received and are ready to
  /// be evaluated.
  ///
  /// Note that ready() may still be false if complete() is true, in the event
  /// that all proposals have been rejected.
  bool complete() const;

  struct Submission
  {
    ParticipantId participant;
    Itinerary itinerary;
  };

  using Proposal = std::vector<Submission>;
  using Alternatives = std::vector<Itinerary>;

  class Table;
  using TablePtr = std::shared_ptr<Table>;
  using ConstTablePtr = std::shared_ptr<const Table>;

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
  class Table : public std::enable_shared_from_this<Table>
  {
  public:

    /// A struct used to specify the rollout for a participant.
    struct Rollout
    {
      ParticipantId participant;
      std::size_t alternative;
    };

    // TODO(MXG): This function should be put into a Table::Viewer class that
    // remains immutable so it can be used across different threads.
    //
    /// View this table with the given parameters.
    ///
    /// \param[in] parameters
    ///   The spacetime parameters to filter irrelevant routes out of the view
    ///
    /// \param[in] rollouts
    ///   The selection of which rollout alternatives should be viewed for the
    ///   participants who have rejected this proposal in the past.
    Viewer::View query(
        const Query::Spacetime& parameters,
        const std::vector<Rollout>& rollouts) const;

    // TODO(MXG): Replace this with a more meaningful Table::Viewer class
    const Viewer* viewer() const;

    /// Return the submission on this Negotiation Table if it has one.
    const Itinerary* submission() const;

    /// The a pointer to the latest itinerary version that was submitted to this
    /// table, if one was submitted at all.
    const Version* version() const;

    /// The proposal on this table so far. This will include the latest
    /// itinerary that has been submitted to this Table if anything has been
    /// submitted. Otherwise it will only include the submissions that underlie
    /// this table.
    Proposal proposal() const;

    /// The participant that is meant to submit to this Table.
    ParticipantId participant() const;

    /// The sequence key that refers to this table. This is equivalent to
    /// [to_accommodate..., for_participant]
    const std::vector<ParticipantId>& sequence() const;

    /// Submit a proposal for a participant that accommodates some of the other
    /// participants in the negotiation (or none if an empty vector is given for
    /// the to_accommodate argument).
    ///
    /// \param[in] itinerary
    ///   The itinerary that is being submitted by this participant.
    ///
    /// \param[in] version
    ///   A version number assigned to the submission. If this is less or equal
    ///   to the last version number given, then nothing will change.
    ///
    /// \return True if the submission was accepted. False if the version was
    /// out of date and nothing changed in the negotiation.
    bool submit(
      std::vector<Route> itinerary,
      Version version);

    /// Reject the submission of this Negotiation::Table. This
    /// indicates that the underlying proposals are infeasible for the
    /// Participant of this Table to accommodate. The rejecter should give a
    /// set of alternative rollouts that it is capable of. That way the proposer
    /// for this Table can submit an itinerary that accommodates it.
    ///
    /// \param[in] version
    ///   A version number assigned to the submission. If this is equal to or
    ///   greater than the last version number given, then this table will be
    ///   put into a rejected state until a higher proposal version is
    ///   submitted.
    ///
    /// \param[in] rejected_by
    ///   The participant who is rejecting this proposal
    ///
    /// \param[in] rollouts
    ///   A set of rollouts that could be used by the participant that is
    ///   rejecting this proposal. The proposer should use this information to
    ///   offer a proposal that can accommodate at least one of these rollouts.
    void reject(
        Version version,
        ParticipantId rejected_by,
        const Alternatives& rollouts);

    /// Returns true if the proposal put on this Table has been rejected.
    bool rejected() const;

    /// When a Negotiation::Table is rejected by one of the participants who is
    /// supposed to respond, they can offer a set of rollout alternatives. If
    /// the proposer can accommodate one of the rollouts, then the negotiation
    /// might be able to proceed. This map gives the rollout alternatives for
    /// each participant that has provided them.
    const std::unordered_map<ParticipantId, Alternatives>& rollouts() const;

    /// Give up on this Negotiation Table. This should be called when the
    /// participant that is supposed to submit to this Table is unable to find
    /// a feasible proposal.
    void forfeit(Version version);

    /// Returns true if the proposer for this Table has forfeited.
    bool forfeited() const;

    /// If by_participant can respond to this table, then this will return a
    /// TablePtr that by_participant can submit a proposal to.
    ///
    /// If this function is called before anything has been submitted to this
    /// Table, then it will certainly return a nullptr.
    TablePtr respond(ParticipantId by_participant);

    // const-qualified respond()
    ConstTablePtr respond(ParticipantId by_participant) const;

    /// Get the parent Table of this Table if it has a parent.
    TablePtr parent();

    // const-qualified parent()
    ConstTablePtr parent() const;

    /// Get the children of this Table if any children exist.
    std::vector<TablePtr> children();

    // const-qualified children()
    std::vector<ConstTablePtr> children() const;

    /// Return true if the negotiation is ongoing (i.e. the Negotiation instance
    /// that created this table is still alive). When the Negotiation instance
    /// that this Table belongs to has destructed, this will begin to return
    /// false.
    bool ongoing() const;

    class Implementation;
  private:
    Table();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
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
  TablePtr table(
    ParticipantId for_participant,
    const std::vector<ParticipantId>& to_accommodate);

  // const-qualified table()
  ConstTablePtr table(
    ParticipantId for_participant,
    const std::vector<ParticipantId>& to_accommodate) const;

  /// Get a Negotiation::Table that corresponds to the given participant
  /// sequence. For a table in terms of for_participant and to_accomodate, you
  /// would call:
  /// table([to_accommodate..., for_participant])
  ///
  /// \param[in] sequence
  ///   The participant sequence that corresponds to the desired table. This is
  ///   equivalent to [to_accommodate..., for_participant]
  TablePtr table(const std::vector<ParticipantId>& sequence);

  // const-qualified table()
  ConstTablePtr table(const std::vector<ParticipantId>& sequence) const;

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
  ///
  /// \return the negotiation table that was considered the best. Call
  /// Table::proposal() on this return value to see the full proposal. If there
  /// was no
  ConstTablePtr evaluate(const Evaluator& evaluator) const;

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
