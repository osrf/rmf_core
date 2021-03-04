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

#include <rmf_traffic/schedule/Negotiation.hpp>

#include "Timeline.hpp"
#include "ViewerInternal.hpp"

#include <rmf_utils/Modular.hpp>

namespace rmf_traffic {
namespace schedule {
namespace {

//==============================================================================
std::size_t factorial(std::size_t N)
{
  std::size_t output = 1;
  while (N > 1)
  {
    output *= N;
    --N;
  }

  return output;
}

//==============================================================================
/// When a set of proposals is rejected, all the possible negotiation tables
/// that could have branched off of it are also rejected. This calculates how
/// many possible negotiations are termined by a rejection of the given depth.
/// For example:
///
/// Participants: [1, 2, 3, 4]
/// num_participants: 4
///
/// Rejected: [2, 1]
/// depth: 2
///
/// Terminated: [2, 1, 3, 4], [2, 1, 4, 3]
/// termination_factor: 2
///
/// \param[in] depth
///   The depth of rejection
///
/// \param[in] num_participants
///   The total number of participants in the negotiation
std::size_t termination_factor(
  const std::size_t depth,
  const std::size_t num_participants)
{
  return factorial(num_participants - depth);
}

//==============================================================================
Itinerary convert_itinerary(std::vector<Route> itinerary)
{
  Itinerary output;
  output.reserve(itinerary.size());
  for (auto&& r : itinerary)
    output.emplace_back(std::make_shared<Route>(std::move(r)));

  return output;
}

//==============================================================================
struct NegotiationData
{
  /// The participants that are part of the negotiation
  std::unordered_set<ParticipantId> participants;

  /// The negotiation tables that have successfully reached a termination
  std::vector<Negotiation::VersionedKeySequence> successful_tables;

  /// The number of negotiation tables that have reached a conclusion (either
  /// successfully finished or rejected)
  std::size_t num_terminated_tables = 0;

  std::unordered_set<Negotiation::Table::Implementation*> forfeited_tables;

  void clear_successful_descendants_of(
    const Negotiation::VersionedKeySequence& sequence)
  {
    const auto erase_it = std::remove_if(
      successful_tables.begin(),
      successful_tables.end(),
      [&](const Negotiation::VersionedKeySequence& table)
      {
        for (std::size_t i = 0; i < sequence.size(); ++i)
        {
          if (table[i].participant != sequence[i].participant)
            return false;
        }

        return true;
      });

    successful_tables.erase(erase_it, successful_tables.end());
  }
};

struct RouteEntry
{
  ConstRoutePtr route;
  ParticipantId participant;
  RouteId route_id;
  std::shared_ptr<const ParticipantDescription> description;
};
using ConstRouteEntryPtr = std::shared_ptr<const RouteEntry>;

using AlternativeTimelinePtr =
  std::shared_ptr<const TimelineView<const RouteEntry>>;

using AlternativesTimelineMap = std::vector<AlternativeTimelinePtr>;
using ParticipantToAlternativesMap =
  std::unordered_map<ParticipantId, AlternativesTimelineMap>;

} // anonymous namespace

//==============================================================================
class Negotiation::Table::Viewer::Endpoint::Implementation
{
public:

  enum Type {
    Initial,
    Final
  };

  Type type;
  ParticipantId participant;
  std::shared_ptr<const Route> route;
  std::shared_ptr<const ParticipantDescription> description;

  static Endpoint make_initial(
    ParticipantId participant,
    std::shared_ptr<const Route> route,
    std::shared_ptr<const ParticipantDescription> description)
  {
    Endpoint output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        Initial,
        participant,
        std::move(route),
        std::move(description)
      });

    return output;
  }

  static Endpoint make_final(
    ParticipantId participant,
    std::shared_ptr<const Route> route,
    std::shared_ptr<const ParticipantDescription> description)
  {
    Endpoint output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        Final,
        participant,
        std::move(route),
        std::move(description)
      });

    return output;
  }

};

//==============================================================================
class Negotiation::Table::Viewer::Implementation
{
public:

  AlternativeTimelinePtr proposed_timeline;
  ParticipantToAlternativesMap alternatives_timelines;
  AlternativeMap alternatives;
  std::shared_ptr<Proposal> base_proposals;
  std::shared_ptr<Query::Participants> participant_query;
  std::shared_ptr<const schedule::Viewer> schedule_viewer;
  rmf_utils::optional<ParticipantId> parent_id;
  VersionedKeySequence sequence;
  std::shared_ptr<const bool> defunct;
  bool rejected;
  bool forfeited;
  std::optional<Itinerary> itinerary;

  std::unordered_map<ParticipantId, Endpoint> initial_endpoints = {};
  std::unordered_map<ParticipantId, Endpoint> final_endpoints = {};

  Viewer::View query(
    const Query::Spacetime& spacetime,
    const VersionedKeySequence& rollouts) const;

  template<typename... Args>
  static Viewer make(Args&& ... args)
  {
    Viewer output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::forward<Args>(args)...});

    output._pimpl->_make_endpoints();

    return output;
  }

  static void insert_initial_endpoint(
    std::unordered_map<ParticipantId, Endpoint>& initial_endpoints,
    const ParticipantId participant,
    const std::shared_ptr<const ParticipantDescription>& description,
    const Itinerary& itinerary)
  {
    ConstRoutePtr initial = nullptr;
    for (const auto& r : itinerary)
    {
      const auto& check = r->trajectory().front().time();
      if (!initial || check < initial->trajectory().front().time())
      {
        initial = r;
      }
    }

    initial_endpoints.insert(
    {
      participant,
      Endpoint::Implementation::make_initial(participant, initial, description)
    });
  }

  static void insert_final_endpoint(
    std::unordered_map<ParticipantId, Endpoint>& final_endpoints,
    const ParticipantId participant,
    const std::shared_ptr<const ParticipantDescription>& description,
    const Itinerary& itinerary)
  {
    ConstRoutePtr final = nullptr;
    for (const auto& r : itinerary)
    {
      const auto& check = r->trajectory().back().time();
      if (!final || final->trajectory().front().time() < check)
      {
        final = r;
      }
    }

    final_endpoints.insert(
    {
      participant,
      Endpoint::Implementation::make_final(participant, final, description)
    });
  }


  std::unordered_map<ParticipantId, Endpoint> get_initial_endpoints(
    const VersionedKeySequence& alt_keys) const
  {
    auto output = initial_endpoints;

    for (const auto& key : alt_keys)
    {
      const auto& description =
        schedule_viewer->get_participant(key.participant);

      insert_initial_endpoint(
        output,
        key.participant,
        description,
        alternatives.at(key.participant)->at(key.version));
    }

    return output;
  }

  std::unordered_map<ParticipantId, Endpoint> get_final_endpoints(
    const VersionedKeySequence& alt_keys) const
  {
    auto output = final_endpoints;

    for (const auto& key : alt_keys)
    {
      const auto& description =
        schedule_viewer->get_participant(key.participant);

      insert_final_endpoint(
        output,
        key.participant,
        description,
        alternatives.at(key.participant)->at(key.version));
    }

    return output;
  }

private:


  void _make_endpoints()
  {
    std::unordered_map<ParticipantId, ConstRoutePtr> initial;
    for (const auto& p : *base_proposals)
    {
      const auto& description = schedule_viewer->get_participant(p.participant);

      insert_initial_endpoint(
        initial_endpoints,
        p.participant,
        description,
        p.itinerary);

      insert_final_endpoint(
        final_endpoints,
        p.participant,
        description,
        p.itinerary);
    }
  }

};

namespace {
//==============================================================================
/// An RAII wrapper for a "defunct" flag. When the wrapper is destructed, it
/// will automatically set the defunct flag to true to indicate that its holder
/// is no longer alive. It can also be flipped to defunct at any time using the
/// terminate() function.
class DefunctFlag
{
public:

  DefunctFlag()
    : _defunct(std::make_shared<bool>(false))
  {
    // Do nothing
  }

  DefunctFlag(const DefunctFlag&) = delete;
  DefunctFlag& operator=(const DefunctFlag&) = delete;

  DefunctFlag(DefunctFlag&&) = default;
  DefunctFlag& operator=(DefunctFlag&&) = default;

  ~DefunctFlag()
  {
    if (_defunct)
      *_defunct = true;
  }

  operator bool() const
  {
    return *_defunct;
  }

  std::shared_ptr<const bool> get() const
  {
    return _defunct;
  }

  void terminate()
  {
    *_defunct = true;
  }

private:
  std::shared_ptr<bool> _defunct;
};
} // anonymous namespace

//==============================================================================
class Negotiation::Table::Implementation
{
public:

  using TableMap = std::unordered_map<ParticipantId, std::shared_ptr<Table>>;

  std::shared_ptr<const schedule::Viewer> const schedule_viewer;
  VersionedKeySequence sequence;
  std::vector<ParticipantId> unsubmitted;

  // ===== Fields that get copied into a Viewer =====
  AlternativeTimelinePtr proposed_timeline;
  ParticipantToAlternativesMap alternatives_timelines;
  Viewer::AlternativeMap alternatives;
  std::shared_ptr<Query::Participants> participant_query;
  // ================================================

  // Lazy evaluation on this viewer allows us to minimize pointless copy
  // operations
  mutable ViewerPtr cached_table_viewer;

  std::shared_ptr<Proposal> base_proposals;
  Proposal proposal;

  const ParticipantId participant;
  const std::size_t depth;
  rmf_utils::optional<Itinerary> itinerary;
  bool rejected = false;
  bool forfeited = false;
  DefunctFlag defunct;
  TableMap descendants;

  Version& version()
  {
    return sequence.back().version;
  }

  Version version() const
  {
    return sequence.back().version;
  }

  std::weak_ptr<NegotiationData> weak_negotiation_data;
  std::weak_ptr<Table> weak_owner;
  std::weak_ptr<Table> weak_parent;

  Implementation(
    TablePtr owner_,
    std::shared_ptr<NegotiationData> negotiation_data_,
    std::shared_ptr<const schedule::Viewer> schedule_viewer_,
    ParticipantId participant_,
    std::size_t depth_,
    VersionedKeySequence submitted_,
    std::vector<ParticipantId> unsubmitted_,
    Proposal initial_proposal_,
    TablePtr parent_)
  : schedule_viewer(std::move(schedule_viewer_)),
    unsubmitted(std::move(unsubmitted_)),
    base_proposals(std::make_shared<Proposal>(std::move(initial_proposal_))),
    proposal(*base_proposals),
    participant(participant_),
    depth(depth_),
    weak_negotiation_data(negotiation_data_),
    weak_owner(owner_),
    weak_parent(std::move(parent_))
  {
    std::vector<std::shared_ptr<void>> handles;
    Timeline<RouteEntry> timeline_builder;

    for (const auto& p : proposal)
    {
      const ParticipantId participant = p.participant;
      const auto& description = schedule_viewer->get_participant(participant);
      for (std::size_t i = 0; i < p.itinerary.size(); ++i)
      {
        const auto& route = p.itinerary[i];

        auto entry = std::make_shared<RouteEntry>(
          RouteEntry{
            route,
            participant,
            i,
            description
          });

        handles.push_back(timeline_builder.insert(entry));
      }
    }

    proposed_timeline = timeline_builder.snapshot();

    std::vector<ParticipantId> all_participants;
    all_participants.reserve(submitted_.size() + unsubmitted_.size());
    for (const auto& s : submitted_)
      all_participants.push_back(s.participant);

    all_participants.insert(
      all_participants.end(), unsubmitted.begin(), unsubmitted.end());

    participant_query = std::make_shared<Query::Participants>(
      Query::Participants::make_all_except(std::move(all_participants)));

    // Add this Table's participant to the sequence
    sequence = std::move(submitted_);
    sequence.push_back({participant, 0});

    // Remove this Table's participant from the unsubmitted
    const auto it = std::remove_if(unsubmitted.begin(), unsubmitted.end(),
        [&](const ParticipantId p) { return p == participant; });
    assert(it != unsubmitted.end());
    unsubmitted.erase(it, unsubmitted.end());

    assert(std::find(unsubmitted.begin(),
      unsubmitted.end(), participant) == unsubmitted.end());
  }

  void add_participant(const ParticipantId new_participant)
  {
    assert(std::find(unsubmitted.begin(),
      unsubmitted.end(), new_participant) == unsubmitted.end());

    assert(std::find_if(sequence.begin(), sequence.end(),
      [&new_participant](const VersionedKey& key)
      {
        return key.participant == new_participant;
      }) == sequence.end());

    unsubmitted.push_back(new_participant);

    if (itinerary)
    {
      // If we already have a submission for this table, then immediately add
      // a descendent for this new participant.
      make_descendent(new_participant);
    }
  }

  void make_descendants()
  {
    assert(itinerary);
    assert(proposal.size() == depth);

    assert(std::find(unsubmitted.begin(),
      unsubmitted.end(), participant) == unsubmitted.end());

    std::unordered_map<ParticipantId, Table> descendants;
    for (const auto u : unsubmitted)
      make_descendent(u);
  }

  void make_descendent(const ParticipantId p)
  {
    auto table = std::make_shared<Table>(Table());
    table->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      table, weak_negotiation_data.lock(), schedule_viewer, p, depth+1,
      sequence, unsubmitted, proposal, weak_owner.lock());

    descendants.insert(std::make_pair(p, std::move(table)));
  }

  static TablePtr make_root(
    std::shared_ptr<const schedule::Viewer> schedule_viewer,
    const std::shared_ptr<NegotiationData> negotiation_data,
    const ParticipantId participant,
    const std::vector<ParticipantId>& participants)
  {
    auto table = std::make_shared<Table>(Table());
    table->_pimpl = rmf_utils::make_unique_impl<Implementation>(
      Implementation(
        table, negotiation_data, std::move(schedule_viewer),
        participant, 1, {}, participants, {}, nullptr));

    return table;
  }

  static Implementation& get(Table& table)
  {
    return *table._pimpl;
  }

  static const Implementation& get(const Table& table)
  {
    return *table._pimpl;
  }

  bool submit(
    std::vector<Route> new_itinerary,
    const Version new_version)
  {
    if (rmf_utils::modular(new_version).less_than_or_equal(version()))
      return false;

    version() = new_version;

    const bool had_itinerary = itinerary.has_value();
    bool formerly_successful = false;

    const auto negotiation_data = weak_negotiation_data.lock();
    if (forfeited && negotiation_data)
    {
      negotiation_data->forfeited_tables.erase(this);
      negotiation_data->num_terminated_tables -=
        termination_factor(depth, negotiation_data->participants.size());
    }
    else if (had_itinerary && descendants.empty())
    {
      // This means that this was a successful terminating node, so we should
      // make note of that to keep our bookkeeping correct.
      formerly_successful = true;
    }

    itinerary = convert_itinerary(new_itinerary);
    rejected = false;
    forfeited = false;

    if (had_itinerary)
    {
      proposal.back() = {participant, *itinerary};
      clear_descendants();
    }
    else
    {
      proposal.push_back({participant, *itinerary});
    }

    make_descendants();

    if (descendants.empty() && !formerly_successful && negotiation_data)
    {
      // If there are no new tables that branch off of this submission, then
      // this submission has successfully terminated this branch of
      // negotiation
      negotiation_data->successful_tables.push_back(sequence);
      negotiation_data->num_terminated_tables += 1;
    }

    return true;
  }

  AlternativesTimelineMap to_timelines(
    const ParticipantId participant,
    const Alternatives& alternatives) const
  {
    AlternativesTimelineMap output;
    const auto& description = schedule_viewer->get_participant(participant);

    for (const auto& alternative : alternatives)
    {
      Timeline<RouteEntry> timeline;
      std::vector<std::shared_ptr<void>> handles;

      std::size_t id = 0;
      for (const auto& route : alternative)
      {
        auto entry = std::make_shared<RouteEntry>(
          RouteEntry{
            route,
            participant,
            id,
            description
          });

        handles.push_back(timeline.insert(entry));
      }

      output.emplace_back(timeline.snapshot());
    }

    return output;
  }

  bool reject(
    const Version rejected_version,
    ParticipantId rejected_by,
    Alternatives offered_alternatives)
  {
    // TODO(MXG): I should also keep track of the rejection version of the
    // rejecter. That way we can correctly identify if the offered_alternatives
    // have been updated or not.
    if (rmf_utils::modular(rejected_version).less_than(version()))
      return false;

    cached_table_viewer.reset();

    alternatives_timelines[rejected_by] =
      to_timelines(rejected_by, offered_alternatives);

    this->alternatives[rejected_by] =
      std::make_shared<Alternatives>(std::move(offered_alternatives));

    version() = rejected_version;

    // We return true here because the alternatives updated which may be
    // relevant to the negotiation participant, even if the rejected status is
    // unchanged.
    if (rejected)
      return true;

    const auto negotiation_data = weak_negotiation_data.lock();
    if (itinerary && descendants.empty() && negotiation_data)
    {
      // This used to be a successfully completed negotiation table.
      // TODO(MXG): It's a bit suspicious that a successfully completed
      // negotiation table would get rejected. Maybe we should put an
      // assertion here.
      negotiation_data->num_terminated_tables -= 1;
    }

    if (itinerary)
    {
      itinerary = rmf_utils::nullopt;
      proposal.pop_back();
    }

    rejected = true;
    clear_descendants();

    if (negotiation_data)
    {
      // Erase any successful tables that branched off of this rejected table
      negotiation_data->clear_successful_descendants_of(sequence);
    }

    return true;
  }

  void forfeit(const Version forfeited_version)
  {
    // TODO(MXG): Consider if this function's implementation can be refactored
    // with reject()
    if (rmf_utils::modular(forfeited_version).less_than(version()))
      return;

    version() = forfeited_version;

    if (forfeited)
      return;

    const auto negotiation_data = weak_negotiation_data.lock();
    if (itinerary && descendants.empty() && negotiation_data)
    {
      // This used to be a successfully completed negotiation table.
      // TODO(MXG): It's a bit suspicious that a successfully completed
      // negotiation table would get forfeited. Maybe we should put an
      // assertion here.
      negotiation_data->num_terminated_tables -= 1;
    }

    if (itinerary)
    {
      itinerary = rmf_utils::nullopt;
      proposal.pop_back();
    }

    forfeited = true;
    clear_descendants();

    if (negotiation_data)
    {
      negotiation_data->num_terminated_tables +=
        termination_factor(depth, negotiation_data->participants.size());
      negotiation_data->forfeited_tables.insert(this);

      negotiation_data->clear_successful_descendants_of(sequence);
    }
  }

  // This function removes descendent tables and makes sure that they can no
  // longer impact the negotiation
  void clear_descendants()
  {
    // TODO(MXG): Instead of clearing all the descendants during a submission,
    // we could simply clear the ones whose proposals are in conflict with the
    // new submission. However, all descendants must always be cleared during a
    // forfeit.

    const auto negotiation_data = weak_negotiation_data.lock();

    std::vector<Table::Implementation*> queue;
    queue.push_back(this);
    while (!queue.empty())
    {
      auto top = queue.back();
      queue.pop_back();

      for (const auto& entry : top->descendants)
      {
        const auto& table = entry.second;
        if (table->_pimpl->forfeited && negotiation_data)
        {
          negotiation_data->num_terminated_tables -=
            termination_factor(table->_pimpl->depth,
              negotiation_data->participants.size());

          negotiation_data->forfeited_tables.erase(table->_pimpl.get());
        }

        table->_pimpl->weak_negotiation_data.reset();
        // Tell the child tables that they are now defunct
        table->_pimpl->defunct.terminate();
        queue.push_back(entry.second->_pimpl.get());
      }
    }

    descendants.clear();
  }
};

//==============================================================================
class Negotiation::Implementation
{
public:

  Implementation(
    std::shared_ptr<const schedule::Viewer> schedule_viewer_,
    std::vector<ParticipantId> participants_)
  : schedule_viewer(std::move(schedule_viewer_)),
    data(std::make_shared<NegotiationData>())
  {
    for (const auto p : participants_)
      data->participants.insert(p);

    max_terminated_tables = factorial(data->participants.size());

    for (const auto p : participants_)
    {
      tables[p] = Table::Implementation::make_root(
        schedule_viewer, data, p, participants_);
    }
  }

  std::shared_ptr<const schedule::Viewer> schedule_viewer;
  std::size_t max_terminated_tables;

  using TableMap = Table::Implementation::TableMap;
  TableMap tables;

  std::shared_ptr<NegotiationData> data;

  TablePtr climb(const TableMap& map, const ParticipantId p)
  {
    const auto it = map.find(p);
    if (it == map.end())
      return nullptr;

    return it->second;
  }

  TablePtr get_entry(
    const std::vector<ParticipantId>& table)
  {
    // TODO(MXG): We could use a TablePtr* here to avoid unnecessary reference
    // counting. However, it would add another layer of indirection and make the
    // nullptr semantics much trickier.
    TablePtr output = nullptr;
    const TableMap* map = &tables;
    for (const auto p : table)
    {
      output = climb(*map, p);
      if (!output)
        return nullptr;

      map = &Table::Implementation::get(*output).descendants;
    }

    return output;
  }

  ConstTablePtr get_entry(const std::vector<ParticipantId>& table) const
  {
    return const_cast<Implementation&>(*this).get_entry(table);
  }

  TablePtr get_entry(
    const ParticipantId for_participant,
    const std::vector<ParticipantId>& to_accommodate)
  {
    TableMap* map = nullptr;
    if (to_accommodate.empty())
    {
      map = &tables;
    }
    else
    {
      const auto output = get_entry(to_accommodate);
      if (!output)
        return nullptr;

      map = &Table::Implementation::get(*output).descendants;
    }
    assert(map);

    return climb(*map, for_participant);
  }

  ConstTablePtr get_entry(
    const ParticipantId for_participant,
    const std::vector<ParticipantId>& to_accommodate) const
  {
    return const_cast<Implementation&>(*this).get_entry(
      for_participant, to_accommodate);
  }

  SearchResult<TablePtr> find_entry(
    const VersionedKeySequence& sequence)
  {
    TablePtr parent = nullptr;
    TablePtr output = nullptr;
    const TableMap* map = &tables;
    for (const auto key : sequence)
    {
      parent = output;
      output = climb(*map, key.participant);
      if (!output)
      {
        if (parent && (parent->rejected() || parent->forfeited()))
        {
          // If this table can't be found but the parent table has a rejected or
          // forfeited status, that means the unfound table has been deprecated;
          // not that it is absent.
          return {SearchStatus::Deprecated, nullptr};
        }

        return {SearchStatus::Absent, nullptr};
      }

      if (key.version < output->version())
        return {SearchStatus::Deprecated, nullptr};

      if (output->version() < key.version)
        return {SearchStatus::Absent, nullptr};

      map = &Table::Implementation::get(*output).descendants;
    }

    return {SearchStatus::Found, output};
  }

  SearchResult<ConstTablePtr> find_entry(
    const VersionedKeySequence& sequence) const
  {
    auto output = const_cast<Implementation&>(*this).find_entry(sequence);
    return {output.status, std::move(output.table)};
  }

  SearchResult<TablePtr> find_entry(
    const ParticipantId for_participant,
    const VersionedKeySequence& to_accommodate)
  {
    TableMap* map = nullptr;
    if (to_accommodate.empty())
    {
      map = &tables;
    }
    else
    {
      const auto output = find_entry(to_accommodate);
      if (!output)
        return output;

      map = &Table::Implementation::get(*output.table).descendants;
    }
    assert(map);

    auto output = climb(*map, for_participant);
    if (!output)
      return {SearchStatus::Absent, nullptr};

    return {SearchStatus::Found, output};
  }

  SearchResult<ConstTablePtr> find_entry(
    const ParticipantId for_participant,
    const VersionedKeySequence& to_accommodate) const
  {
    auto output = const_cast<Implementation&>(*this).find_entry(
      for_participant, to_accommodate);
    return {output.status, output.table};
  }

  void add_participant(const ParticipantId new_participant)
  {
    if (!data->participants.insert(new_participant).second)
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[rmf_traffic::schedule::Negotiation::add_participant] "
        "Participant [" + std::to_string(
          new_participant) + "] is already "
        "present in the Negotiation");
      // *INDENT-ON*
    }

    // We can update the maximum number of terminating tables by just
    // multiplying by the new size of negotiation participants, because the
    // value should be the factorial of the total number of negotiation
    // participants.
    max_terminated_tables *= data->participants.size();

    // With a new participant, none of the successfully terminated negotiations
    // are valid anymore.
    data->successful_tables.clear();

    // Restart the counter to zero.
    data->num_terminated_tables = 0;

    // The rejected tables are still terminated, but the number of tables that
    // are terminated due to rejection will be higher now, so we need to
    // recalculate it.
    const std::size_t N = data->participants.size();
    for (const auto rejected : data->forfeited_tables)
      data->num_terminated_tables += termination_factor(rejected->depth, N);

    std::vector<TableMap*> queue;
    std::vector<Table::Implementation*> current_tables;
    queue.push_back(&tables);
    while (!queue.empty())
    {
      auto* next = queue.back();
      queue.pop_back();

      for (auto& element : *next)
      {
        const auto& entry = element.second;
        current_tables.push_back(&Table::Implementation::get(*entry));
        queue.push_back(&Table::Implementation::get(*entry).descendants);
      }
    }

    // We collect all the current tables before adding the new participant to
    // any of them, because if we try to traverse the tables and add the
    // participant at the same time, we might accidentally traverse over tables
    // that are being freshly created for the new participant.
    for (auto* t : current_tables)
      t->add_participant(new_participant);

    tables[new_participant] = Table::Implementation::make_root(
      schedule_viewer, data, new_participant,
      std::vector<ParticipantId>(
        data->participants.begin(),
        data->participants.end()));
  }

  ~Implementation()
  {
    std::vector<Table::Implementation*> queue;
    for (auto& table : tables)
      queue.push_back(&Table::Implementation::get(*table.second));

    while (!queue.empty())
    {
      auto top = queue.back();
      queue.pop_back();

      for (const auto& entry : top->descendants)
      {
        auto& table = Table::Implementation::get(*entry.second);
        table.defunct.terminate();
        queue.push_back(&table);
      }
    }
  }
};

//==============================================================================
rmf_utils::optional<Negotiation> Negotiation::make(
  std::shared_ptr<const schedule::Viewer> schedule_viewer,
  std::vector<ParticipantId> participants)
{
  if (!schedule_viewer)
    return rmf_utils::nullopt;

  for (const auto p : participants)
  {
    if (!schedule_viewer->get_participant(p))
      return rmf_utils::nullopt;
  }

  Negotiation negotiation;
  negotiation._pimpl = rmf_utils::make_unique_impl<Implementation>(
    std::move(schedule_viewer), std::move(participants));
  return negotiation;
}

//==============================================================================
std::shared_ptr<Negotiation> Negotiation::make_shared(
  std::shared_ptr<const schedule::Viewer> schedule_viewer,
  std::vector<ParticipantId> participants)
{
  auto negotiation = make(std::move(schedule_viewer), std::move(participants));
  if (!negotiation)
    return nullptr;

  return std::make_shared<Negotiation>(*std::move(negotiation));
}

//==============================================================================
const std::unordered_set<ParticipantId>& Negotiation::participants() const
{
  return _pimpl->data->participants;
}

//==============================================================================
void Negotiation::add_participant(ParticipantId p)
{
  _pimpl->add_participant(p);
}

//==============================================================================
bool Negotiation::ready() const
{
  return !_pimpl->data->successful_tables.empty();
}

//==============================================================================
bool Negotiation::complete() const
{
//  return _pimpl->data->num_terminated_tables == _pimpl->max_terminated_tables;
  // TODO(MXG): Figure out what bug or misconception is allowing the
  // num_terminated_tables to exceed max_terminated_tables
  return _pimpl->data->num_terminated_tables >= _pimpl->max_terminated_tables;
}

namespace {
//==============================================================================
class NegotiationRelevanceInspector : public TimelineInspector<RouteEntry>
{
public:

  using Storage = schedule::Viewer::View::Implementation::Storage;

  std::vector<Storage> routes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    assert(entry->route);
    if (relevant(*entry))
    {
      routes.emplace_back(
        Storage{
          entry->participant,
          entry->route_id,
          entry->route,
          entry->description
        });
    }
  }
};
} // anonymous namespace

//==============================================================================
Viewer::View Negotiation::Table::Viewer::Implementation::query(
  const Query::Spacetime& spacetime,
  const VersionedKeySequence& chosen_alternatives) const
{
  const auto& all_participants = Query::Participants::make_all();

  // Query for the relevant routes that are being negotiated
  NegotiationRelevanceInspector inspector;
  proposed_timeline->inspect(spacetime, all_participants, inspector);

  // Query for the routes in the child rollouts that are being considered
  for (const auto& alternative : chosen_alternatives)
  {
    const auto& participant_alternatives =
        alternatives_timelines.at(alternative.participant);
    assert(alternative.version < participant_alternatives.size());

    participant_alternatives.at(alternative.version)
        ->inspect(spacetime, all_participants, inspector);
  }

  // Query for the relevant routes that are outside of the negotiation
  Viewer::View view = schedule_viewer->query(spacetime, *participant_query);

  // Merge them together into a single view
  Viewer::View::Implementation::append_to_view(
    view, std::move(inspector.routes));

  return view;
}

//==============================================================================
Viewer::View Negotiation::Table::Viewer::query(
  const Query::Spacetime& parameters,
  const VersionedKeySequence& alternatives) const
{
  return _pimpl->query(parameters, alternatives);
}

//==============================================================================
ParticipantId Negotiation::Table::Viewer::Endpoint::participant() const
{
  return _pimpl->participant;
}

//==============================================================================
const rmf_traffic::Trajectory::Waypoint&
Negotiation::Table::Viewer::Endpoint::waypoint() const
{
  if (Implementation::Initial == _pimpl->type)
    return _pimpl->route->trajectory().front();

  return _pimpl->route->trajectory().back();
}

//==============================================================================
const std::string& Negotiation::Table::Viewer::Endpoint::map() const
{
  return _pimpl->route->map();
}

//==============================================================================
const ParticipantDescription&
Negotiation::Table::Viewer::Endpoint::description() const
{
  return *_pimpl->description;
}

//==============================================================================
Negotiation::Table::Viewer::Endpoint::Endpoint()
{
  // Do nothing
}

//==============================================================================
auto Negotiation::Table::Viewer::initial_endpoints(
  const VersionedKeySequence& alternatives) const
-> std::unordered_map<ParticipantId, Endpoint>
{
  return _pimpl->get_initial_endpoints(alternatives);
}

//==============================================================================
auto Negotiation::Table::Viewer::final_endpoints(
  const VersionedKeySequence& alternatives) const
-> std::unordered_map<ParticipantId, Endpoint>
{
  return _pimpl->get_final_endpoints(alternatives);
}

//==============================================================================
auto Negotiation::Table::Viewer::alternatives() const -> const AlternativeMap&
{
  return _pimpl->alternatives;
}

//==============================================================================
auto Negotiation::Table::Viewer::base_proposals() const -> const Proposal&
{
  return *_pimpl->base_proposals;
}

//==============================================================================
std::shared_ptr<const ParticipantDescription>
Negotiation::Table::Viewer::get_description(ParticipantId participant_id) const
{
  return _pimpl->schedule_viewer->get_participant(participant_id);
}

//==============================================================================
ParticipantId Negotiation::Table::Viewer::participant_id() const
{
  return _pimpl->sequence.back().participant;
}

//==============================================================================
rmf_utils::optional<ParticipantId> Negotiation::Table::Viewer::parent_id() const
{
  return _pimpl->parent_id;
}

//==============================================================================
auto Negotiation::Table::Viewer::sequence() const -> const VersionedKeySequence&
{
  return _pimpl->sequence;
}

//==============================================================================
bool Negotiation::Table::Viewer::defunct() const
{
  assert(_pimpl->defunct);
  return *_pimpl->defunct;
}

//==============================================================================
bool Negotiation::Table::Viewer::rejected() const
{
  return _pimpl->rejected;
}

//==============================================================================
bool Negotiation::Table::Viewer::forfeited() const
{
  return _pimpl->forfeited;
}

//==============================================================================
const Itinerary* Negotiation::Table::Viewer::submission() const
{
  if (_pimpl->itinerary)
    return &(*_pimpl->itinerary);

  return nullptr;
}

//==============================================================================
Negotiation::Table::Viewer::Viewer()
{
  // Do nothing
}

//==============================================================================
auto Negotiation::Table::viewer() const -> ViewerPtr
{
  if (_pimpl->cached_table_viewer)
    return _pimpl->cached_table_viewer;

  rmf_utils::optional<ParticipantId> parent_id;
  if (const auto p = parent())
    parent_id = p->participant();

  _pimpl->cached_table_viewer = std::make_shared<Viewer>(
    Viewer::Implementation::make(
      _pimpl->proposed_timeline,
      _pimpl->alternatives_timelines,
      _pimpl->alternatives,
      _pimpl->base_proposals,
      _pimpl->participant_query,
      _pimpl->schedule_viewer,
      parent_id,
      _pimpl->sequence,
      _pimpl->defunct.get(),
      _pimpl->rejected,
      _pimpl->forfeited,
      _pimpl->itinerary));

  return _pimpl->cached_table_viewer;
}

//==============================================================================
const Itinerary* Negotiation::Table::submission() const
{
  if (_pimpl->itinerary)
    return &(*_pimpl->itinerary);

  return nullptr;
}

//==============================================================================
Version Negotiation::Table::version() const
{
  return _pimpl->version();
}

//==============================================================================
auto Negotiation::Table::proposal() const -> const Proposal&
{
  return _pimpl->proposal;
}

//==============================================================================
ParticipantId Negotiation::Table::participant() const
{
  return _pimpl->participant;
}

//==============================================================================
const Negotiation::VersionedKeySequence& Negotiation::Table::sequence() const
{
  return _pimpl->sequence;
}

//==============================================================================
std::vector<ParticipantId> Negotiation::Table::unversioned_sequence() const
{
  std::vector<ParticipantId> output;
  output.reserve(_pimpl->sequence.size());
  for (const auto& key : _pimpl->sequence)
    output.push_back(key.participant);

  return output;
}

//==============================================================================
bool Negotiation::Table::submit(
  std::vector<Route> itinerary,
  const Version version)
{
  return _pimpl->submit(std::move(itinerary), version);
}

//==============================================================================
bool Negotiation::Table::reject(
  const Version version,
  ParticipantId rejected_by,
  Alternatives rollouts)
{
  return _pimpl->reject(version, rejected_by, std::move(rollouts));
}

//==============================================================================
bool Negotiation::Table::rejected() const
{
  return _pimpl->rejected;
}

//==============================================================================
void Negotiation::Table::forfeit(Version version)
{
  _pimpl->forfeit(version);
}

//==============================================================================
bool Negotiation::Table::forfeited() const
{
  return _pimpl->forfeited;
}

//==============================================================================
bool Negotiation::Table::defunct() const
{
  return _pimpl->defunct;
}

//==============================================================================
auto Negotiation::Table::respond(const ParticipantId by_participant) -> TablePtr
{
  const auto it = _pimpl->descendants.find(by_participant);
  if (it == _pimpl->descendants.end())
    return nullptr;

  return it->second;
}

//==============================================================================
auto Negotiation::Table::respond(const ParticipantId by_participant) const
-> ConstTablePtr
{
  return const_cast<Table&>(*this).respond(by_participant);
}

//==============================================================================
auto Negotiation::Table::parent() -> TablePtr
{
  return _pimpl->weak_parent.lock();
}

//==============================================================================
auto Negotiation::Table::parent() const -> ConstTablePtr
{
  return _pimpl->weak_parent.lock();
}

//==============================================================================
auto Negotiation::Table::children() -> std::vector<TablePtr>
{
  std::vector<TablePtr> children_;
  for (const auto& c : _pimpl->descendants)
    children_.push_back(c.second);
  return children_;
}

//==============================================================================
auto Negotiation::Table::children() const -> std::vector<ConstTablePtr>
{
  std::vector<ConstTablePtr> children_;
  for (const auto& c : _pimpl->descendants)
    children_.push_back(c.second);
  return children_;
}

//==============================================================================
bool Negotiation::Table::ongoing() const
{
  return static_cast<bool>(_pimpl->weak_negotiation_data.lock());
}

//==============================================================================
Negotiation::Table::Table()
{
  // Do nothing
}

//==============================================================================
auto Negotiation::table(
  const ParticipantId for_participant,
  const std::vector<ParticipantId>& to_accommodate) -> TablePtr
{
  return _pimpl->get_entry(for_participant, to_accommodate);
}

//==============================================================================
auto Negotiation::table(
  const ParticipantId for_participant,
  const std::vector<ParticipantId>& to_accommodate) const -> ConstTablePtr
{
  return _pimpl->get_entry(for_participant, to_accommodate);
}

//==============================================================================
auto Negotiation::table(
  const std::vector<ParticipantId>& sequence) -> TablePtr
{
  return _pimpl->get_entry(sequence);
}

//==============================================================================
auto Negotiation::table(
  const std::vector<ParticipantId>& sequence) const -> ConstTablePtr
{
  return _pimpl->get_entry(sequence);
}

//==============================================================================
auto Negotiation::find(
  const ParticipantId for_participant,
  const VersionedKeySequence& to_accommodate) -> SearchResult<TablePtr>
{
  return _pimpl->find_entry(for_participant, to_accommodate);
}

//==============================================================================
auto Negotiation::find(
  const ParticipantId for_participant,
  const VersionedKeySequence& to_accommodate) const
-> SearchResult<ConstTablePtr>
{
  return _pimpl->find_entry(for_participant, to_accommodate);
}

//==============================================================================
auto Negotiation::find(const VersionedKeySequence& sequence)
-> SearchResult<TablePtr>
{
  return _pimpl->find_entry(sequence);
}

//==============================================================================
auto Negotiation::find(const VersionedKeySequence& sequence) const
-> SearchResult<ConstTablePtr>
{
  return _pimpl->find_entry(sequence);
}

//==============================================================================
auto Negotiation::evaluate(const Evaluator& evaluator) const -> ConstTablePtr
{
  const auto& successes = _pimpl->data->successful_tables;
  if (successes.empty())
    return nullptr;

  std::vector<const Proposal*> proposals;
  std::vector<ConstTablePtr> tables;
  proposals.reserve(successes.size());
  for (const auto& s : successes)
  {
    auto table_ptr = _pimpl->find_entry(s).table;
    assert(table_ptr);

    const auto& proposal = Table::Implementation::get(*table_ptr).proposal;
    assert(Table::Implementation::get(*table_ptr).itinerary);
    assert(!Table::Implementation::get(*table_ptr).rejected);
    assert(proposal.size() == Table::Implementation::get(*table_ptr).depth);
    assert(Table::Implementation::get(*table_ptr).descendants.empty());

    tables.emplace_back(std::move(table_ptr));
    proposals.emplace_back(&proposal);
  }

  const std::size_t choice = evaluator.choose(proposals);
  assert(choice < tables.size());

  return tables[choice];
}

//==============================================================================
Negotiation::Negotiation()
{
  // Do nothing
}

namespace {
//==============================================================================
rmf_utils::optional<Time> get_finish_time(const Itinerary& itinerary)
{
  rmf_utils::optional<Time> finish_time;
  for (const auto& route : itinerary)
  {
    const auto* t = route->trajectory().finish_time();
    if (!t)
      continue;

    if (!finish_time)
      finish_time = *route->trajectory().finish_time();
    else
    {
      if (*t < *finish_time)
        finish_time = *t;
    }
  }

  return finish_time;
}
} // anonymous namespace

//==============================================================================
std::size_t QuickestFinishEvaluator::choose(
  const std::vector<const Negotiation::Proposal*>& proposals) const
{
  std::unordered_map<ParticipantId, Time> best_finish_times;

  std::vector<std::unordered_map<ParticipantId, Time>> all_finish_times;
  all_finish_times.reserve(proposals.size());

  for (const auto& proposal : proposals)
  {
    all_finish_times.push_back({});
    auto& finish_times = all_finish_times.back();

    for (const auto& p : *proposal)
    {
      auto finish_time = get_finish_time(p.itinerary);
      if (!finish_time)
        continue;

      finish_times[p.participant] = *finish_time;

      const auto insertion = best_finish_times.insert(
        std::make_pair(p.participant, *finish_time));

      if (insertion.second)
      {
        // The insertion took place, so there's no need to compare to the
        // previous best
        continue;
      }

      if (*finish_time < insertion.first->second)
        insertion.first->second = *finish_time;
    }
  }

  assert(all_finish_times.size() == proposals.size());

  double best_penalty = std::numeric_limits<double>::infinity();
  std::size_t best_index = std::numeric_limits<std::size_t>::max();
  for (std::size_t i = 0; i < proposals.size(); ++i)
  {
    const auto& finish_times = all_finish_times[i];
    double penalty = 0;
    for (const auto& t : finish_times)
    {
      const ParticipantId participant = t.first;
      const Time time = t.second;
      const Time best_finish_time = best_finish_times.at(participant);
      assert(best_finish_time <= time);

      penalty += time::to_seconds(time - best_finish_time);
    }

    if (penalty < best_penalty)
    {
      best_penalty = penalty;
      best_index = i;
    }
  }

  assert(best_index < proposals.size());
  return best_index;
}

} // namespace schedule
} // namespace rmf_traffic
