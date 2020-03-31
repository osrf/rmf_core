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

#include "Modular.hpp"
#include "Timeline.hpp"
#include "ViewerInternal.hpp"

namespace rmf_traffic {
namespace schedule {
namespace {

//==============================================================================
std::size_t factorial(std::size_t N)
{
  std::size_t output = 1;
  while(N > 1)
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
  std::vector<std::vector<ParticipantId>> successful_tables;

  /// The number of negotiation tables that have reached a conclusion (either
  /// successfully finished or rejected)
  std::size_t num_terminated_tables = 0;

  std::unordered_set<Negotiation::Table::Implementation*> rejected_tables;
};

} // anonymous namespace

//==============================================================================
class Negotiation::Table::Implementation
{
public:

  using TableMap = std::unordered_map<ParticipantId, std::shared_ptr<Table>>;

  struct RouteEntry
  {
    ConstRoutePtr route;
    ParticipantId participant;
    RouteId route_id;
    std::shared_ptr<const ParticipantDescription> description;
    std::shared_ptr<void> timeline_handle;
  };
  using RouteEntryPtr = std::shared_ptr<RouteEntry>;

  const Viewer* const viewer;
  std::vector<ParticipantId> sequence;
  std::vector<ParticipantId> unsubmitted;
  std::vector<RouteEntryPtr> entries;
  Timeline<RouteEntry> timeline;

  Query::Participants participant_query;

  Proposal proposal;

  const ParticipantId participant;
  const std::size_t depth;
  rmf_utils::optional<Itinerary> itinerary;
  Version version = std::numeric_limits<Version>::max();
  bool rejected = false;
  TableMap descendants;

  std::weak_ptr<NegotiationData> weak_negotiation_data;
  std::weak_ptr<Table> weak_owner;
  std::weak_ptr<Table> weak_parent;

  Implementation(
      TablePtr owner_,
      std::shared_ptr<NegotiationData> negotiation_data_,
      const Viewer& viewer_,
      ParticipantId participant_,
      std::size_t depth_,
      std::vector<ParticipantId> submitted_,
      std::vector<ParticipantId> unsubmitted_,
      Proposal initial_proposal_,
      TablePtr parent_)
    : viewer(&viewer_),
      unsubmitted(std::move(unsubmitted_)),
      proposal(std::move(initial_proposal_)),
      participant(participant_),
      depth(depth_),
      weak_negotiation_data(negotiation_data_),
      weak_owner(owner_),
      weak_parent(std::move(parent_))
  {
    for (const auto& p : proposal)
    {
      const ParticipantId participant = p.participant;
      const auto& description = viewer->get_participant(participant);
      for (std::size_t i=0; i < p.itinerary.size(); ++i)
      {
        const auto& route = p.itinerary[i];

        auto entry = std::make_unique<RouteEntry>(
              RouteEntry{
                route,
                participant,
                i,
                description,
                nullptr
              });

        timeline.insert(*entry);
        entries.push_back(std::move(entry));
      }
    }

    std::vector<ParticipantId> all_participants = submitted_;
    all_participants.insert(
          all_participants.end(), unsubmitted.begin(), unsubmitted.end());
    participant_query = Query::Participants::make_all_except(all_participants);

    // Add this Table's participant to the sequence
    sequence = std::move(submitted_);
    sequence.push_back(participant);

    // Remove this Table's participant from the unsubmitted
    const auto it = std::remove_if(unsubmitted.begin(), unsubmitted.end(),
                   [&](const ParticipantId p) { return p == participant; });
    assert(it != unsubmitted.end());
    unsubmitted.erase(it);
  }

  Viewer::View query(const Query::Spacetime& spacetime) const;

  void add_participant(const ParticipantId new_participant)
  {
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

    std::unordered_map<ParticipantId, Table> descendants;
    for (const auto u : unsubmitted)
      make_descendent(u);
  }

  void make_descendent(const ParticipantId p)
  {
    auto table = std::make_shared<Table>(Table());
    table->_pimpl = rmf_utils::make_unique_impl<Implementation>(
          table, weak_negotiation_data.lock(), *viewer, p, depth+1,
          sequence, unsubmitted, proposal, weak_owner.lock());

    descendants.insert(std::make_pair(p, std::move(table)));
  }

  static TablePtr make_root(
      const Viewer& viewer,
      const std::shared_ptr<NegotiationData> negotiation_data,
      const ParticipantId participant,
      const std::vector<ParticipantId>& participants)
  {
    auto table = std::make_shared<Table>(Table());
    table->_pimpl = rmf_utils::make_unique_impl<Implementation>(
          Implementation(
            table, negotiation_data, viewer, participant, 1, {},
            participants, {}, nullptr));

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
    if (!modular(version).less_than(new_version))
      return false;

    bool formerly_successful = false;
    const auto negotiation_data = weak_negotiation_data.lock();
    if (rejected && negotiation_data)
    {
      negotiation_data->rejected_tables.erase(this);
      negotiation_data->num_terminated_tables -=
          termination_factor(depth, negotiation_data->participants.size());
    }
    else if (itinerary && descendants.empty())
    {
      // This means that this was a successful terminating node, so we should
      // make note of that to keep our bookkeeping correct.
      formerly_successful = true;
    }

    const bool had_itinerary = itinerary.has_value();

    version = new_version;
    itinerary = convert_itinerary(new_itinerary);
    rejected = false;

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

  void reject()
  {
    if (rejected)
      return;

    const auto negotiation_data = weak_negotiation_data.lock();
    if (itinerary && descendants.empty() && negotiation_data)
    {
      // This used to be a successfully completed negotiation table.
      // TODO(MXG): It's a bit suspicious that a successfully completed
      // negotiation table would get rejected. Maybe we should put an
      // assertion here.
      negotiation_data->num_terminated_tables -= 1;
    }

    rejected = true;
    itinerary = rmf_utils::nullopt;
    clear_descendants();

    if (negotiation_data)
    {
      negotiation_data->num_terminated_tables += termination_factor(
            depth, negotiation_data->participants.size());
      negotiation_data->rejected_tables.insert(this);

      // Erase any successful tables that branched off of this rejected table
      const auto erase_it = std::remove_if(
            negotiation_data->successful_tables.begin(),
            negotiation_data->successful_tables.end(),
            [&](const std::vector<ParticipantId>& table)
      {
        for (std::size_t i=0; i < sequence.size(); ++i)
        {
          if (table[i] != sequence[i])
            return false;
        }

        return true;
      });

      negotiation_data->successful_tables.erase(
            erase_it, negotiation_data->successful_tables.end());
    }
  }

  // This function removes descendent tables and makes sure that they can no
  // longer impact the negotiation
  void clear_descendants()
  {
    // TODO(MXG): Instead of clearing all the descendants during a submission,
    // we could simply clear the ones whose proposals are in conflict with the
    // new submission. However, all descendants must always be cleared during a
    // rejection.

    const auto negotiation_data = weak_negotiation_data.lock();

    std::vector<Table::Implementation*> queue;
    queue.push_back(this);
    while (!queue.empty())
    {
      auto top = queue.back();
      queue.pop_back();

      for (const auto entry : top->descendants)
      {
        const auto& table = entry.second;
        if (table->_pimpl->rejected && negotiation_data)
        {
          negotiation_data->num_terminated_tables -=
              termination_factor(
                table->_pimpl->depth, negotiation_data->participants.size());

          negotiation_data->rejected_tables.erase(table->_pimpl.get());
        }

        table->_pimpl->weak_negotiation_data.reset();
        table->_pimpl->weak_parent.reset();
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
      const Viewer& viewer_,
      std::vector<ParticipantId> participants_)
    : viewer(&viewer_),
      data(std::make_shared<NegotiationData>())
  {
    for (const auto p : participants_)
      data->participants.insert(p);

    max_terminated_tables = factorial(data->participants.size());

    for (const auto p : participants_)
    {
      tables[p] = Table::Implementation::make_root(
            viewer_, data, p, participants_);
    }
  }

  const Viewer* viewer;
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

  void add_participant(const ParticipantId new_participant)
  {
    if (!data->participants.insert(new_participant).second)
    {
      throw std::runtime_error(
          "[rmf_traffic::schedule::Negotiation::add_participant] "
          "Participant [" + std::to_string(new_participant) + "] is already "
          "present in the Negotiation");
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
    for (const auto rejected : data->rejected_tables)
      data->num_terminated_tables += termination_factor(rejected->depth, N);

    std::vector<TableMap*> queue;
    queue.push_back(&tables);
    while (!queue.empty())
    {
      auto* next = queue.back();
      queue.pop_back();

      for (auto& element : *next)
      {
        const auto& entry = element.second;
        Table::Implementation::get(*entry).add_participant(new_participant);

        queue.push_back(&Table::Implementation::get(*entry).descendants);
      }
    }

    tables[new_participant] = Table::Implementation::make_root(
          *viewer, data, new_participant,
          std::vector<ParticipantId>(
            data->participants.begin(),
            data->participants.end()));
  }
};

//==============================================================================
Negotiation::Negotiation(
    const Viewer& viewer,
    std::vector<ParticipantId> participants)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(
             viewer, std::move(participants)))
{
  // Do nothing
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
  return _pimpl->data->num_terminated_tables == _pimpl->max_terminated_tables;
}

namespace {
//==============================================================================
class NegotiationRelevanceInspector
    : public TimelineInspector<Negotiation::Table::Implementation::RouteEntry>
{
public:

  using RouteEntry = Negotiation::Table::Implementation::RouteEntry;

  using Storage = Viewer::View::Implementation::Storage;

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
Viewer::View Negotiation::Table::Implementation::query(
    const Query::Spacetime &spacetime) const
{
  Query query = query_all();
  query.spacetime() = spacetime;

  // Query for the relevant routes that are being negotiated
  NegotiationRelevanceInspector inspector;
  timeline.inspect(query, inspector);

  // Query for the relevant routes that are outside of the negotiation
  query.participants() = participant_query;
  Viewer::View view = viewer->query(query);

  // Merge them together into a single view
  Viewer::View::Implementation::append_to_view(
        view, std::move(inspector.routes));

  return view;
}

//==============================================================================
Viewer::View Negotiation::Table::query(const Query::Spacetime& parameters) const
{
  return _pimpl->query(parameters);
}

//==============================================================================
const Itinerary* Negotiation::Table::submission() const
{
  if (_pimpl->itinerary)
    return &(*_pimpl->itinerary);

  return nullptr;
}

//==============================================================================
const Version* Negotiation::Table::version() const
{
  if (_pimpl->itinerary)
    return &_pimpl->version;

  return nullptr;
}

//==============================================================================
auto Negotiation::Table::proposal() const -> Proposal
{
  return _pimpl->proposal;
}

//==============================================================================
ParticipantId Negotiation::Table::participant() const
{
  return _pimpl->participant;
}

//==============================================================================
const std::vector<ParticipantId>& Negotiation::Table::sequence() const
{
  return _pimpl->sequence;
}

//==============================================================================
bool Negotiation::Table::submit(
    std::vector<Route> itinerary,
    const Version version)
{
  return _pimpl->submit(std::move(itinerary), version);
}

//==============================================================================
void Negotiation::Table::reject()
{
  _pimpl->reject();
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
    auto table_ptr = _pimpl->get_entry(s);
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
  for (std::size_t i=0; i < proposals.size(); ++i)
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
