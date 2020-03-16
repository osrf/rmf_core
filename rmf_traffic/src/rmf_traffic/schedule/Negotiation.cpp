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
std::string to_string(const std::vector<ParticipantId>& participants)
{
  std::string output;
  for (const auto p : participants)
    output += " " + std::to_string(p) + " ";

  return output;
}

} // anonymous namespace

//==============================================================================
class Negotiation::Table::Implementation
{
public:

  struct RouteEntry
  {
    ConstRoutePtr route;
    ParticipantId participant;
    RouteId route_id;
    std::shared_ptr<const ParticipantDescription> description;
    std::shared_ptr<void> timeline_handle;
  };
  using RouteEntryPtr = std::shared_ptr<RouteEntry>;

  const Viewer* viewer;
  std::vector<ParticipantId> submitted;
  std::vector<ParticipantId> unsubmitted;
  std::vector<RouteEntryPtr> entries;
  Timeline<RouteEntry> timeline;

  Query::Participants participant_query;

  Proposal proposal;

  Implementation(
      const Viewer& viewer_,
      std::vector<ParticipantId> submitted_,
      std::vector<ParticipantId> unsubmitted_,
      Proposal proposal_)
    : viewer(&viewer_),
      submitted(std::move(submitted_)),
      unsubmitted(std::move(unsubmitted_)),
      proposal(std::move(proposal_))
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

    std::vector<ParticipantId> all_participants = submitted;
    all_participants.insert(
          all_participants.end(), unsubmitted.begin(), unsubmitted.end());
    participant_query = Query::Participants::make_all_except(all_participants);
  }

  Viewer::View query(const Query::Spacetime& spacetime) const;

  static void add_participant(Table& table, ParticipantId new_participant)
  {
    table._pimpl->unsubmitted.push_back(new_participant);
  }

  static std::unordered_map<ParticipantId, Table> make_descendants(
      const Table& from,
      const ParticipantId submitting,
      const Itinerary& itinerary)
  {
    Proposal proposal = from._pimpl->proposal;
    proposal.push_back({submitting, itinerary});

    std::vector<ParticipantId> submitted = from._pimpl->submitted;
    submitted.push_back(submitting);

    std::vector<ParticipantId> unsubmitted = from._pimpl->unsubmitted;
    const auto it = std::remove_if(unsubmitted.begin(), unsubmitted.end(),
                   [&](const ParticipantId p) { return p == submitting; });
    assert(it != unsubmitted.end());
    unsubmitted.erase(it);

    std::unordered_map<ParticipantId, Table> descendants;
    for (const auto u : unsubmitted)
    {
      Table table;
      table._pimpl = rmf_utils::make_unique_impl<Implementation>(
            *from._pimpl->viewer,
            submitted,
            unsubmitted,
            proposal);

      descendants.insert(std::make_pair(u, std::move(table)));
    }

    return descendants;
  }

  static Table make_root(
      const Viewer& viewer,
      const std::vector<ParticipantId>& participants)
  {
    Table table;
    table._pimpl = rmf_utils::make_unique_impl<Implementation>(
          Implementation(viewer, {}, participants, {}));

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
};

//==============================================================================
class Negotiation::Implementation
{
public:

  Implementation(
      const Viewer& viewer_,
      std::vector<ParticipantId> participants_)
    : viewer(&viewer_)
  {
    for (const auto p : participants_)
      participants.insert(p);

    max_terminated_tables = factorial(participants.size());

    for (const auto p : participants_)
    {
      tables[p] = std::make_unique<TableEntry>(
            Table::Implementation::make_root(viewer_, participants_), p, 1);
    }
  }

  const Viewer* viewer;
  std::unordered_set<ParticipantId> participants;
  std::size_t max_terminated_tables;

  struct TableEntry;
  using TableEntryPtr = std::unique_ptr<TableEntry>;
  using TableMap = std::unordered_map<ParticipantId, TableEntryPtr>;
  TableMap tables;

  /// The negotiation tables that have successfully reached a termination
  std::vector<std::vector<ParticipantId>> successful_tables;

  /// The number of negotiation tables that have reached a conclusion (either
  /// successfully finished or rejected)
  std::size_t num_terminated_tables = 0;

  std::vector<std::size_t> rejected_table_depths;

  struct TableEntry
  {
    std::shared_ptr<Table> table;
    ParticipantId participant;
    std::size_t depth;
    rmf_utils::optional<Itinerary> itinerary;
    TableMap descendants;
    bool rejected = false;

    TableEntry(Table table_, ParticipantId participant_, std::size_t depth_)
      : table(std::make_shared<Table>(std::move(table_))),
        participant(participant_),
        depth(depth_)
    {
      // Do nothing
    }

    void submit(
        Implementation& impl,
        const ParticipantId for_participant,
        std::vector<Route> new_itinerary)
    {
      bool formerly_successful = false;
      if (rejected)
      {
        auto& rejections = impl.rejected_table_depths;
        const auto it =
            std::find(rejections.begin(), rejections.end(), depth);
        assert(it != rejections.end());

        rejections.erase(it);
        impl.num_terminated_tables -= depth;
      }
      else if (itinerary && descendants.empty())
      {
        // This means that this was a successful terminating node, so we should
        // make note of that to keep our bookkeeping correct.
        formerly_successful = true;
      }

      itinerary = convert_itinerary(std::move(new_itinerary));
      auto new_tables = Table::Implementation::make_descendants(
            *table, for_participant, *itinerary);
      if (new_tables.empty() && !formerly_successful)
      {
        // If there are no new tables that branch off of this submission, then
        // this submission has successfully terminated this branch of
        // negotiation
        auto sequence = Table::Implementation::get(*table).submitted;
        sequence.push_back(for_participant);
        impl.successful_tables.push_back(std::move(sequence));
        impl.num_terminated_tables += 1;
      }

      for (auto& t : new_tables)
      {
        auto new_entry =
            std::make_unique<TableEntry>(std::move(t.second), t.first, depth+1);

        descendants.insert(std::make_pair(t.first, std::move(new_entry)));
      }
    }

    void reject(Implementation& impl)
    {
      if (rejected)
        return;

      if (itinerary && descendants.empty())
      {
        // This used to be a successfully completed negotiation table.
        // TODO(MXG): It's a bit suspicious that a successfully completed
        // negotiation table would get rejected. Maybe we should put an
        // assertion here.
        impl.num_terminated_tables -= 1;
      }

      rejected = true;
      itinerary = rmf_utils::nullopt;
      descendants.clear();

      impl.num_terminated_tables += termination_factor(
            depth, impl.participants.size());

      // Erase any successful tables that branched off of this rejected table
      const auto& submitted = Table::Implementation::get(*table).submitted;
      const auto erase_it = std::remove_if(
            impl.successful_tables.begin(),
            impl.successful_tables.end(),
            [&](const std::vector<ParticipantId>& table)
      {
        for (std::size_t i=0; i < submitted.size(); ++i)
        {
          if (table[i] != submitted[i])
            return false;
        }

        return true;
      });

      impl.successful_tables.erase(erase_it, impl.successful_tables.end());
    }
  };

  TableEntry* climb(const TableMap& map, const ParticipantId p)
  {
    const auto it = map.find(p);
    if (it == map.end())
      return nullptr;

    return it->second.get();
  }

  TableEntry* get_entry(
      const std::vector<ParticipantId>& table)
  {
    TableEntry* output = nullptr;
    const TableMap* map = &tables;
    for (const auto p : table)
    {
      output = climb(*map, p);
      if (!output)
        return nullptr;

      map = &output->descendants;
    }

    return output;
  }

  const TableEntry* get_entry(
      const std::vector<ParticipantId>& table) const
  {
    return const_cast<Implementation&>(*this).get_entry(table);
  }

  TableEntry* get_entry(
      const ParticipantId for_participant,
      const std::vector<ParticipantId>& to_accommodate)
  {
    const TableMap* map = nullptr;
    if (to_accommodate.empty())
    {
      map = &tables;
    }
    else
    {
      const auto* output = get_entry(to_accommodate);
      if (!output)
        return nullptr;

      map = &output->descendants;
    }
    assert(map);

    return climb(*map, for_participant);
  }

  const TableEntry* get_entry(
      const ParticipantId for_participant,
      const std::vector<ParticipantId>& to_accommodate) const
  {
    return const_cast<Implementation&>(*this).get_entry(
          for_participant, to_accommodate);
  }

  void add_participant(const ParticipantId new_participant)
  {
    if (!participants.insert(new_participant).second)
    {
      throw std::runtime_error(
          "[rmf_traffic::schedule::Negotiation::add_participant] "
          "Participant [" + std::to_string(new_participant) + "] is already "
          "present in the Negotiation");
    }

    participants.insert(new_participant);

    // We can update the maximum number of terminating tables by just
    // multiplying by the new size of negotiation participants, because the
    // value should be the factorial of the total number of negotiation
    // participants.
    max_terminated_tables *= participants.size();

    // With a new participant, none of the successfully terminated negotiations
    // are valid anymore.
    successful_tables.clear();

    // Restart the counter to zero.
    num_terminated_tables = 0;

    // The rejected tables are still terminated, but the number of tables that
    // are terminated due to rejection will be higher now, so we need to
    // recalculate it.
    const std::size_t N = participants.size();
    for (const auto depth : rejected_table_depths)
      num_terminated_tables += termination_factor(depth, N);

    std::vector<TableMap*> queue;
    queue.push_back(&tables);
    while (!queue.empty())
    {
      auto* next = queue.back();
      queue.pop_back();

      for (auto& element : *next)
      {
        const auto& entry = element.second;
        Table::Implementation::add_participant(*entry->table, new_participant);

        queue.push_back(&entry->descendants);
      }
    }
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
void Negotiation::add_participant(ParticipantId p)
{
  _pimpl->add_participant(p);
}

//==============================================================================
void Negotiation::submit(
    const ParticipantId for_participant,
    const std::vector<ParticipantId>& to_accommodate,
    std::vector<Route> itinerary)
{
  auto* entry = _pimpl->get_entry(for_participant, to_accommodate);
  if (!entry)
  {
    throw std::runtime_error(
          "[rmf_traffic::schedule::Negotiation::submit] Cannot set proposal "
          "for [" + std::to_string(for_participant) + "] to accommodate ["
          + to_string(to_accommodate)
          + "] because that table does not exist yet");
  }

  entry->submit(*_pimpl, for_participant, std::move(itinerary));
}

//==============================================================================
void Negotiation::reject(const std::vector<ParticipantId>& table)
{
  assert(!table.empty());
  auto* entry = _pimpl->get_entry(table);
  if (!entry)
  {
    throw std::runtime_error(
          "[rmf_traffic::schedule::Negotiation::reject] Cannot reject proposal "
          "[" + to_string(table) + "] because that table does not exist yet.");
  }

  entry->reject(*_pimpl);
}

//==============================================================================
bool Negotiation::ready() const
{
  return !_pimpl->successful_tables.empty();
}

//==============================================================================
bool Negotiation::complete() const
{
  return _pimpl->num_terminated_tables == _pimpl->max_terminated_tables;
}

//==============================================================================
rmf_utils::optional<Negotiation::Proposal> Negotiation::evaluate(
    const Evaluator& evaluator) const
{
  if (_pimpl->successful_tables.empty())
    return rmf_utils::nullopt;

  std::vector<Proposal> proposals;
  for (const auto& p : _pimpl->successful_tables)
  {
    const auto* entry = _pimpl->get_entry(p);
    assert(entry);
    assert(entry->itinerary);
    assert(!entry->rejected);
    assert(entry->descendants.empty());

    Proposal proposal = Table::Implementation::get(*entry->table).proposal;
    proposal.push_back({entry->participant, *entry->itinerary});
    proposals.emplace_back(std::move(proposal));
  }

  const std::size_t choice = evaluator.choose(proposals);
  assert(choice < proposals.size());

  return std::move(proposals[choice]);
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
Negotiation::Table::Table()
{
  // Do nothing
}

//==============================================================================
auto Negotiation::table(
    const ParticipantId for_participant,
    const std::vector<ParticipantId>& to_accommodate) const
-> std::shared_ptr<const Table>
{
  if (const auto entry = _pimpl->get_entry(for_participant, to_accommodate))
    return entry->table;

  return nullptr;
}

//==============================================================================
namespace {
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
    const std::vector<Negotiation::Proposal>& proposals) const
{
  std::unordered_map<ParticipantId, Time> best_finish_times;

  std::vector<std::unordered_map<ParticipantId, Time>> all_finish_times;
  all_finish_times.reserve(proposals.size());

  for (const auto& proposal : proposals)
  {
    all_finish_times.push_back({});
    auto& finish_times = all_finish_times.back();

    for (const auto& p : proposal)
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
