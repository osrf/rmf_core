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

#include "conflicts.hpp"
#include "geometry.hpp"

#include <set>
#include <map>


#include <iostream>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
bool Bracket::operator==(const Bracket& other) const
{
  return
         start == other.start
      && finish == other.finish
      && include_start == other.include_start
      && include_finish == other.include_finish;
}

//==============================================================================
bool BracketPair::operator==(const BracketPair& other) const
{
  return A == other.A && B == other.B;
}

//==============================================================================
BracketPair BracketPair::complement() const
{
  return BracketPair{B, A};
}

//==============================================================================
std::shared_ptr<Timeline> Timeline::make(
    const std::vector<AlignedBracketPair>& alignments)
{
  return std::shared_ptr<Timeline>(new Timeline(alignments));
}

//==============================================================================
bool Timeline::is_behind(
    const ReservedRange& range_A,
    const ReservedRange& range_B) const
{
  const std::size_t a = range_A.end;
  const std::size_t b = range_B.begin;

  const auto it = _map.lower_bound(a);
  if (it == _map.end())
    return false;

  return it->second.index <= b;
}

//==============================================================================
Timeline::Timeline(const std::vector<AlignedBracketPair>& alignments)
{
  for (const auto& pair : alignments)
  {
    if (pair.A.include_start && pair.B.include_start)
    {
      // This is always highest priority
      _map[pair.B.start] = Comparison{Comparison::EqualTo, pair.A.start};
    }
    else if (pair.A.include_start)
    {
      _insert_if_preferable(
            pair.B.start, Comparison{Comparison::LessThan, pair.A.start});
    }

    if (pair.A.include_finish && pair.B.include_finish)
    {
      // This is always highest priority
      _map[pair.B.finish] = Comparison{Comparison::EqualTo, pair.A.finish};
    }
    else if (pair.B.include_finish)
    {
      _insert_if_preferable(
            pair.B.finish, Comparison{Comparison::LessThan, pair.A.finish});
    }
  }
}

//==============================================================================
void Timeline::_insert_if_preferable(
    const std::size_t index,
    const Comparison comp)
{
  const auto insertion = _map.insert({index, comp});
  if (!insertion.second)
  {
    const auto it = insertion.first;
    if (it->second.type == Comparison::EqualTo)
      return;

    if (comp.index < it->second.index)
      it->second = comp;
  }
}

//==============================================================================
class BehindConstraint : public Constraint
{
public:

  BehindConstraint(
      std::size_t is_behind,
      std::size_t is_in_front,
      std::shared_ptr<Timeline> timeline)
    : _is_behind(is_behind),
      _is_in_front(is_in_front),
      _timeline(std::move(timeline))
  {
    _dependencies.insert(is_behind);
    _dependencies.insert(is_in_front);
  }

  bool evaluate(const State& state) const final
  {
    return _evaluate(
          get_range(state, _is_behind),
          get_range(state, _is_in_front));
  }

  const std::unordered_set<std::size_t>& dependencies() const final
  {
    return _dependencies;
  }

  std::optional<bool> partial_evaluate(const State& state) const final
  {
    const auto is_behind_it = state.find(_is_behind);
    if (is_behind_it == state.end())
      return std::nullopt;

    const auto is_in_front_it = state.find(_is_in_front);
    if (is_in_front_it == state.end())
      return std::nullopt;

    return _evaluate(is_behind_it->second, is_in_front_it->second);
  }

private:

  const ReservedRange& get_range(
      const State& state,
      const std::size_t participant) const
  {
    const auto it = state.find(participant);
    if (it != state.end())
      return it->second;

    std::string error = "Failed to evalute BehindConstraint comparing "
        + std::to_string(_is_behind) + " to "
        + std::to_string(_is_in_front) + ". Participant "
        + std::to_string(participant) + " is missing from the state.";
    throw std::runtime_error(error);
  }

  bool _evaluate(
      const ReservedRange& should_be_behind,
      const ReservedRange& should_be_in_front) const
  {
    const bool result =
        _timeline->is_behind(
          should_be_behind,
          should_be_in_front);
    std::cout << "Checking if [" << _is_behind << ":" << should_be_behind.begin
              << "->" << should_be_behind.end
              << "] is behind [" << _is_in_front << ":" << should_be_in_front.begin
              << "->" << should_be_in_front.end
              << "]: " << result << std::endl;
    return result;
  }

  std::size_t _is_behind;
  std::size_t _is_in_front;
  std::shared_ptr<Timeline> _timeline;
  std::unordered_set<std::size_t> _dependencies;
};

//==============================================================================
std::shared_ptr<Constraint> behind(
    std::size_t is_behind,
    std::size_t is_in_front,
    std::shared_ptr<Timeline> timeline)
{
  return std::make_shared<BehindConstraint>(
        is_behind, is_in_front, std::move(timeline));
}

//==============================================================================
bool compatible_start_and_finish(
    std::size_t start,
    bool include_start,
    std::size_t finish,
    bool include_finish)
{
  if (finish+1 < start)
    return false;

  if (start == finish)
  {
    if (!include_start && !include_finish)
      return false;
  }

  if (finish+1 == start)
  {
    if (!include_start || !include_finish)
      return false;
  }

  return true;
}

//==============================================================================
bool can_merge_brackets(
    const Bracket& bracket0,
    const Bracket& bracket1)
{
  if (!compatible_start_and_finish(
        bracket0.start, bracket0.include_start,
        bracket1.finish, bracket1.include_finish))
    return false;

  if (!compatible_start_and_finish(
        bracket1.start, bracket1.include_start,
        bracket0.finish, bracket0.include_finish))
    return false;

  return true;
}

//==============================================================================
Bracket merge_brackets(
    const Bracket& bracket0,
    const Bracket& bracket1)
{
  Bracket output;

  if (bracket0.start == bracket1.start)
  {
    output.start = bracket0.start;
    output.include_start = bracket0.include_start || bracket1.include_start;
  }
  else if (bracket0.start < bracket1.start)
  {
    output.start = bracket0.start;
    output.include_start = bracket0.include_start;
  }
  else
  {
    output.start = bracket1.start;
    output.include_start = bracket1.include_start;
  }

  if (bracket0.finish == bracket1.finish)
  {
    output.finish = bracket0.finish;
    output.include_finish = bracket0.include_finish || bracket1.include_finish;
  }
  else if (bracket1.finish < bracket0.finish)
  {
    output.finish = bracket0.finish;
    output.include_finish = bracket0.include_finish;
  }
  else
  {
    output.finish = bracket1.finish;
    output.include_finish = bracket1.include_finish;
  }

  return output;
}

//==============================================================================
bool can_merge_pair(
    const BracketPair& pair0,
    const BracketPair& pair1)
{
  return can_merge_brackets(pair0.A, pair1.A)
      && can_merge_brackets(pair0.B, pair1.B);
}

//==============================================================================
bool try_merge(
    BracketPair& pair0,
    const BracketPair& pair1,
    std::size_t& merge_count)
{
  if (!can_merge_pair(pair0, pair1))
    return false;

  pair0.A = merge_brackets(pair0.A, pair1.A);
  pair0.B = merge_brackets(pair0.B, pair1.B);
  ++merge_count;
  return true;
}

//==============================================================================
void expand_bracket(
    Bracket& bracket,
    const std::vector<Writer::Checkpoint>& path)
{
  // This function accounts for points that the robot is not allowed to hold at.
  // These points get absorbed into the brackets as if they are part of the
  // conflicts.

  while (bracket.start > 0)
  {
    const bool can_hold_at_start = path.at(bracket.start).can_hold;

    if (can_hold_at_start && !bracket.include_start)
      break;

    const bool can_hold_at_next = path.at(bracket.start-1).can_hold;
    if (can_hold_at_next)
    {
      bracket.include_start = true;
      break;
    }

    --bracket.start;
    bracket.include_start = false;
  }
}

//==============================================================================
Brackets compute_brackets(
    const std::vector<Writer::Checkpoint>& path_a,
    const double radius_a,
    const std::vector<Writer::Checkpoint>& path_b,
    const double radius_b,
    const double angle_threshold)
{
  std::multimap<std::size_t, AlignedBracketPair> aligned_set;
  std::multimap<std::size_t, ConflictBracketPair> conflict_set;
  for (std::size_t a=0; a < path_a.size()-1; ++a)
  {
    const auto& it_a_start = path_a[a];
    const auto& it_a_finish = path_a[a+1];

    if (it_a_start.map_name != it_a_finish.map_name)
      continue;

    const Segment segment_a{
      it_a_start.position, it_a_finish.position, radius_a};

    for (std::size_t b=0; b < path_b.size()-1; ++b)
    {
      const auto& it_b_start = path_b[b];
      const auto& it_b_finish = path_b[b+1];

      if (it_b_start.map_name != it_b_finish.map_name)
        continue;

      if (it_a_start.map_name != it_b_start.map_name)
        continue;

      const Segment segment_b{
        it_b_start.position, it_b_finish.position, radius_b};

      const auto info = detect_conflict(segment_a, segment_b, angle_threshold);

      if (info.is_nothing())
        continue;

      BracketPair pair;
      pair.A.start = a;
      pair.A.finish = a+1;
      pair.A.include_start = info.include_cap_a[ConflictInfo::Start];
      pair.A.include_finish = info.include_cap_a[ConflictInfo::Finish];

      pair.B.start = b;
      pair.B.finish = b+1;
      pair.B.include_start = info.include_cap_b[ConflictInfo::Start];
      pair.B.include_finish = info.include_cap_b[ConflictInfo::Finish];

      if (info.is_conflict())
      {
        expand_bracket(pair.A, path_a);
        expand_bracket(pair.B, path_b);

        conflict_set.emplace(
              std::make_pair(pair.A.start, ConflictBracketPair{pair}));
      }
      else if (info.is_alignment())
      {
        aligned_set.emplace(
              std::make_pair(pair.A.start, AlignedBracketPair{pair}));
      }
    }
  }

  std::vector<ConflictBracketPair> final_conflicts;
  while (!conflict_set.empty())
  {
    std::size_t merge_count = 0;

    auto it_a = conflict_set.begin();
    auto pair = it_a->second;
    conflict_set.erase(it_a++);

    while (it_a != conflict_set.end())
    {
      if (try_merge(pair, it_a->second, merge_count))
      {
        conflict_set.erase(it_a++);
      }
      else
      {
        ++it_a;
      }
    }

    if (merge_count == 0)
    {
      final_conflicts.push_back(pair);
    }
    else
    {
      conflict_set.insert(std::make_pair(pair.A.start, pair));
    }
  }

  std::vector<AlignedBracketSet> final_alignments;
  bool begin_new_alignment = true;
  while (!aligned_set.empty())
  {
    if (begin_new_alignment)
    {
      const auto initial_it = aligned_set.begin();
      final_alignments.push_back({initial_it->second, {initial_it->second}});
      aligned_set.erase(initial_it);
      begin_new_alignment = false;
    }

    std::size_t merge_count = 0;

    auto& current_set = final_alignments.back();
    auto& whole_bracket = current_set.whole_bracket;

    auto it_a = aligned_set.begin();
    while (it_a != aligned_set.end())
    {
      if (try_merge(whole_bracket, it_a->second, merge_count))
      {
        current_set.segments.push_back(it_a->second);
        aligned_set.erase(it_a++);
      }
      else
      {
        ++it_a;
      }
    }

    if (merge_count == 0)
      begin_new_alignment = true;
  }

  return Brackets{std::move(final_conflicts), std::move(final_alignments)};
}

//==============================================================================
std::pair<std::size_t, ConstConstraintPtr> compute_blocker(
    const Bracket& me,
    const std::size_t my_path_size,
    const Bracket& other,
    const std::size_t other_path_size,
    const std::size_t other_id)
{
  const std::size_t go_from = [&]() -> std::size_t
  {
    if (me.start == 0)
      return 0;

    if (me.include_start)
      return me.start-1;

    return me.start;
  }();

  const bool other_may_hold =
      (me.finish < my_path_size-1) || !me.include_finish;

  std::optional<std::size_t> blocker_hold_point;
  if (other_may_hold)
  {
    if (!other.include_start)
      blocker_hold_point = other.start;
    else if (other.start > 0)
      blocker_hold_point = other.start - 1;
    // else we do not allow the other participant to hold
  }

  std::optional<BlockageEndCondition> end_condition;
  if (other.include_finish)
  {
    if (other.finish < other_path_size-1)
    {
      end_condition = BlockageEndCondition{
          other.finish, BlockageEndCondition::HasPassed};
    }
  }
  else
  {
    end_condition = BlockageEndCondition{
        other.finish, BlockageEndCondition::HasReached};
  }

  return std::make_pair(
        go_from, blockage(other_id, blocker_hold_point, end_condition));
}

//==============================================================================
std::array<IndexToConstraint, 2> compute_blockers(
    const std::vector<ConflictBracketPair>& conflict_brackets,
    const std::size_t id_a,
    const std::size_t a_path_size,
    const std::size_t id_b,
    const std::size_t b_path_size)
{
  std::array<IndexToConstraint, 2> blockers;
  for (const auto& bracket : conflict_brackets)
  {
    blockers[0].insert(
      compute_blocker(bracket.A, a_path_size, bracket.B, b_path_size, id_b));

    blockers[1].insert(
      compute_blocker(bracket.B, b_path_size, bracket.A, a_path_size, id_a));
  }

  return blockers;
}

//==============================================================================
Alignment get_alignment(const std::vector<AlignedBracketPair>& alignments)
{
  Alignment output;
  output.timeline = Timeline::make(alignments);

  for (const auto& pair : alignments)
  {
    auto& caveat = output.index_to_caveats[pair.A.start];
    if (pair.B.include_start && !pair.A.include_start)
      caveat.push_back(pair.B.start);

    if (pair.B.include_finish)
      caveat.push_back(pair.B.finish);
  }

  return output;
}

//==============================================================================
std::vector<AlignedBracketPair> get_complement(
    const std::vector<AlignedBracketPair>& alignments)
{
  std::vector<AlignedBracketPair> output;
  output.reserve(alignments.size());
  for (const auto& a : alignments)
    output.push_back(AlignedBracketPair{a.complement()});

  return output;
}

//==============================================================================
std::array<std::vector<Alignment>, 2> compute_alignments(
    const std::vector<AlignedBracketSet>& alignments)
{
  std::array<std::vector<Alignment>, 2> output;
  for (const auto& alignment : alignments)
  {
    output.at(0).push_back(get_alignment(alignment.segments));
    output.at(1).push_back(get_alignment(get_complement(alignment.segments)));
  }

  return output;
}

//==============================================================================
FinalConstraints compute_final_ShouldGo_constraints(
    const PeerToPeerBlockers& peer_blockers,
    const PeerToPeerAlignment& peer_alignment)
{
  using IndexToZeroOrderConstraints =
    std::unordered_map<std::size_t, std::vector<ConstConstraintPtr>>;

  using ZeroOrderConstraintMap =
    std::unordered_map<std::size_t, IndexToZeroOrderConstraints>;

  ZeroOrderConstraintMap zero_order;
  for (const auto& p : peer_blockers)
  {
    const std::size_t participant = p.first;
    auto& index_to_constraints = zero_order[participant];

    for (const auto& peer : p.second)
    {
      for (const auto& checkpoint : peer.second)
        index_to_constraints[checkpoint.first].push_back(checkpoint.second);
    }
  }

  Blockers first_order;
  for (const auto& p : zero_order)
  {
    const std::size_t participant = p.first;
    auto& index_to_constraint = first_order[participant];

    for (const auto& checkpoint : p.second)
    {
      const std::size_t index = checkpoint.first;
      const auto& constraints = checkpoint.second;
      assert(!constraints.empty());

      if (constraints.size() > 1)
      {
        auto and_constraint = std::make_shared<AndConstraint>();
        for (const auto& c : constraints)
          and_constraint->add(c);

        index_to_constraint[index] = std::move(and_constraint);
      }
      else
      {
        assert(constraints.size() == 1);
        index_to_constraint[index] = constraints.front();
      }
    }
  }

  /// We remap Peer->Alignment->Index->caveat to Index->Peer->Caveat
  struct Caveats
  {
    std::shared_ptr<Timeline> timeline;
    std::vector<std::size_t> caveats;
  };

  using IndexToPeerToCaveats =
    std::unordered_map<std::size_t, std::unordered_map<std::size_t, Caveats>>;
  using AllCaveats = std::unordered_map<std::size_t, IndexToPeerToCaveats>;
  AllCaveats all_caveats;
  for (const auto& p : peer_alignment)
  {
    const std::size_t participant = p.first;
    auto& p_caveats = all_caveats[participant];

    for (const auto& o : p.second)
    {
      const std::size_t other = o.first;

      for (const auto& align : o.second)
      {
        for (const auto& c : align.index_to_caveats)
        {
          auto& caveat = p_caveats[c.first][other];
          caveat.timeline = align.timeline;
          caveat.caveats.insert(
                caveat.caveats.end(),
                c.second.begin(),
                c.second.end());
        }
      }
    }
  }

  using Shares = Blockers;
  Shares shares;
  for (const auto& p : all_caveats)
  {
    const std::size_t participant = p.first;
    for (const auto& c : p.second)
    {
      const std::size_t checkpoint = c.first;
      std::vector<ConstConstraintPtr> sharing_constraints;
      for (const auto& o : c.second)
      {
        const std::size_t other = o.first;
        const auto other_constraints_it = first_order.find(other);
        if (other_constraints_it == first_order.end())
          continue;

        const auto& other_constraints = other_constraints_it->second;

        const auto timeline = o.second.timeline;
        for (const auto& caveat : o.second.caveats)
        {
          const auto c_it = other_constraints.find(caveat);
          if (c_it == other_constraints.end())
            continue;

          auto sharing_constraint = std::make_shared<OrConstraint>();
          sharing_constraint->add(behind(other, participant, timeline));
          sharing_constraint->add(passed(other, caveat));
          sharing_constraint->add(c_it->second);

          sharing_constraints.emplace_back(std::move(sharing_constraint));
        }
      }

      if (!sharing_constraints.empty())
      {
        shares[participant][checkpoint] =
            std::make_shared<AndConstraint>(std::move(sharing_constraints));
      }
    }
  }

  const auto gridlock_constraint = compute_gridlock_constraint(first_order);

  // Now we will move the first order constraints into the container for the
  // final order constraints and modify them in place by adding the gridlock
  // constraint to them
  Blockers final_order;
  for (auto& p : first_order)
  {
    const std::size_t participant = p.first;
    const auto p_shares_it = shares.find(participant);

    for (auto& c : p.second)
    {
      const std::size_t checkpoint = c.first;
      auto and_constraint = std::make_shared<AndConstraint>();
      and_constraint->add(gridlock_constraint);
      and_constraint->add(c.second);

      if (p_shares_it != shares.end())
      {
        const auto c_shares_it = p_shares_it->second.find(checkpoint);
        if (c_shares_it != p_shares_it->second.end())
          and_constraint->add(c_shares_it->second);
      }

      final_order[participant][checkpoint] = and_constraint;
    }
  }

  return FinalConstraints{std::move(final_order), gridlock_constraint};
}

} // namespace blockade
} // namespace rmf_traffic

//==============================================================================
std::ostream& operator<<(
    std::ostream& os, const rmf_traffic::blockade::Bracket& b)
{
  if (b.include_start)
    os << "[";
  else
    os << "(";

  os << b.start << ", " << b.finish;

  if (b.include_finish)
    os << "]";
  else
    os << ")";

  return os;
}

//==============================================================================
std::ostream& operator<<(
    std::ostream& os, const rmf_traffic::blockade::ConflictBracketPair& pair)
{
  os << pair.A << "x" << pair.B;
  return os;
}

//==============================================================================
std::ostream& operator<<(
    std::ostream& os, const rmf_traffic::blockade::AlignedBracketPair& pair)
{
  os << pair.A << "|" << pair.B;
  return os;
}
