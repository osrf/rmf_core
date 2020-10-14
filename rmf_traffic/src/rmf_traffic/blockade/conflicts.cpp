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

namespace rmf_traffic {
namespace blockade {

//==============================================================================
bool compatible_start_and_finish(
    std::size_t start,
    bool include_start,
    std::size_t finish,
    bool include_finish)
{
  if (start < finish+1)
    return false;

  if (start == finish)
  {
    if (!include_start && !include_finish)
      return false;
  }

  return true;
}

//==============================================================================
bool can_merge_brackets(
    const ConflictBracket& bracket0,
    const ConflictBracket& bracket1)
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
ConflictBracket merge_brackets(
    const ConflictBracket& bracket0,
    const ConflictBracket& bracket1)
{
  ConflictBracket output;

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
bool try_merge(
    BracketPair& pair0,
    const BracketPair& pair1,
    std::size_t& merge_count)
{
  if (!can_merge_brackets(pair0.A, pair1.A)
      || !can_merge_brackets(pair0.B, pair1.B))
    return false;

  pair0.A = merge_brackets(pair0.A, pair1.A);
  pair0.B = merge_brackets(pair0.B, pair1.B);
  ++merge_count;
  return true;
}

//==============================================================================
void expand_bracket(
    ConflictBracket& bracket,
    const std::vector<Writer::Item>& path)
{
  // This function accounts for points that the robot is not allowed to hold at.
  // These points get absorbed into the brackets as if they are part of the
  // conflicts.

  while (bracket.start > 0)
  {
    const bool can_hold_at_start =
        path.at(bracket.start-1).can_hold_at_finish;

    if (can_hold_at_start && !bracket.include_start)
      break;

    --bracket.start;
    bracket.include_start = false;
  }

  while (bracket.finish < path.size())
  {
    const bool can_hold_at_finish =
        path.at(bracket.finish-1).can_hold_at_finish;

    if (can_hold_at_finish && !bracket.include_finish)
      break;

    ++bracket.finish;
    bracket.include_finish = false;
  }
}

//==============================================================================
std::vector<BracketPair> compute_conflict_brackets(
    const std::vector<Writer::Item>& path_a,
    const std::vector<Writer::Item>& path_b,
    const double angle_threshold)
{
  std::multimap<std::size_t, BracketPair> a_set;
  for (std::size_t a=0; a < path_a.size(); ++a)
  {
    const auto& it_a = path_a[a];
    const Segment segment_a{it_a.start, it_a.finish, it_a.radius};

    for (std::size_t b=0; b < path_b.size(); ++b)
    {
      const auto& it_b = path_b[b];
      const Segment segment_b{it_b.start, it_b.finish, it_b.radius};

      const auto info = detect_conflict(segment_a, segment_b, angle_threshold);

      if (!info.has_conflict)
        continue;

      BracketPair pair;
      pair.A.start = a;
      pair.A.finish = a+1;
      pair.A.include_start = info.include_cap_a[ConflictInfo::Start];
      pair.A.include_finish = info.include_cap_b[ConflictInfo::Finish];

      pair.B.start = b;
      pair.B.finish = b+1;
      pair.B.include_start = info.include_cap_b[ConflictInfo::Start];
      pair.B.include_finish = info.include_cap_b[ConflictInfo::Finish];

      expand_bracket(pair.A, path_a);
      expand_bracket(pair.B, path_b);

      a_set.emplace(std::make_pair(pair.A.start, pair));
    }
  }

  std::vector<BracketPair> final_pairs;
  while (!a_set.empty())
  {
    std::size_t merge_count = 0;

    auto it_a = a_set.begin();
    auto pair = it_a->second;
    a_set.erase(it_a++);

    while (it_a != a_set.end())
    {
      if (pair.A.finish < it_a->second.A.start+1)
        break;

      if (try_merge(pair, it_a->second, merge_count))
      {
        a_set.erase(it_a++);
      }
      else
      {
        ++it_a;
      }
    }

    if (merge_count == 0)
    {
      final_pairs.push_back(pair);
    }
    else
    {
      a_set.insert(std::make_pair(pair.A.start, pair));
    }
  }

  return final_pairs;
}

} // namespace blockade
} // namespace rmf_traffic
