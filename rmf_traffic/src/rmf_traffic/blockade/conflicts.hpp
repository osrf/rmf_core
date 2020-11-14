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

#ifndef SRC__RMF_TRAFFIC__BLOCKADE__CONFLICTS_HPP
#define SRC__RMF_TRAFFIC__BLOCKADE__CONFLICTS_HPP

#include "Constraint.hpp"
#include <rmf_traffic/blockade/Writer.hpp>

#include <map>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
struct Bracket
{
  std::size_t start;
  std::size_t finish;

  bool include_start;
  bool include_finish;

  bool operator==(const Bracket& other) const;
};

//==============================================================================
struct BracketPair
{
  Bracket A;
  Bracket B;

  bool operator==(const BracketPair& other) const;
};

//==============================================================================
struct ConflictBracketPair : BracketPair { };
struct AlignedBracketPair : BracketPair { };

//==============================================================================
class Timeline
{
public:

  Timeline(const std::vector<AlignedBracketPair>& alignments);

  /// Returns true if waypoint A_a is definitely behind waypoint B_b along an
  /// aligned pair of paths. Return false if A_a is not behind B_b or if it
  /// cannot be determined.
  bool is_behind(std::size_t a, std::size_t b) const;

private:
  struct Comparison
  {
    enum Type
    {
      EqualTo,
      LessThan
    };

    Type type;
    std::size_t index;
  };

  std::map<std::size_t, Comparison, std::greater<std::size_t>> _map;
};

//==============================================================================
struct AlignedBracketSet
{
  AlignedBracketPair whole_bracket;
  std::vector<AlignedBracketPair> segments;
};

//==============================================================================
struct Brackets
{
  std::vector<ConflictBracketPair> conflicts;
  std::vector<AlignedBracketSet> alignments;
};

//==============================================================================
Brackets compute_brackets(
    const std::vector<Writer::Checkpoint>& path_a,
    double radius_a,
    const std::vector<Writer::Checkpoint>& path_b,
    double radius_b,
    double angle_threshold);

//==============================================================================
std::array<IndexToConstraint, 2> compute_blockers(
    const std::vector<ConflictBracketPair>& conflict_brackets,
    std::size_t id_a,
    std::size_t a_path_size,
    std::size_t id_b,
    std::size_t b_path_size);

//==============================================================================
std::array<IndexToConstraint, 2> compute_alignments(
    const std::vector<AlignedBracketSet>& alignments,
    const std::array<IndexToConstraint, 2> blockers,
    std::size_t id_a,
    std::size_t id_b);

//==============================================================================
// A map from <a participant's peer> to <the map from the participant's index to
// the constraint imposed by the peer>
using PeerToIndexToConstraint =
  std::unordered_map<std::size_t, IndexToConstraint>;

//==============================================================================
/// A map from <a participant> to <the Peer-To-Index-To-Constraint map>
///
/// e.g. if
/// g(A2) := h(B1) || r(B3)
///
/// then
/// peer_to_peer_blockers[A][B][2] := h(B1) || r(B3)
using PeerToPeerBlockers =
  std::unordered_map<std::size_t, PeerToIndexToConstraint>;

//==============================================================================
using PeerAlignment = std::unordered_map<std::size_t, IndexToConstraint>;

//==============================================================================
using PeerToPeerAlignment = std::unordered_map<std::size_t, PeerAlignment>;

//==============================================================================
struct FinalConstraints
{
  Blockers should_go;
  ConstConstraintPtr gridlock;
};

//==============================================================================
FinalConstraints compute_final_ShouldGo_constraints(
    const PeerToPeerBlockers& peer_blockers,
    const PeerToPeerAlignment& peer_alignment);

} // namespace blockade
} // namespace rmf_traffic

//==============================================================================
std::ostream& operator<<(
    std::ostream& os, const rmf_traffic::blockade::Bracket& b);

//==============================================================================
std::ostream& operator<<(
    std::ostream& os, const rmf_traffic::blockade::ConflictBracketPair& pair);

//==============================================================================
std::ostream& operator<<(
    std::ostream& os, const rmf_traffic::blockade::AlignedBracketPair& pair);

#endif // SRC__RMF_TRAFFIC__BLOCKADE__CONFLICTS_HPP
