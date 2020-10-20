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

namespace rmf_traffic {
namespace blockade {

//==============================================================================
struct ConflictBracket
{
  std::size_t start;
  std::size_t finish;

  bool include_start;
  bool include_finish;
};

//==============================================================================
struct BracketPair
{
  ConflictBracket A;
  ConflictBracket B;
};

//==============================================================================
std::vector<BracketPair> compute_conflict_brackets(
    const std::vector<Writer::Checkpoint>& path_a,
    double radius_a,
    const std::vector<Writer::Checkpoint>& path_b,
    double radius_b,
    double angle_threshold);

//==============================================================================
std::array<IndexToConstraint, 2> compute_blockers(
    const std::vector<BracketPair>& conflict_brackets,
    std::size_t id_a,
    std::size_t a_path_size,
    std::size_t id_b,
    std::size_t b_path_size);

//==============================================================================
// A map from <a participant's peer> to <the map from the participant's index to
// the constraint imposed by the peer>
using PeerToIndexToConstraint =
  std::unordered_map<std::size_t, IndexToConstraint>;

//==============================================================================
// A map from <a participant> to <the Peer-To-Index-To-Constraint map>
using PeerToPeerBlockers =
  std::unordered_map<std::size_t, PeerToIndexToConstraint>;

//==============================================================================
Blockers compute_final_ShouldGo_constraints(
    const PeerToPeerBlockers& peer_blockers);

} // namespace blockade
} // namespace rmf_traffic

//==============================================================================
std::ostream& operator<<(
    std::ostream& os, const rmf_traffic::blockade::ConflictBracket& b)
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
    std::ostream& os, const rmf_traffic::blockade::BracketPair& pair)
{
  os << pair.A << "x" << pair.B;
  return os;
}

#endif // SRC__RMF_TRAFFIC__BLOCKADE__CONFLICTS_HPP
