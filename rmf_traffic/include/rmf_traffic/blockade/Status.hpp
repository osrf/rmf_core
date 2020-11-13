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

#ifndef RMF_TRAFFIC__BLOCKADE__STATUS_HPP
#define RMF_TRAFFIC__BLOCKADE__STATUS_HPP

#include <cstddef>
#include <cstdint>
#include <unordered_map>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
using ParticipantId = uint64_t;
using ReservationId = uint64_t;
using CheckpointId = uint64_t;
using Version = uint64_t;

//==============================================================================
struct Status
{
  // TODO(MXG): PIMPLfy this
  ReservationId reservation;
  std::optional<CheckpointId> last_ready;
  CheckpointId last_reached;
  bool critical_error;
};

//==============================================================================
struct ReservedRange
{
  std::size_t begin;
  std::size_t end;

  bool operator==(const ReservedRange& other) const
  {
    return (begin == other.begin) && (end == other.end);
  }
};

} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__STATUS_HPP
