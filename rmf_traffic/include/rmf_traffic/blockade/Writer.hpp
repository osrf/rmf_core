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

#ifndef RMF_TRAFFIC__BLOCKADE__WRITER_HPP
#define RMF_TRAFFIC__BLOCKADE__WRITER_HPP

#include <Eigen/Geometry>

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace blockade {

using ParticipantId = uint64_t;
using ReservationId = uint64_t;
using CheckpointId = uint64_t;
using Version = uint64_t;

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

//==============================================================================
using Assignments = std::unordered_map<ParticipantId, ReservedRange>;

//==============================================================================
class Writer
{
public:

  struct Checkpoint
  {
    Eigen::Vector2d position;
    std::string map_name;
    bool can_hold;
  };

  struct Reservation
  {
    ReservationId id;
    std::vector<Checkpoint> path;
    double radius;
  };

  virtual void set(
      ParticipantId participant,
      const Reservation& reservation) = 0;

  virtual void ready(
      ParticipantId participant,
      ReservationId reservation,
      CheckpointId checkpoint) = 0;

  virtual void reached(
      ParticipantId participant,
      ReservationId reservation,
      CheckpointId checkpoint) = 0;

  virtual void finished(
      ParticipantId participant,
      ReservationId reservation) = 0;

  virtual const Assignments& assignments() const = 0;

  struct Status
  {
    ReservationId reservation;
    CheckpointId last_ready;
    CheckpointId last_reached;
  };

  virtual const std::unordered_map<ParticipantId, Status>& statuses() const = 0;

  virtual ~Writer() = default;
};

} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__WRITER_HPP
