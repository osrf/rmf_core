/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "ShapeInternal.hpp"

#include <rmf_traffic/Conflict.hpp>

#include <fcl/continuous_collision.h>
#include <fcl/ccd/motion.h>

namespace rmf_traffic {

//==============================================================================
class ConflictData::Implementation
{
public:

  Time time;
  Segments segments;

};

//==============================================================================
Time ConflictData::get_time() const
{
  return _pimpl->time;
}

//==============================================================================
ConflictData::Segments ConflictData::get_segments()
{
  return _pimpl->segments;
}

//==============================================================================
ConflictData::ConflictData()
{
  // Do nothing
}

//==============================================================================
const char* bad_conflict_access::what() const noexcept
{
  return "[rmf_traffic::bad_conflict_access] Attempted to dereference a "
      "ConflictResult which contained no conflict. This is not permitted.";
}

//==============================================================================
bad_conflict_access::bad_conflict_access()
{
  // This constructor is a no-op, but we'll keep a definition for it in case we
  // need it in the future. Allowing the default constructor to be inferred
  // could cause issues if we want to change the implementation of this
  // exception in the future, like if we want to add more information to the
  // error message output.
}

//==============================================================================
class ConflictResult::Implementation
{
public:

  ConflictData data;

  Implementation(ConflictData incoming_data)
    : data(incoming_data)
  {
    // Do nothing
  }

  static ConflictData make_conflict(Time time, ConflictData::Segments segments)
  {
    ConflictData result;
    result._pimpl = rmf_utils::make_impl<ConflictData::Implementation>(
          time, std::move(segments));

    return result;
  }
};

//==============================================================================
bool ConflictResult::has_conflict() const
{
  return _pimpl.get();
}

//==============================================================================
ConflictResult::operator bool() const
{
  return has_conflict();
}

//==============================================================================
const ConflictData& ConflictResult::operator*() const
{
  if(!_pimpl)
    throw bad_conflict_access();

  return _pimpl->data;
}

//==============================================================================
const ConflictData* ConflictResult::operator->() const
{
  if(!_pimpl)
    throw bad_conflict_access();

  return &_pimpl->data;
}

//==============================================================================
ConflictResult::ConflictResult()
{
  // Do nothing
}

//==============================================================================
ConflictResult DetectConflict::between(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b)
{
  if(!broad_phase(trajectory_a, trajectory_b))
    return ConflictResult();

  return narrow_phase(trajectory_a, trajectory_b);
}

//==============================================================================
bool DetectConflict::broad_phase(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b)
{
  if(trajectory_a.get_map_name() != trajectory_b.get_map_name())
    return false;

  const auto* t_a0 = trajectory_a.start_time();
  const auto* t_bf = trajectory_b.finish_time();

  if(!t_a0 || !t_bf)
  {
    // If the start or finish time of either Trajectory is missing, then it is
    // an empty Trajectory, and so no conflict can happen.
    return false;
  }

  if(*t_bf < *t_a0)
  {
    // If Trajectory `b` finishes before Trajectory `a` starts, then there
    // cannot be any conflict.
    return false;
  }

  const auto* t_b0 = trajectory_b.start_time();
  const auto* t_af = trajectory_a.finish_time();

  if(*t_af < *t_b0)
  {
    // If Trajectory `a` finished before Trajectory `b` starts, then there
    // cannot be any conflict.
    return false;
  }

  return true;
}

//==============================================================================
class DetectConflict::Implementation
{
public:

  static ConflictResult check_static_segments(
      const Trajectory::Segment& segment_a,
      const Trajectory::Segment& segment_b)
  {

  }

  static ConflictResult check_against_static(
      const Trajectory& trajectory,
      const Trajectory::Segment& obstacle)
  {
    Trajectory::const_iterator it_f =
        trajectory.find(obstacle.get_finish_time());
    Trajectory::const_iterator it_0 = --Trajectory::const_iterator(it_f);


  }

};

//==============================================================================
ConflictResult DetectConflict::narrow_phase(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b)
{
  const Time& t_a0 = *trajectory_a.start_time();
  const Time& t_b0 = *trajectory_b.start_time();

  Trajectory::const_iterator a_it;
  Trajectory::const_iterator b_it;

  if(t_a0 < t_b0)
  {
    a_it = trajectory_a.find(t_b0);
    b_it = ++trajectory_b.begin();
  }
  else if(t_b0 < t_a0)
  {
    a_it = ++trajectory_a.begin();
    b_it = trajectory_b.find(t_a0);
  }
  else
  {
    // The Trajectories begin at the exact same time
    a_it = ++trajectory_a.begin();
    b_it = ++trajectory_b.begin();
  }

  // Check for the special case where one of the Trajectories has only one
  // Segment (making it static).
  const bool a_static = a_it == trajectory_a.end();
  const bool b_static = b_it == trajectory_b.end();
  if(a_static)
  {
    if(b_static)
    {
      return Implementation::check_static_segments(
            *trajectory_a.begin(), *trajectory_b.begin());
    }
    return Implementation::check_against_static(
          trajectory_b, *trajectory_a.begin());
  }
  else if(b_static)
  {
    return Implementation::check_against_static(
          trajectory_a, *trajectory_b.begin());
  }

  if(b_it == trajectory_b.end())
  {
    // trajectory_b is static
    return Implementation::check_against_static(
          trajectory_a, *trajectory_b.begin());
  }

  while(a_it != trajectory_a.end() && b_it != trajectory_b.end())
  {

  }

  // If no conflict was detected, return an empty ConflictResult
  return ConflictResult();
}

} // namespace rmf_traffic
