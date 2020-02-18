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

#ifndef RMF_TRAFFIC__TEST__UNIT__UTILS_CONFLICT_HPP
#define RMF_TRAFFIC__TEST__UNIT__UTILS_CONFLICT_HPP

#include "src/rmf_traffic/DetectConflictInternal.hpp"

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_utils/catch.hpp>

using ConflictData = rmf_traffic::DetectConflict::Implementation::Conflict;

//==============================================================================
inline std::vector<ConflictData> get_conflicts(
    const rmf_traffic::Profile& p1,
    const rmf_traffic::Trajectory& t1,
    const rmf_traffic::Profile& p2,
    const rmf_traffic::Trajectory& t2)
{
  rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
  rmf_traffic::DetectConflict::Implementation::between(
        p1, t1, p2, t2, rmf_traffic::DetectConflict::Interpolate::CubicSpline,
        &conflicts);

  return conflicts;
}

//==============================================================================
inline void CHECK_between_is_commutative(
    const rmf_traffic::Profile& p1,
    const rmf_traffic::Trajectory& t1,
    const rmf_traffic::Profile& p2,
    const rmf_traffic::Trajectory& t2)
{
  const auto conflicts_1 = get_conflicts(p1, t1, p2, t2);
  const auto conflicts_2 = get_conflicts(p2, t2, p1, t1);

  REQUIRE(conflicts_1.size() == conflicts_2.size());
  for (auto i_1 = conflicts_1.begin(), i_2 = conflicts_2.begin();
       i_1 != conflicts_1.end(); i_1++, i_2++)
  {
    CHECK(i_1->time == i_2->time);
    rmf_traffic::Trajectory::const_iterator conflict_1_t1 = i_1->a_it;
    rmf_traffic::Trajectory::const_iterator conflict_2_t1 = i_2->b_it;
    CHECK(conflict_1_t1 == conflict_2_t1);

    rmf_traffic::Trajectory::const_iterator conflict_1_t2 = i_1->b_it;
    rmf_traffic::Trajectory::const_iterator conflict_2_t2 = i_2->a_it;
    CHECK(conflict_1_t2 == conflict_2_t2);
  }
}

inline void CHECK_ConflictData(
    const ConflictData& data,
    const rmf_traffic::Time start_time,
    const rmf_traffic::Time expected_conflict_time,
    rmf_traffic::Trajectory::const_iterator t1_expected_conflict_segment,
    rmf_traffic::Trajectory::const_iterator t2_expected_conflict_segment,
    const double error_margin)
{
    // t1 and t2 are precisely pointing to the Trajectory segments we are trying to verify,
    // Not a "mock up" waypoint created separately
    // Note that the const_iterators should not be the first in a trajectory: ie trajectory.begin()
    // ConflictData will not return the first iterator as it only represents a point in time and cannot "conflict"

    rmf_traffic::Trajectory::const_iterator t1_conflict_segment = data.a_it;
    rmf_traffic::Trajectory::const_iterator t2_conflict_segment = data.b_it;
    CHECK(t1_conflict_segment == t1_expected_conflict_segment);
    CHECK(t2_conflict_segment == t2_expected_conflict_segment);

    // We can only test for approximate accuracies due to the numerical nature of FCL
    const double expected_duration = rmf_traffic::time::to_seconds(expected_conflict_time - start_time);
    const double computed_duration = rmf_traffic::time::to_seconds(data.time - start_time);
    CHECK(computed_duration == Approx(expected_duration).margin(error_margin));
}

struct ConflictDataParams
{
    const rmf_traffic::Time start_time;
    const rmf_traffic::Time expected_conflict_time;
    rmf_traffic::Trajectory::const_iterator t1_expected_conflict_segment;
    rmf_traffic::Trajectory::const_iterator t2_expected_conflict_segment;
    const double error_margin;
};

inline void CHECK_ConflictList(
    const std::vector<ConflictData>& computed_conflicts,
    const std::vector<ConflictDataParams>& expected_conflicts)
{
  REQUIRE(computed_conflicts.size() == computed_conflicts.size());
  auto c_it = computed_conflicts.begin();
  auto exp_it = expected_conflicts.begin();

  for (; c_it != computed_conflicts.end(); ++c_it, ++exp_it)
  {
    // unpack
    const rmf_traffic::Time start_time = exp_it->start_time;
    const rmf_traffic::Time expected_conflict_time =
        exp_it->expected_conflict_time;

    const auto t1_expected_conflict_segment =
        exp_it->t1_expected_conflict_segment;

    const auto t2_expected_conflict_segment =
        exp_it->t2_expected_conflict_segment;

    ConflictData data = *c_it;
    const double error_margin = exp_it->error_margin;

    CHECK_ConflictData(
          data,
          start_time,
          expected_conflict_time,
          t1_expected_conflict_segment,
          t2_expected_conflict_segment,
          error_margin);
  }
}

#endif // RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
