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

#include <rmf_traffic/Conflict.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_utils/catch.hpp>

//==============================================================================

inline void CHECK_broad_phase_is_commutative(rmf_traffic::Trajectory t1, rmf_traffic::Trajectory t2)
{
    CHECK(rmf_traffic::DetectConflict::broad_phase(t1, t2) == rmf_traffic::DetectConflict::broad_phase(t2, t1));
}

inline void CHECK_narrow_phase_is_commutative(rmf_traffic::Trajectory t1, rmf_traffic::Trajectory t2)
{
    REQUIRE(rmf_traffic::DetectConflict::broad_phase(t1, t2));
    auto conflicts_1 = rmf_traffic::DetectConflict::narrow_phase(t1, t2);
    auto conflicts_2 = rmf_traffic::DetectConflict::narrow_phase(t2, t1);
    REQUIRE(conflicts_1.size() == conflicts_2.size());
    for (auto i_1 = conflicts_1.begin(), i_2 = conflicts_2.begin(); i_1 != conflicts_1.end(); i_1++, i_2++)
    {
        CHECK(i_1->get_time() == i_2->get_time());
        rmf_traffic::Trajectory::const_iterator conflict_1_t1 = i_1->get_segments().first;
        rmf_traffic::Trajectory::const_iterator conflict_2_t1 = i_2->get_segments().second;
        CHECK(conflict_1_t1 == conflict_2_t1);

        rmf_traffic::Trajectory::const_iterator conflict_1_t2 = i_1->get_segments().second;
        rmf_traffic::Trajectory::const_iterator conflict_2_t2 = i_2->get_segments().first;
        CHECK(conflict_1_t2 == conflict_2_t2);
    }
}

inline void CHECK_ConflictData(rmf_traffic::ConflictData conflict_data,
                               const rmf_traffic::Time start_time,
                               const rmf_traffic::Time expected_conflict_time,
                               rmf_traffic::Trajectory::const_iterator expected_t1,
                               rmf_traffic::Trajectory::const_iterator expected_t2,
                               const double error_margin = 0.2)
{
    // t1 and t2 are precisely pointing to the Trajectory segments we are trying to verify,
    // Not a "mock up" segment created separately

    rmf_traffic::Trajectory::const_iterator conflict_data_t1 = conflict_data.get_segments().first;
    rmf_traffic::Trajectory::const_iterator conflict_data_t2 = conflict_data.get_segments().second;
    CHECK(conflict_data_t1 == expected_t1);
    CHECK(conflict_data_t2 == expected_t2);

    // We can only test for approximate accuracies due to the numerical nature of FCL
    const double expected_duration = rmf_traffic::time::to_seconds(expected_conflict_time - start_time);
    const double computed_duration = rmf_traffic::time::to_seconds(conflict_data.get_time() - start_time);
    CHECK(computed_duration == Approx(expected_duration).margin(error_margin));
    }
#endif // RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
