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
    const auto conflicts_1 = rmf_traffic::DetectConflict::narrow_phase(t1, t2);
    const auto conflicts_2 = rmf_traffic::DetectConflict::narrow_phase(t2, t1);
    REQUIRE(conflicts_1.size() == conflicts_2.size());
    for (auto i_1 = conflicts_1.begin(), i_2 = conflicts_2.begin(); i_1 != conflicts_1.end(); i_1++, i_2++)
    {
        CHECK(i_1->get_time() == i_2->get_time());
        // add iterative checks
    }
}

// inline void CHECK_conflict_trace(std::vector<rmf_traffic::ConflictData> detected_conflicts,
//                                  std::vector<std::tuple<rmf_traffic::Trajectory::iterator,
//                                                        rmf_traffic::Trajectory::iterator,
//                                                        rmf_traffic::Time>>
//                                      expected_conflicts)
// {
    // Check expected details of the conflict data
// }
#endif // RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
