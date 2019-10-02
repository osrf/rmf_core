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

inline void check_broad_phase_is_commutative(rmf_traffic::Trajectory t1, rmf_traffic::Trajectory t2)
{
    CHECK(rmf_traffic::DetectConflict::broad_phase(t1, t2) == rmf_traffic::DetectConflict::broad_phase(t2, t1));
}

inline void require_broad_phase_is_commutative(rmf_traffic::Trajectory t1, rmf_traffic::Trajectory t2)
{
    REQUIRE(rmf_traffic::DetectConflict::broad_phase(t1, t2) == rmf_traffic::DetectConflict::broad_phase(t2, t1));
}

// inline void check_narrow_phase_is_commutative(rmf_traffic::Trajectory t1, rmf_traffic::Trajectory t2)
// {
//     REQUIRE(rmf_traffic::DetectConflict::broad_phase(t1, t2));
//     auto conflicts_1 = rmf_traffic::DetectConflict::narrow_phase(t1, t2);
//     auto conflicts_2 = rmf_traffic::DetectConflict::narrow_phase(t2, t1);
// }
#endif // RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
