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

#ifndef RMF_TRAFFIC__TEST__UNIT__SCHEDULE__UTILS_TRAJECTORY_HPP
#define RMF_TRAFFIC__TEST__UNIT__SCHEDULE__UTILS_TRAJECTORY_HPP

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <iostream>

#include <rmf_utils/catch.hpp>

inline void CHECK_EQUAL_TRAJECTORY(const rmf_traffic::Trajectory *t, rmf_traffic::Trajectory t2)
{
  rmf_traffic::Trajectory t1= *t;
  REQUIRE(t1.size()==t2.size());

  for(auto it1=t1.begin(),it2=t2.begin();it1!=t1.end();it1++,it2++)
  {
    CHECK(it1->position() == it2->position());
    CHECK(it1->velocity() == it2->velocity());
    CHECK(it1->time() == it2->time());
  }
}

inline void CHECK_TRAJECTORY_COUNT(
    const rmf_traffic::schedule::Viewer& d,
    const std::size_t expected_participant_num,
    const std::size_t expected_trajectory_num)
{
  const auto view = d.query(rmf_traffic::schedule::query_all());

  std::size_t trajectory_count = 0;
  std::unordered_set<rmf_traffic::schedule::ParticipantId> participants;
  for(const auto& v : view)
  {
    participants.insert(v.participant);
    ++trajectory_count;
  }

  CHECK(trajectory_count == expected_trajectory_num);
  CHECK(participants.size() == expected_participant_num);
}

inline std::vector<rmf_traffic::Trajectory> get_conflicting_trajectories(
    const rmf_traffic::schedule::Viewer& viewer,
    const rmf_traffic::schedule::Viewer::View& view,
    const rmf_traffic::Profile& p,
    const rmf_traffic::Trajectory& t)
{
  std::vector<rmf_traffic::Trajectory> collision_trajectories;
  for(const auto& v : view)
  {
    const auto& v_p = viewer.get_participant(v.participant)->profile();
    const auto& v_t = v.route.trajectory();
    if(rmf_traffic::DetectConflict::between(v_p, v_t, p, t))
        collision_trajectories.push_back(v_t);
  }

  return collision_trajectories;
}

inline rmf_traffic::schedule::Writer::Input create_test_input(
    rmf_traffic::RouteId id, const rmf_traffic::Trajectory& t)
{
  return rmf_traffic::schedule::Writer::Input{
    {
      {id, std::make_shared<rmf_traffic::Route>("test_map", t)}
    }
  };
}



#endif //RMF_TRAFFIC__TEST__UNIT__SCHEDULE__UTILS_TRAJECTORY_HPP
