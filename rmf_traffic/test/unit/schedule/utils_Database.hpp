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

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_utils/catch.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include<iostream>
#include<rmf_traffic/Conflict.hpp>
inline void CHECK_EQUAL_TRAJECTORY(const rmf_traffic::Trajectory *t, rmf_traffic::Trajectory t2)
{
rmf_traffic::Trajectory t1= *t;
REQUIRE(t1.size()==t2.size());

    for(auto it1=t1.begin(),it2=t2.begin();it1!=t1.end();it1++,it2++)
    {
    CHECK(it1->get_finish_position()==it2->get_finish_position());
    CHECK(it1->get_finish_time()==it2->get_finish_time());
    CHECK(it1->get_profile()==it2->get_profile());
    }
}

inline void CHECK_TRAJECTORY_COUNT(rmf_traffic::schedule::Database d, int n)
{
    int trajectory_count=0;
    auto query_everything= rmf_traffic::schedule::query_everything();
    auto changes= d.changes(query_everything);
    for(auto it=changes.begin();it!=changes.end();it++)
    {
      //std::cout<<"ID:"<<static_cast<int>(it->get_mode())<<std::endl;
      if(static_cast<int>(it->get_mode())==1)
        ++trajectory_count;
      }

    CHECK(trajectory_count==n);

} 

inline std::vector<rmf_traffic::Trajectory> get_collision_trajectories(const rmf_traffic::schedule::Viewer::View& view,rmf_traffic::Trajectory& t)
{

  std::vector<rmf_traffic::Trajectory> collision_trajectories;
  for(auto trajectory: view)
  {
    if(rmf_traffic::DetectConflict::between(trajectory,t).size()>0)
        collision_trajectories.push_back(trajectory);

  }
  return collision_trajectories;

}





#endif //RMF_TRAFFIC__TEST__UNIT__SCHEDULE__UTILS_TRAJECTORY_HPP
