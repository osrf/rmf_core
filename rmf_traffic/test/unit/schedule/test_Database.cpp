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


#include "rmf_traffic/schedule/Database.hpp"
#include <rmf_traffic/geometry/Box.hpp>

#include <rmf_utils/catch.hpp>
#include<iostream>
using namespace std::chrono_literals;


SCENARIO("Test Database Conflicts")

{

    GIVEN("A Database with single trajectory")
    {

        rmf_traffic::schedule::Database db;

        //check for empty instantiation
        const rmf_traffic::schedule::Query query_param= rmf_traffic::schedule::query_everything();
        rmf_traffic::schedule::Database::Patch changes= db.changes(query_param);
        REQUIRE(changes.latest_version()==0);
        CHECK(changes.size()==0);
        for(auto x :changes)
        {
            std::cout<<static_cast<int>(x.get_mode())<<std::endl;
            std::cout<<x.cull()->time().time_since_epoch().count()<<std::endl;

        }
        //adding simple two-point trajecotry
        rmf_traffic::Trajectory t1("test_map");
        rmf_traffic::Trajectory::ProfilePtr profile=
    
        WHEN("Trajectory is inserted")
        {
            //check changes
        }

        WHEN("Trajectory is interrupted")
        {

        }

        WHEN("Trajectory is replaced")
        {

        }

        WHEN("Trajectory is erased")
        {
            
        }
        WHEN("Trajectory is culled")
        {
            
        }




    }




}