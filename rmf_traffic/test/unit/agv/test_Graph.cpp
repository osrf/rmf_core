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

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_traffic/Conflict.hpp>

#include <rmf_utils/catch.hpp>

#include "../utils_Trajectory.hpp"

#include <iostream>
#include<iomanip>

void CHECK_WAYPOINT(rmf_traffic::agv::Graph::Waypoint wp,Eigen::Vector2d waypoint_location,std::string test_map_name,std::size_t index, bool holding)
    {  
        CHECK((wp.get_location()-waypoint_location).norm() ==Approx(0));
        CHECK(wp.get_map_name()==test_map_name);
        CHECK(wp.index()==index);
        CHECK(wp.is_holding_point()==holding);

    }

SCENARIO("Tests for Graph API")
{

using namespace std::chrono_literals;

rmf_traffic::Time time= std::chrono::steady_clock::now();
rmf_traffic::agv::Graph graph;
REQUIRE(graph.num_waypoints()==0);
REQUIRE(graph.num_lanes()==0);
REQUIRE(graph.num_doors()==0);


const std::string test_map_name="test_map";

WHEN("Graph is empty")
    {
        // perhaps this function can accept an \[out] Waypoint parameter and return bool
        CHECK_THROWS(graph.get_waypoint(0));
        

    }
WHEN("A non-holding waypoint is added")
    {
        Eigen::Vector2d waypoint_location=Eigen::Vector2d{0,0};
        auto& wp1= graph.add_waypoint(test_map_name,waypoint_location);
        CHECK(graph.num_waypoints()==1);
        CHECK_WAYPOINT(wp1,waypoint_location,test_map_name,0,false);

        wp1.set_holding_point(true);
        wp1.set_map_name("not_test");
        wp1.set_location(Eigen::Vector2d{1,1});

        //checking values of updated waypoint
        const auto wp=graph.get_waypoint(0);
        CHECK_WAYPOINT(wp,Eigen::Vector2d{1,1},"not_test",0,true);

        WHEN("A second waypoint is added")
        {

            graph.add_waypoint(test_map_name,Eigen::Vector2d{2,2});
            CHECK(graph.num_waypoints()==2);
            CHECK_WAYPOINT(graph.get_waypoint(1),Eigen::Vector2d{2,2},test_map_name,1,false);


        }

    }
WHEN("A lane without a door is added")
    {

        
    }



}