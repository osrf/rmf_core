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
        const rmf_traffic::schedule::Query query_everything= rmf_traffic::schedule::query_everything();
        rmf_traffic::schedule::Database::Patch changes= db.changes(query_everything);
        REQUIRE(changes.size()==0);
      /*  for(auto x :changes)
        {
            std::cout<<static_cast<int>(x.get_mode())<<std::endl;
            std::cout<<x.cull()->time().time_since_epoch().count()<<std::endl;

        }*/
        //adding simple two-point trajecotry
        const double profile_scale = 1.0;
        const rmf_traffic::Time time = std::chrono::steady_clock::now();
        rmf_traffic::geometry::Box shape(profile_scale,profile_scale);
        rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
        rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
        rmf_traffic::Trajectory t1("test_map");
        t1.insert(time, profile, Eigen::Vector3d{-5,0,0}, Eigen::Vector3d{0,0,0});
        t1.insert(time + 10s, profile, Eigen::Vector3d{5,0,0}, Eigen::Vector3d{0,0,0});
        REQUIRE(t1.size()==2);

        rmf_traffic::schedule::Version version=  db.insert(t1);
        REQUIRE(version==1);
        //create a query after_version 0
        auto query= rmf_traffic::schedule::make_query(0);
        changes=db.changes(query);
        CHECK(changes.size()==1); //currently failing 
        CHECK(changes.latest_version()==1);
        for(auto change :changes)
              {

                CHECK(static_cast<int>(change.get_mode())==1); //MODE:Insert=1
                CHECK(change.id()==1);
                CHECK(change.insert()!=nullptr);
              }
            
    
        WHEN("Another Trajectory is inserted")
        {
            rmf_traffic::Trajectory t2("test_map");
            t2.insert(time,profile, Eigen::Vector3d{0,-5,0},Eigen::Vector3d{0,0,0});
            t2.insert(time+10s,profile, Eigen::Vector3d{0,5,0},Eigen::Vector3d{0,0,0});
            REQUIRE(t2.size()==2);

            rmf_traffic::schedule::Version version2= db.insert(t2);
            REQUIRE(version2==2);
            changes=db.changes(query_everything);
            REQUIRE(changes.size()==2);
            REQUIRE(changes.latest_version()==2);
            //get the latest query
            query=rmf_traffic::schedule::make_query(1);
            changes=db.changes(query);
            REQUIRE(changes.size()==1);//only one change made since first insert
            REQUIRE(changes.begin()->id()==2);
            REQUIRE(static_cast<int>(changes.begin()->get_mode())==1);
            REQUIRE(changes.begin()->insert()!=nullptr);

        }

        WHEN("Trajectory is interrupted")
        {

          rmf_traffic::Trajectory t2("test_map");
          t2.insert(time+5s,profile, Eigen::Vector3d{0,1,0},Eigen::Vector3d{0,0,0});
          t2.insert(time+5s,profile, Eigen::Vector3d{0,1,0},Eigen::Vector3d{0,0,0});
          REQUIRE(t2.size()==2);
          rmf_traffic::schedule::Version version2= db.interrupt(1,t2,0s);
          CHECK(version2==2);
          changes=db.changes(query_everything);
          REQUIRE(changes.size()==2);
          REQUIRE(changes.latest_version()==2);
          auto interrupt_change= changes.end();
          REQUIRE(static_cast<int>(interrupt_change->get_mode())==2);
          REQUIRE(interrupt_change->interrupt()!=nullptr);
          auto interrupt=interrupt_change->interrupt();
          CHECK(interrupt->original_id()==1);
          CHECK(interrupt->interruption()==&t2); //INVALID. Write function to compare

          //check for number of trajectories in db
  

        }

        WHEN("Trajectory is delayed")
        {
          //introduce a delay after the first segment in t1
          rmf_traffic::schedule::Version version2= db.delay(1,time,5s);
          CHECK(version2==2);
          changes=db.changes(query_everything);
          REQUIRE(changes.size()==2);
          REQUIRE(changes.latest_version()==2);
          auto delay_change= changes.end();
          REQUIRE(static_cast<int>(delay_change->get_mode())==3);
          REQUIRE(delay_change->delay()!=nullptr);
          auto delay=delay_change->delay();
          CHECK(delay->original_id()==1);
          CHECK(delay->from()==time);
          CHECK(delay->duration()==5s);
          
        }

        WHEN("Trajectory is replaced")
        {
          rmf_traffic::Trajectory t2("test_map");
          t2.insert(time+5s,profile, Eigen::Vector3d{0,1,0},Eigen::Vector3d{0,0,0});
          t2.insert(time+5s,profile, Eigen::Vector3d{0,1,0},Eigen::Vector3d{0,0,0});
          REQUIRE(t2.size()==2);

          rmf_traffic::schedule::Version version2= db.replace(1,t2);
          CHECK(version2==2);
          changes=db.changes(query_everything);
          REQUIRE(changes.size()==2);
          REQUIRE(changes.latest_version()==2);
          auto replace_change= changes.end();
          REQUIRE(static_cast<int>(replace_change->get_mode())==4);
          REQUIRE(replace_change->replace()!=nullptr);
          auto replace=replace_change->replace();
          CHECK(replace->original_id()==1);
          REQUIRE(replace->trajectory()!=nullptr);

          CHECK(replace->trajectory()==&t2); //CHANGE

          

        }

        WHEN("Trajectory is erased")
        {

          rmf_traffic::schedule::Version version2= db.erase(1);
          CHECK(version2==2);
          changes=db.changes(query_everything);
          REQUIRE(changes.size()==2);
          REQUIRE(changes.latest_version()==2);
          auto erase_change= changes.end();
          REQUIRE(static_cast<int>(erase_change->get_mode())==5);
          REQUIRE(erase_change->erase()!=nullptr);
          auto erase=erase_change->erase();
          CHECK(erase->original_id()==1);
            
        }
        WHEN("Trajectory is culled")
        {
          auto cull_time=time+20s;
          rmf_traffic::schedule::Version version2= db.cull(cull_time);
          CHECK(version2==2);
          changes=db.changes(query_everything);
          REQUIRE(changes.size()==2);
          REQUIRE(changes.latest_version()==2);
          auto cull_change= changes.end();
          REQUIRE(static_cast<int>(cull_change->get_mode())==6);
          REQUIRE(cull_change->cull()!=nullptr);
          auto cull=cull_change->cull();
          CHECK(cull->time()==cull_time);
            
        }




    }




}