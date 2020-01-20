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

#include "utils_Database.hpp"
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/geometry/Box.hpp>

#include "src/rmf_traffic/schedule/debug_Viewer.hpp"

#include <rmf_utils/catch.hpp>
#include<iostream>
using namespace std::chrono_literals;
#include <rmf_traffic/schedule/Mirror.hpp>
#include<rmf_traffic/Conflict.hpp>

SCENARIO("Test Mirror of a Database with two trajectories")
{

//Creating database db and checking for empty initialziation 
rmf_traffic::schedule::Database db;
const rmf_traffic::schedule::Query query_everything= rmf_traffic::schedule::query_everything();
rmf_traffic::schedule::Database::Patch changes= db.changes(query_everything);
REQUIRE(changes.size()==0);

//Creating Trajectories to insert
const rmf_traffic::Time time = std::chrono::steady_clock::now();
double profile_scale=1;
rmf_traffic::geometry::Box shape(profile_scale,profile_scale);
rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
rmf_traffic::Trajectory t1("test_map");
t1.insert(time, profile, Eigen::Vector3d{-5,0,0}, Eigen::Vector3d{0,0,0});
t1.insert(time + 10s, profile, Eigen::Vector3d{5,0,0}, Eigen::Vector3d{0,0,0});
REQUIRE(t1.size()==2);


rmf_traffic::Trajectory t2("test_map");
t2.insert(time, profile, Eigen::Vector3d{-5,10,0}, Eigen::Vector3d{0,0,0});
t2.insert(time+10s,profile, Eigen::Vector3d{5,10,0},Eigen::Vector3d{0,0,0});
REQUIRE(t2.size()==2);

const auto version_1=db.insert(t1);
REQUIRE(version_1==1);
const auto version_2= db.insert(t2);
REQUIRE(version_2==2);
REQUIRE_FALSE(rmf_traffic::DetectConflict::broad_phase(t1,t2));
REQUIRE(rmf_traffic::DetectConflict::narrow_phase(t1,t2).size()==0);

rmf_traffic::schedule::Mirror m1;
//updating mirror
changes=db.changes(query_everything);
auto version=m1.update(changes);
REQUIRE(version==version_2);
CHECK(m1.oldest_version()==0);
CHECK(m1.latest_version()==version_2);



  WHEN("A non-conflicting trajectory is added to db")
        {   

          rmf_traffic::Trajectory t3("test_map");
          t3.insert(time,profile,Eigen::Vector3d{-5,-10,0},Eigen::Vector3d{0,0,0});
          t3.insert(time+10s,profile,Eigen::Vector3d{5,10,0},Eigen::Vector3d{0,0,0});
          const auto version_3=db.insert(t3);
          REQUIRE(version_3==3);
          CHECK_TRAJECTORY_COUNT(db,3);

          THEN("m1 latest version is different from db")
          {
            CHECK(m1.latest_version()!=db.latest_version());
            CHECK(m1.oldest_version()==db.oldest_version());

          }

          THEN("Updating the mirror should update its latest version")
          {
            rmf_traffic::schedule::Query query=rmf_traffic::schedule::make_query(version_2);
            changes=db.changes(query);
            m1.update(changes);
            CHECK(m1.latest_version()==db.latest_version());

            const auto view=m1.query(rmf_traffic::schedule::query_everything());
            //check contents of view


          }
            
        }

  GIVEN ("A trajctory to be submitted to the scheduler that conflicts with the mirror's view")
  {

    rmf_traffic::Trajectory t3("test_map");
    t3.insert(time,profile,Eigen::Vector3d{0,-5,0},Eigen::Vector3d{0,0,0});
    t3.insert(time+10s,profile,Eigen::Vector3d{0,5,0},Eigen::Vector3d{0,0,0});
    auto view=m1.query(query_everything);
    auto collision_trajectories=get_collision_trajectories(view,t3);
    CHECK(collision_trajectories.size()==1);


      WHEN("Replacing conflicting trajectory in db and updating mirror should eliminate conflict")
      {
        rmf_traffic::Trajectory t4("test_map");
        t4.insert(time,profile,Eigen::Vector3d{-5,0,0},Eigen::Vector3d{0,0,0});
        t4.insert(time+10s,profile,Eigen::Vector3d{-2,0,0},Eigen::Vector3d{0,0,0});
        db.replace(version_1,t4);
        changes= db.changes(rmf_traffic::schedule::make_query(db.latest_version()-1));
        m1.update(changes);
        CHECK(db.latest_version()==m1.latest_version());
        view=m1.query(query_everything);
        collision_trajectories=get_collision_trajectories(view,t3);
        CHECK(collision_trajectories.size()==0);
      }

      WHEN("Erasing conflicting trajectory in db and updating mirror should eliminate conflict")
      {

        db.erase(version_1);
        changes= db.changes(rmf_traffic::schedule::make_query(db.latest_version()-1));
        m1.update(changes);
        CHECK(db.latest_version()==m1.latest_version());
        view=m1.query(query_everything);
        collision_trajectories=get_collision_trajectories(view,t3);
        CHECK(collision_trajectories.size()==0);
      }

      WHEN("Delaying conflicting trajectory in db and updating mirror should eliminate conflict")

      {
        db.delay(version_1,time,20s); //this should delay the first trajecotory enough to avoid collision
        changes= db.changes(rmf_traffic::schedule::make_query(db.latest_version()-1));
        m1.update(changes);
        CHECK(db.latest_version()==m1.latest_version());
        view=m1.query(query_everything);
        collision_trajectories=get_collision_trajectories(view,t3);
        CHECK(collision_trajectories.size()==0);

       }


      WHEN("Interrupting conflicting trajectory in db and updating mirror should eliminate conflict")

      {

        rmf_traffic::Trajectory t4("test_map");
        t4.insert(time+2s,profile,Eigen::Vector3d{-5,10,0},Eigen::Vector3d{0,0,0});
        db.interrupt(version_1,t4,0s);
        changes= db.changes(rmf_traffic::schedule::make_query(db.latest_version()-1));
        m1.update(changes);
        CHECK(db.latest_version()==m1.latest_version());
        view=m1.query(query_everything);
        collision_trajectories=get_collision_trajectories(view,t3);
        CHECK(collision_trajectories.size()==0);

       }

      WHEN("Culling conflicting trajectory in db and updating mirror should eliminate conflict")

      {
        db.cull(time+11s);
        changes= db.changes(rmf_traffic::schedule::make_query(db.latest_version()-1));
        m1.update(changes);
        CHECK(db.latest_version()==m1.latest_version());
        view=m1.query(query_everything);
        collision_trajectories=get_collision_trajectories(view,t3);
        CHECK(collision_trajectories.size()==0);

       }

  }

}

SCENARIO("Testing specialized mirrors")

{

  rmf_traffic::schedule::Database db;
  const rmf_traffic::schedule::Query query_everything= rmf_traffic::schedule::query_everything();
  rmf_traffic::schedule::Database::Patch changes= db.changes(query_everything);
  REQUIRE(changes.size()==0);

  //Creating trajectories t1, t2, t3 in "test_map"
  const rmf_traffic::Time time = std::chrono::steady_clock::now();
  double profile_scale=1;
  rmf_traffic::geometry::Box shape(profile_scale,profile_scale);
  rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
  rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);

  rmf_traffic::Trajectory t1("test_map");
  t1.insert(time, profile, Eigen::Vector3d{-5,0,0}, Eigen::Vector3d{0,0,0});
  t1.insert(time + 10s, profile, Eigen::Vector3d{5,0,0}, Eigen::Vector3d{0,0,0});
  REQUIRE(t1.size()==2);


  rmf_traffic::Trajectory t2("test_map");
  t2.insert(time, profile, Eigen::Vector3d{-5,10,0}, Eigen::Vector3d{0,0,0});
  t2.insert(time+11s,profile, Eigen::Vector3d{5,10,0},Eigen::Vector3d{0,0,0});
  REQUIRE(t2.size()==2);

  rmf_traffic::Trajectory t3("test_map");
  t3.insert(time+11s, profile, Eigen::Vector3d{0,-5,0}, Eigen::Vector3d{0,0,0});
  t3.insert(time+20s,profile, Eigen::Vector3d{0,5,0},Eigen::Vector3d{0,0,0});
  REQUIRE(t3.size()==2);


  //creating trajectories t4 and t5 in "test_map_2"

  rmf_traffic::Trajectory t4("test_map_2");
  t4.insert(time, profile, Eigen::Vector3d{-5,0,0}, Eigen::Vector3d{0,0,0});
  t4.insert(time + 10s, profile, Eigen::Vector3d{5,0,0}, Eigen::Vector3d{0,0,0});
  REQUIRE(t4.size()==2);


  rmf_traffic::Trajectory t5("test_map_2");
  t5.insert(time, profile, Eigen::Vector3d{-5,10,0}, Eigen::Vector3d{0,0,0});
  t5.insert(time+10s,profile, Eigen::Vector3d{5,10,0},Eigen::Vector3d{0,0,0});
  REQUIRE(t5.size()==2);

  const auto version_1=db.insert(t1);
  REQUIRE(version_1==1);
  const auto version_2= db.insert(t2);
  REQUIRE(version_2==2);
  const auto version_3=db.insert(t3);
  REQUIRE(version_3==3);
  const auto version_4= db.insert(t4);
  REQUIRE(version_4==4);
  const auto version_5=db.insert(t5);
  REQUIRE(version_5==5);

REQUIRE(rmf_traffic::DetectConflict::between(t1,t2).size()==0);
REQUIRE(rmf_traffic::DetectConflict::between(t1,t3).size()==0);
REQUIRE(rmf_traffic::DetectConflict::between(t1,t4).size()==0);
REQUIRE(rmf_traffic::DetectConflict::between(t1,t5).size()==0);
REQUIRE(rmf_traffic::DetectConflict::between(t2,t3).size()==0);
REQUIRE(rmf_traffic::DetectConflict::between(t2,t4).size()==0);
REQUIRE(rmf_traffic::DetectConflict::between(t2,t5).size()==0);
REQUIRE(rmf_traffic::DetectConflict::between(t3,t4).size()==0);
REQUIRE(rmf_traffic::DetectConflict::between(t4,t5).size()==0);

GIVEN("Query patch with spacetime region overlapping with t1")
  {
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  rmf_traffic::schedule::Query query= rmf_traffic::schedule::make_query(0,{});
  REQUIRE(query.spacetime().regions() != nullptr);

  //creating space to add to region
  const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  rmf_traffic::geometry::Space space(final_box,tf);
  std::vector<rmf_traffic::geometry::Space> spaces;
  spaces.push_back(space);
  //creating a region with time bounds and spaces that overlap with trajectory
  rmf_traffic::Region region("test_map",time, time+10s,spaces);
  query.spacetime().regions()->push_back(region);

  rmf_traffic::schedule::Database::Patch changes= db.changes(query);
  REQUIRE(changes.size()>0);
  CHECK(changes.size()==1);
  CHECK(changes.begin()->id()==1);

  }

  GIVEN("Query patch with spacetime region overlapping with t2")
  {
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  rmf_traffic::schedule::Query query= rmf_traffic::schedule::make_query(0,{});
  REQUIRE(query.spacetime().regions() != nullptr);

  //creating space to add to region
  const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  tf.translate(Eigen::Vector2d{0.0,10.0});
  rmf_traffic::geometry::Space space(final_box,tf);
  std::vector<rmf_traffic::geometry::Space> spaces;
  spaces.push_back(space);
  //creating a region with time bounds and spaces that overlap with trajectory
  rmf_traffic::Region region("test_map",time, time+10s,spaces);
  query.spacetime().regions()->push_back(region);

  //querying for Patch using defined spacetime query
  rmf_traffic::schedule::Database::Patch changes= db.changes(query);
  REQUIRE(changes.size()>0);
  CHECK(changes.size()==1);
  CHECK(changes.begin()->id()==2);


  }


/*
COMMENTED DUE TO NON-DETERMINISTIC BEHAVIOR OF FCL

  GIVEN("Query patch with rotated spacetime region overlapping with t1")
  {
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  rmf_traffic::schedule::Query query= rmf_traffic::schedule::make_query(0,{});
  REQUIRE(query.spacetime().regions() != nullptr);

  //creating space to add to region
  const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  tf.rotate(Eigen::Rotation2Dd(M_PI_2));
  rmf_traffic::geometry::Space space(final_box,tf);
  std::vector<rmf_traffic::geometry::Space> spaces;
  spaces.push_back(space);
  //creating a region with time bounds and spaces that overlap with trajectory
  rmf_traffic::Region region("test_map",time, time+10s,spaces);
  query.spacetime().regions()->push_back(region);

  //querying for Patch using defined spacetime query
  rmf_traffic::schedule::Database::Patch changes= db.changes(query);
  REQUIRE(changes.size() >0);
  CHECK(changes.size() == 1);
  CHECK(changes.begin()->id() == 1);

  }
*/


  GIVEN("Query patch with spacetime region overlapping with t3")
  {
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  rmf_traffic::schedule::Query query= rmf_traffic::schedule::make_query(0,{});
  REQUIRE(query.spacetime().regions() != nullptr);

  //creating space to add to region
  const auto box = rmf_traffic::geometry::Box(1.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  rmf_traffic::geometry::Space space(final_box,tf);
  std::vector<rmf_traffic::geometry::Space> spaces;
  spaces.push_back(space);

  //creating a region with time bounds and spaces that overlap with trajectory
  rmf_traffic::Region region("test_map",time+10s, time+20s,spaces);
  query.spacetime().regions()->push_back(region);

  //querying for Patch using defined spacetime query
  rmf_traffic::schedule::Database::Patch changes= db.changes(query);
  REQUIRE(changes.size() >0);
  CHECK(changes.size()==1);
  CHECK(changes.begin()->id()==3);


  }



  GIVEN("Query patch with spacetime region overlapping with t1 and t3")
  {
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  rmf_traffic::schedule::Query query= rmf_traffic::schedule::make_query(0,{});
  REQUIRE(query.spacetime().regions() != nullptr);

  //creating space to add to region
  const auto box = rmf_traffic::geometry::Box(10, 10);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  rmf_traffic::geometry::Space space(final_box,tf);
  std::vector<rmf_traffic::geometry::Space> spaces;
  spaces.push_back(space);

  //creating a region with time bounds and spaces that overlap with trajectory
  rmf_traffic::Region region("test_map",time, time+20s,spaces);
  query.spacetime().regions()->push_back(region);

  //querying for Patch using defined spacetime query
  rmf_traffic::schedule::Database::Patch changes= db.changes(query);

  // Only t1 and t3 are within range of the region. t2 runs along the line y=10,
  // which is outside the range of a 10x10 box that is centered at the origin,
  // because that box will only reach out to y=5.
  REQUIRE(changes.size() >0);
  CHECK(changes.size()==2);
  CHECK(changes.begin()->id()==1);
  CHECK((++changes.begin())->id()==3);
  }



  GIVEN("Query patch with spacetime region overlapping with t1, t2 & t3")
  {
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  rmf_traffic::schedule::Query query= rmf_traffic::schedule::make_query(0,{});
  REQUIRE(query.spacetime().regions() != nullptr);

  //creating space to add to region
  const auto box = rmf_traffic::geometry::Box(10, 20);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  rmf_traffic::geometry::Space space(final_box,tf);
  std::vector<rmf_traffic::geometry::Space> spaces;
  spaces.push_back(space);

  //creating a region with time bounds and spaces that overlap with trajectory
  rmf_traffic::Region region("test_map",time, time+20s,spaces);
  query.spacetime().regions()->push_back(region);

  //querying for Patch using defined spacetime query
  rmf_traffic::schedule::Database::Patch changes= db.changes(query);
  REQUIRE(changes.size() >0);
  CHECK(changes.size()==3);
  CHECK(changes.begin()->id()==1);
  CHECK((++changes.begin())->id()==2);
  CHECK((--changes.end())->id()==3);

  }


}
