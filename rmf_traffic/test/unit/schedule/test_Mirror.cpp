

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
rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
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
REQUIRE(rmf_traffic::DetectConflict::broad_phase(t1,t2));
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
