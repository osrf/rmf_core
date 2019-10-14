

#include "utils_Database.hpp"
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/geometry/Box.hpp>

#include "src/rmf_traffic/schedule/debug_Viewer.hpp"

#include <rmf_utils/catch.hpp>
#include<iostream>
using namespace std::chrono_literals;
#include <rmf_traffic/schedule/Mirror.hpp>

SCENARIO("Test Mirror given a Database with two non-conflicting trajectories")
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

GIVEN("A Mirror m1 of the Database db")
    {

        rmf_traffic::schedule::Mirror m1;
        //updating mirror
        changes=db.changes(query_everything);
        auto version=m1.update(changes);
        REQUIRE(version==version_2);
        CHECK(m1.oldest_version()==0);
        CHECK(m1.latest_version()==version_2);





        


        
    }


}
