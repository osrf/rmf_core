#include<iostream>
#include <rmf_traffic/geometry/Box.hpp>
#include<rmf_traffic/geometry/Circle.hpp>
#include<rmf_traffic/Conflict.hpp>
#include<rmf_traffic/schedule/Mirror.hpp>
#include<rmf_traffic/schedule/Query.hpp>
#include "utils_Database.hpp"
#include <rmf_traffic/schedule/Database.hpp>
#include "src/rmf_traffic/DetectConflictInternal.hpp"
#include <rmf_utils/catch.hpp>
#include<rmf_traffic/Trajectory.hpp>




SCENARIO("Testing intersaection of spacetime with trajectories")
{
  using namespace std::chrono_literals;
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  //creating space to create spacetime
  const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);

  std::chrono::_V2::steady_clock::time_point lower_time_bound=time;
  std::chrono::_V2::steady_clock::time_point upper_time_bound=time+10s;

  rmf_traffic::internal::Spacetime region={
    &lower_time_bound,
    &upper_time_bound,
    tf,
    final_box
  };


  GIVEN("A box trajectory that does not intersect with the spacetime region")
{

    //creating trajectory
  rmf_traffic::geometry::Box shape(1,1);
  rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
  rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
  rmf_traffic::Trajectory t1("test_map");

  t1.insert(time, profile, Eigen::Vector3d{-5,10,0},Eigen::Vector3d{5,0,0});
  t1.insert(time+10s, profile, Eigen::Vector3d{5,10,0},Eigen::Vector3d{5,0,0});
  REQUIRE(t1.size()==2);


  std::vector<rmf_traffic::Trajectory::const_iterator> output_iterators;
  CHECK_FALSE(rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators));
}

  GIVEN("A box trajectory that completely intersects with the spacetime region")
  {
  //creating trajectory
  rmf_traffic::geometry::Box shape(1,1);
  rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
  rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
  rmf_traffic::Trajectory t1("test_map");

  t1.insert(time, profile, Eigen::Vector3d{-5,0,0},Eigen::Vector3d{5,0,0});
  t1.insert(time+10s, profile, Eigen::Vector3d{5,0,0},Eigen::Vector3d{5,0,0});
  REQUIRE(t1.size()==2);


  std::vector<rmf_traffic::Trajectory::const_iterator> output_iterators;
  CHECK(rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators));
  }

  GIVEN("A box trajectory that partially intersects with the spacetime region")
  {
  //creating trajectory
  rmf_traffic::geometry::Box shape(1,1);
  rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
  rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
  rmf_traffic::Trajectory t1("test_map");

  t1.insert(time, profile, Eigen::Vector3d{-1,0,0},Eigen::Vector3d{5,0,0});
  t1.insert(time+10s, profile, Eigen::Vector3d{1,0,0},Eigen::Vector3d{5,0,0});
  REQUIRE(t1.size()==2);


  std::vector<rmf_traffic::Trajectory::const_iterator> output_iterators;
  CHECK(rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators));
  }


  GIVEN("A circular trajectory that partially intersects with the spacetime region")
  {
  //creating trajectory
  rmf_traffic::geometry::Circle shape(1); //radius 1m
  rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
  rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
  rmf_traffic::Trajectory t1("test_map");

  t1.insert(time, profile, Eigen::Vector3d{-1,-1.5,0},Eigen::Vector3d{5,0,0});
  t1.insert(time+10s, profile, Eigen::Vector3d{1,-1.5,0},Eigen::Vector3d{5,0,0});
  REQUIRE(t1.size()==2);


  std::vector<rmf_traffic::Trajectory::const_iterator> output_iterators;
  CHECK(rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators));
  }

  GIVEN("A circular trajectory that touches the spacetime region")
  {
  //creating trajectory
  rmf_traffic::geometry::Circle shape(1); //radius 1m
  rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
  rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
  rmf_traffic::Trajectory t1("test_map");

  t1.insert(time, profile, Eigen::Vector3d{-1,-2,0},Eigen::Vector3d{5,0,0});
  t1.insert(time+10s, profile, Eigen::Vector3d{1,-2,0},Eigen::Vector3d{5,0,0});
  REQUIRE(t1.size()==2);
  std::vector<rmf_traffic::Trajectory::const_iterator> output_iterators;
  CHECK_FALSE(rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators));
  }



}


SCENARIO("Testing intersection of curved trajectory with various spacetimes")
{
  using namespace std::chrono_literals;
  auto time= std::chrono::steady_clock::now();
  rmf_traffic::Trajectory t1("test_map");
  rmf_traffic::geometry::Circle shape(1); //radius 1m
  rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
  rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
  
  t1.insert(time,profile,Eigen::Vector3d{-5,0,0},Eigen::Vector3d{1,0,0});
  t1.insert(time+10s,profile,Eigen::Vector3d{0,-5,0},Eigen::Vector3d{0,-1,0});


  GIVEN("spacetime that encompasses the trajectory in space and time")
  {

    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
    //creating space to create spacetime
    const auto box = rmf_traffic::geometry::Box(10.0, 10.0);
    const auto final_box = rmf_traffic::geometry::make_final_convex(box);

    std::chrono::_V2::steady_clock::time_point lower_time_bound=time;
    std::chrono::_V2::steady_clock::time_point upper_time_bound=time+10s;

    rmf_traffic::internal::Spacetime region={
      &lower_time_bound,
      &upper_time_bound,
      tf,
      final_box
    };
    std::vector<rmf_traffic::Trajectory::const_iterator> output_iterators;

    CHECK(rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators));


  }

  GIVEN("spacetime that encompasses the trajectory in space but not time")
  {

    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
    //creating space to create spacetime
    const auto box = rmf_traffic::geometry::Box(10.0, 10.0);
    const auto final_box = rmf_traffic::geometry::make_final_convex(box);

    std::chrono::_V2::steady_clock::time_point lower_time_bound=time+20s;
    std::chrono::_V2::steady_clock::time_point upper_time_bound=time+30s;

    rmf_traffic::internal::Spacetime region={
      &lower_time_bound,
      &upper_time_bound,
      tf,
      final_box
    };
    std::vector<rmf_traffic::Trajectory::const_iterator> output_iterators;

    CHECK_FALSE(rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators));


  }


  GIVEN("spacetime that partially intersects the trajectory in space and time")
  {

  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  //creating space to create spacetime
  const auto box = rmf_traffic::geometry::Box(2, 2);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);

  std::chrono::_V2::steady_clock::time_point lower_time_bound=time;
  std::chrono::_V2::steady_clock::time_point upper_time_bound=time+10s;
  //tf.translate(Eigen::Translation2d(-2,-2));
  rmf_traffic::internal::Spacetime region={
    &lower_time_bound,
    &upper_time_bound,
    tf,
    final_box
  };
  std::vector<rmf_traffic::Trajectory::const_iterator> output_iterators;

  CHECK(rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators));


  }

  






}
