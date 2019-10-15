#include<iostream>
#include <rmf_traffic/geometry/Box.hpp>
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


  std::cout<<"Testing spacetime\n";
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();


  //creating space to add to region
  const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);

  std::chrono::steady_clock::time_point* lower_time_bound;
  std::chrono::steady_clock::time_point* upper_time_bound;
  *lower_time_bound=time;
  *upper_time_bound=time+10s;


  rmf_traffic::internal::Spacetime region={
    lower_time_bound,
    upper_time_bound,
    tf,
    final_box
  };


  //creating trajectory
  rmf_traffic::geometry::Box shape(1,1);
  rmf_traffic::geometry::FinalConvexShapePtr final_shape= rmf_traffic::geometry::make_final_convex(shape);
  rmf_traffic::Trajectory::ProfilePtr profile = rmf_traffic::Trajectory::Profile::make_strict(final_shape);
  rmf_traffic::Trajectory t1("test_map");

  t1.insert(time, profile, Eigen::Vector3d{-5,0,0},Eigen::Vector3d{5,0,0});
  t1.insert(time+10s, profile, Eigen::Vector3d{5,0,0},Eigen::Vector3d{5,0,0});
  REQUIRE(t1.size()==2);


  std::vector<rmf_traffic::Trajectory::const_iterator>* output_iterators;
  REQUIRE(rmf_traffic::internal::detect_conflicts(t1,region,output_iterators));



}
