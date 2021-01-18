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

#include <iostream>

//#include <rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/geometry/Box.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>
#include <rmf_traffic/schedule/Query.hpp>
#include "utils_Database.hpp"
#include <rmf_traffic/schedule/Database.hpp>
#include "src/rmf_traffic/DetectConflictInternal.hpp"
#include <rmf_utils/catch.hpp>
#include <rmf_traffic/Trajectory.hpp>


SCENARIO("Testing intersection of spacetime with trajectories")
{
  using namespace std::chrono_literals;
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  // creating space to create spacetime
  const auto region_box_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(10.0, 1.0);

  std::chrono::_V2::steady_clock::time_point lower_time_bound = time;
  std::chrono::_V2::steady_clock::time_point upper_time_bound = time+10s;

  rmf_traffic::internal::Spacetime region = {
    &lower_time_bound,
    &upper_time_bound,
    tf,
    region_box_shape
  };

  const auto box_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(1, 1);
  rmf_traffic::Profile box{box_shape};

  const auto circle_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  rmf_traffic::Profile circle{circle_shape};

  GIVEN("A box trajectory that does not intersect with the spacetime region")
  {
    rmf_traffic::Trajectory t1;
    t1.insert(time, {-5, 10, 0}, {5, 0, 0});
    t1.insert(time+10s, {5, 10, 0}, {5, 0, 0});
    REQUIRE(t1.size() == 2);

    CHECK_FALSE(rmf_traffic::internal::detect_conflicts(box, t1, region));
  }

  GIVEN("A box trajectory that completely intersects with the spacetime region")
  {
    rmf_traffic::Trajectory t1;
    t1.insert(time, {-5, 0, 0}, {5, 0, 0});
    t1.insert(time+10s, {5, 0, 0}, {5, 0, 0});
    REQUIRE(t1.size() == 2);

    CHECK(rmf_traffic::internal::detect_conflicts(box, t1, region));
  }

  GIVEN("A box trajectory that partially intersects with the spacetime region")
  {
    rmf_traffic::Trajectory t1;
    t1.insert(time, {-1, 0, 0}, {5, 0, 0});
    t1.insert(time+10s, {1, 0, 0}, {5, 0, 0});
    REQUIRE(t1.size() == 2);

    CHECK(rmf_traffic::internal::detect_conflicts(box, t1, region));
  }

  GIVEN(
    "A circular trajectory that partially intersects with the spacetime region")
  {
    rmf_traffic::Trajectory t1;
    t1.insert(time, {-1, -1.5, 0}, {5, 0, 0});
    t1.insert(time+10s, {1, -1.5, 0}, {5, 0, 0});
    REQUIRE(t1.size() == 2);

    CHECK(rmf_traffic::internal::detect_conflicts(circle, t1, region));
  }

  GIVEN("A circular trajectory that touches the spacetime region")
  {
    rmf_traffic::Trajectory t1;
    t1.insert(time, {-1, -2, 0}, {5, 0, 0});
    t1.insert(time+10s, {1, -2, 0}, {5, 0, 0});
    REQUIRE(t1.size() == 2);

    CHECK_FALSE(rmf_traffic::internal::detect_conflicts(circle, t1, region));
  }
}


SCENARIO("Testing intersction of various spacetimes and trajectories")
{
  using namespace std::chrono_literals;
  auto time = std::chrono::steady_clock::now();
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
  std::chrono::_V2::steady_clock::time_point lower_time_bound;
  std::chrono::_V2::steady_clock::time_point upper_time_bound;

  const auto region_box_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(10.0, 1.0);

  const auto double_region_box_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(10.0, 2.0);

  const auto vertical_region_box_shape =
    rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(1.0, 10.0);

  const auto double_vertical_region_box_shape =
    rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(2.0, 10.0);

  const auto unit_box_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(1, 1);
  rmf_traffic::Profile unit_box{unit_box_shape};

  const auto double_box_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(2.0, 2.0);
  rmf_traffic::Profile double_box{double_box_shape};

  const auto unit_circle_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  rmf_traffic::Profile unit_circle{unit_circle_shape};

  const auto small_circle_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.5);
  rmf_traffic::Profile small_circle{small_circle_shape};

  GIVEN(
    "A trajectory along X-Axis and spacetime region that completely overlaps")
  {
    // Creating trajectory
    rmf_traffic::Trajectory t1;
    t1.insert(time, {-5, 0, 0}, {0, 0, 0});
    t1.insert(time + 10s, {5, 0, 0}, {0, 0, 0});
    REQUIRE(t1.size() == 2);

    // Creating Spacetime
    lower_time_bound = time;
    upper_time_bound = time+10s;

    rmf_traffic::internal::Spacetime region{
      &lower_time_bound,
      &upper_time_bound,
      tf,
      region_box_shape
    };

    rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
    CHECK(rmf_traffic::internal::detect_conflicts(
        unit_box, t1, region, &conflicts));
    CHECK(conflicts.size() == 1);
  }

  GIVEN("A trajectory along y=10 and spacetime region that completely overlaps")
  {
    // trajectory timing is varied

    // Creating trajectory
    rmf_traffic::Trajectory t1;
    t1.insert(time, {-5, 10, 0}, {0, 0, 0});
    t1.insert(time + 10s, {5, 10, 0}, {0, 0, 0});
    REQUIRE(t1.size() == 2);

    // Creating Spacetime
    lower_time_bound = time;
    upper_time_bound = time+10s;

    tf.translate(Eigen::Vector2d{0.0, 10.0});

    rmf_traffic::internal::Spacetime region{
      &lower_time_bound,
      &upper_time_bound,
      tf,
      region_box_shape
    };

    rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
    CHECK(rmf_traffic::internal::detect_conflicts(
        unit_box, t1, region, &conflicts));
    CHECK(conflicts.size() == 1);
  }

  GIVEN(
    "A trajectory along Y-Axis and spacetime region that completely overlaps")
  {
    // trajectory timing is varied

    // Creating trajectory
    rmf_traffic::Trajectory t1;
    t1.insert(time+11s, {0, -5, 0}, {0, 0, 0});
    t1.insert(time+20s, {0, 5, 0}, {0, 0, 0});
    REQUIRE(t1.size() == 2);

    // Creating Spacetime
    lower_time_bound = time+10s;
    upper_time_bound = time+20s;

    tf.rotate(Eigen::Rotation2Dd(M_PI_2));

    rmf_traffic::internal::Spacetime region = {
      &lower_time_bound,
      &upper_time_bound,
      tf,
      region_box_shape
    };

    rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
    CHECK(rmf_traffic::internal::detect_conflicts(
        unit_box, t1, region, &conflicts));
    CHECK(conflicts.size() == 1);
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  GIVEN("A trajectory along X-Axis and spacetime region roated at the origin")
  {
    bool rotate = false;
    double rot_ang = 0;

    /* WHEN("Trajectory profile is 1x1, space is 10x1, space box rotated by 90deg ") //this fails
     {

     //Creating trajectory
     rmf_traffic::Trajectory t1("test_map");
     rmf_traffic::geometry::Box shape(1.0,1.0);
     rmf_traffic::geometry::ConstFinalConvexShapePtr final_shape =rmf_traffic::geometry::make_final_convex(shape);
     auto profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
     t1.insert(time, profile, Eigen::Vector3d{-5.0,0,0}, Eigen::Vector3d{0,0,0});
     t1.insert(time+10s, profile, Eigen::Vector3d{5.0,0,0}, Eigen::Vector3d{0,0,0});
     REQUIRE(t1.size()==2);

     //Creating Spacetime

     const auto box = rmf_traffic::geometry::Box(10.0, 1.0); //Centered at (0,0) with width 10m and height 1m
     const auto final_box = rmf_traffic::geometry::make_final_convex(box);
     lower_time_bound=time;
     upper_time_bound=time+10s;

     rotate=true;
     rot_ang=90;
     if(rotate)
         tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg

     rmf_traffic::internal::Spacetime region={
       &lower_time_bound,
       &upper_time_bound,
       tf,
       final_box
     };
     // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
     // std::cout<<" Rot:"<<rotate<<" ";
     // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

     bool conflict= rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators);
     // CHECK(conflict);
     // CHECK(output_iterators.size()==1);
     }

     WHEN("Trajectory profile is 1x1, space is 10x1, space box rotated by 69deg ") //this fails
     {

     //Creating trajectory
     rmf_traffic::Trajectory t1("test_map");
     rmf_traffic::geometry::Box shape(1.0,1.0);
     rmf_traffic::geometry::ConstFinalConvexShapePtr final_shape =rmf_traffic::geometry::make_final_convex(shape);
     auto profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
     t1.insert(time, profile, Eigen::Vector3d{-5.0,0,0}, Eigen::Vector3d{0,0,0});
     t1.insert(time+10s, profile, Eigen::Vector3d{5.0,0,0}, Eigen::Vector3d{0,0,0});
     REQUIRE(t1.size()==2);

     //Creating Spacetime

     const auto box = rmf_traffic::geometry::Box(10.0, 1.0); //Centered at (0,0) with width 10m and height 1m
     const auto final_box = rmf_traffic::geometry::make_final_convex(box);
     lower_time_bound=time;
     upper_time_bound=time+10s;

     rotate=true;
     rot_ang=69;
     if(rotate)
         tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg

     //print out the translation compotent of tf
     // for(int i=0;i<2;i++)
     //   std::cout<<"Translation "<<i<<":"<<tf.translation()[i]<<std::endl;


     rmf_traffic::internal::Spacetime region={
       &lower_time_bound,
       &upper_time_bound,
       tf,
       final_box
     };
     std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
     std::cout<<" Rot:"<<rotate<<" ";
     if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

     bool conflict= rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators);
     CHECK(conflict);
     CHECK(output_iterators.size()==1);
     }*/

    WHEN("Trajectory profile is 1x1, space is 10x1, space box rotated by 68deg ") //this passes
    {
      //Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      //Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      rotate = true;
      rot_ang = 68;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      //print out the translation compotent of tf
      // for(int i=0;i<2;i++)
      //   std::cout<<"Translation "<<i<<":"<<tf.translation()[i]<<std::endl;


      rmf_traffic::internal::Spacetime region{
        &lower_time_bound,
        &upper_time_bound,
        tf,
        region_box_shape
      };
      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    /*WHEN("Trajectory profile is 1x1, space is 10x2, space box rotated by 90deg ") //this fails
    {

    //Creating trajectory
    rmf_traffic::Trajectory t1("test_map");
    rmf_traffic::geometry::Box shape(1.0,1.0);
    rmf_traffic::geometry::ConstFinalConvexShapePtr final_shape =rmf_traffic::geometry::make_final_convex(shape);
    auto profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
    t1.insert(time, profile, Eigen::Vector3d{-5.0,0,0}, Eigen::Vector3d{0,0,0});
    t1.insert(time+10s, profile, Eigen::Vector3d{5.0,0,0}, Eigen::Vector3d{0,0,0});
    REQUIRE(t1.size()==2);

    //Creating Spacetime

    const auto box = rmf_traffic::geometry::Box(10.0, 2.0); //Centered at (0,0) with width 10m and height 1m
    const auto final_box = rmf_traffic::geometry::make_final_convex(box);
    lower_time_bound=time;
    upper_time_bound=time+10s;

    rotate=true;
    rot_ang=90;
    if(rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg


    rmf_traffic::internal::Spacetime region={
      &lower_time_bound,
      &upper_time_bound,
      tf,
      final_box
    };

    // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
    // std::cout<<" Rot:"<<rotate<<" ";
    // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

    bool conflict= rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators);
    CHECK(conflict);
    CHECK(output_iterators.size()==1);
    }

    WHEN("Trajectory profile is 1x1, space is 10x2, space box rotated by 75deg")
    {

    //Creating trajectory
    rmf_traffic::Trajectory t1("test_map");
    rmf_traffic::geometry::Box shape(1.0,1.0);
    rmf_traffic::geometry::ConstFinalConvexShapePtr final_shape =rmf_traffic::geometry::make_final_convex(shape);
    auto profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
    t1.insert(time, profile, Eigen::Vector3d{-5.0,0,0}, Eigen::Vector3d{0,0,0});
    t1.insert(time+10s, profile, Eigen::Vector3d{5.0,0,0}, Eigen::Vector3d{0,0,0});
    REQUIRE(t1.size()==2);

    //Creating Spacetime

    const auto box = rmf_traffic::geometry::Box(10.0, 2.0); //Centered at (0,0) with width 10m and height 1m
    const auto final_box = rmf_traffic::geometry::make_final_convex(box);
    lower_time_bound=time;
    upper_time_bound=time+10s;

    rotate=true;
    rot_ang=75;
    if(rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg



    rmf_traffic::internal::Spacetime region={
      &lower_time_bound,
      &upper_time_bound,
      tf,
      final_box
    };

    std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
    std::cout<<" Rot:"<<rotate<<" ";
    if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

    bool conflict= rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators);
    CHECK(conflict);
    CHECK(output_iterators.size()==1);
    }*/

#ifdef RMF_TRAFFIC__USING_FCL_0_6
    WHEN("Trajectory profile is 1x1, space is 10x2, space box rotated by 74deg")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      rotate = true;
      rot_ang = 74;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        double_region_box_shape
      };

      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    WHEN("Trajectory profile is 2x2, space is 10x1, space box rotated by 90deg ") //this passes
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      rotate = true;
      rot_ang = 90;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        region_box_shape
      };
      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";
      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          double_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }
#endif // RMF_TRAFFIC__USING_FCL_0_6

//    WHEN("Trajectory profile is 2x2, space is 10x2, space box rotated by 90deg ") //this fails
//    {
//      // Creating trajectory
//      rmf_traffic::Trajectory t1("test_map");
//      rmf_traffic::geometry::Box shape(2.0,2.0);
//      rmf_traffic::geometry::ConstFinalConvexShapePtr final_shape =rmf_traffic::geometry::make_final_convex(shape);
//      auto profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
//      t1.insert(time, profile, Eigen::Vector3d{-5.0,0,0}, Eigen::Vector3d{0,0,0});
//      t1.insert(time+10s, profile, Eigen::Vector3d{5.0,0,0}, Eigen::Vector3d{0,0,0});
//      REQUIRE(t1.size()==2);

//      // Creating Spacetime

//      const auto box = rmf_traffic::geometry::Box(10.0, 2.0); //Centered at (0,0) with width 10m and height 1m
//      const auto final_box = rmf_traffic::geometry::make_final_convex(box);
//      lower_time_bound=time;
//      upper_time_bound=time+10s;

//      rotate=true;
//      rot_ang=90;
//      if(rotate)
//          tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg


//      rmf_traffic::internal::Spacetime region={
//        &lower_time_bound,
//        &upper_time_bound,
//        tf,
//        final_box
//      };
//      std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
//      std::cout<<" Rot:"<<rotate<<" ";
//      if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

//      bool conflict= rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators);
//      CHECK(conflict);
//      CHECK(output_iterators.size()==1);
//    }

//    WHEN("Trajectory profile is 2x2, space is 10x2, space box rotated by 88deg ") //this fails
//    {
//      //Creating trajectory
//      rmf_traffic::Trajectory t1("test_map");
//      rmf_traffic::geometry::Box shape(2.0,2.0);
//      rmf_traffic::geometry::ConstFinalConvexShapePtr final_shape =rmf_traffic::geometry::make_final_convex(shape);
//      auto profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
//      t1.insert(time, profile, Eigen::Vector3d{-5.0,0,0}, Eigen::Vector3d{0,0,0});
//      t1.insert(time+10s, profile, Eigen::Vector3d{5.0,0,0}, Eigen::Vector3d{0,0,0});
//      REQUIRE(t1.size()==2);

//      //Creating Spacetime

//      const auto box = rmf_traffic::geometry::Box(10.0, 2.0); //Centered at (0,0) with width 10m and height 1m
//      const auto final_box = rmf_traffic::geometry::make_final_convex(box);
//      lower_time_bound=time;
//      upper_time_bound=time+10s;

//      rotate=true;
//      rot_ang=88;
//      if(rotate)
//          tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg


//      rmf_traffic::internal::Spacetime region={
//        &lower_time_bound,
//        &upper_time_bound,
//        tf,
//        final_box
//      };
//      std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
//      std::cout<<" Rot:"<<rotate<<" ";
//      if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

//      bool conflict= rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators);
//      CHECK(conflict);
//      CHECK(output_iterators.size()==1);
//    }


    WHEN("Trajectory profile is 2x2, space is 10x2, space box rotated by 87deg ") //this passes
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      //Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      rotate = true;
      rot_ang = 87;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg


      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        double_region_box_shape
      };
      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          double_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    // OBTAINING SAME INTERSECTION PROFILE BY REDEFINING THE SPACE BOX (NO ROTATION)
    WHEN("Trajectory profile is 1x1, space is 1x10, space box not rotated")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      rotate = false;
      rot_ang = 0;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        vertical_region_box_shape
      };
      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    WHEN("Trajectory profile is 1x1, space is 2x10, space box not rotated")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      bool rotate = false;
      double rot_ang = 0;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        double_vertical_region_box_shape
      };
      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";
      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }


    WHEN("Trajectory profile is 2x2, space is 2x10, space box not rotated")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      bool rotate = false;
      double rot_ang = 0;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        double_vertical_region_box_shape
      };

      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";
      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          double_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    WHEN("Trajectory profile is 1x1, space is 1x1, space box not rotated")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      bool rotate = false;
      double rot_ang = 0;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        unit_box_shape
      };

      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    WHEN("Trajectory profile is 1x1, space is 1x1, space box rotated by 0")
    {

      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      bool rotate = true;
      double rot_ang = 0;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        unit_box_shape
      };

      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

#ifdef RMF_TRAFFIC__USING_FCL_0_6
    WHEN("Trajectory profile is 1x1, space is 1x1, space box rotated by 90")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      bool rotate = true;
      double rot_ang = 90;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        unit_box_shape
      };

      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }
#endif // RMF_TRAFFIC__USING_FCL_0_6

    WHEN("Trajectory profile is 1x1, space is 2x1, space box not rotated")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      bool rotate = false;
      double rot_ang = 0;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Box>(2.0, 1.0)
      };

      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";
      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    WHEN(
      "Trajectory profile is 0.5x1 and dim is 1x1, space is 10x1, space box rotated by 90")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-0.25, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {0.25, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      bool rotate = true;
      double rot_ang = 90;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        region_box_shape
      };

      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";
      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          rmf_traffic::Profile{
            rmf_traffic::geometry::make_final_convex<
              rmf_traffic::geometry::Box>(0.5, 1.0)},
          t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }


    WHEN(
      "Trajectory profile is 1x1 and dim is 1x1, space is 10x1, space box rotated by 90")
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      //Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      bool rotate = true;
      double rot_ang = 90;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        region_box_shape
      };

      // std::cout<<"\nT_box("<<shape.get_x_length()<<","<<shape.get_y_length()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
      // std::cout<<" Rot:"<<rotate<<" ";
      // if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";
      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          unit_box, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++++++++ CIRCULAR TRAJECTORIES +++++++++++++++++++++++++++

//    WHEN("Trajectory profile is circle(0.5), space is 10x1, space box rotated by 90deg ") //this fails
//    {

//      //Creating trajectory
//      rmf_traffic::Trajectory t1("test_map");
//      rmf_traffic::geometry::Circle shape(0.5); //radius
//      rmf_traffic::geometry::ConstFinalConvexShapePtr final_shape =rmf_traffic::geometry::make_final_convex(shape);
//      auto profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
//      t1.insert(time, profile, Eigen::Vector3d{-5.0,0,0}, Eigen::Vector3d{0,0,0});
//      t1.insert(time+10s, profile, Eigen::Vector3d{5.0,0,0}, Eigen::Vector3d{0,0,0});
//      REQUIRE(t1.size()==2);

//      //Creating Spacetime

//      const auto box = rmf_traffic::geometry::Box(10.0, 1.0); //Centered at (0,0) with width 10m and height 1m
//      const auto final_box = rmf_traffic::geometry::make_final_convex(box);
//      lower_time_bound=time;
//      upper_time_bound=time+10s;

//      rotate=true;
//      rot_ang=90;
//      if(rotate)
//          tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg

//      //print out the translation compotent of tf
//      // for(int i=0;i<2;i++)
//      //   std::cout<<"Translation "<<i<<":"<<tf.translation()[i]<<std::endl;


//      rmf_traffic::internal::Spacetime region={
//        &lower_time_bound,
//        &upper_time_bound,
//        tf,
//        final_box
//      };
//      std::cout<<"\nT_Circle("<<shape.get_radius()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
//      std::cout<<" Rot:"<<rotate<<" ";
//      if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

//      bool conflict= rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators);
//      CHECK(conflict);
//      CHECK(output_iterators.size()==1);
//    }

//    WHEN("Trajectory profile is circle(1), space is 10x1, space box rotated by 90deg ") //this fails
//    {

//      //Creating trajectory
//      rmf_traffic::Trajectory t1("test_map");
//      rmf_traffic::geometry::Circle shape(1); //radius
//      rmf_traffic::geometry::ConstFinalConvexShapePtr final_shape =rmf_traffic::geometry::make_final_convex(shape);
//      auto profile = rmf_traffic::Trajectory::Profile::make_guided(final_shape);
//      t1.insert(time, profile, Eigen::Vector3d{-5.0,0,0}, Eigen::Vector3d{0,0,0});
//      t1.insert(time+10s, profile, Eigen::Vector3d{5.0,0,0}, Eigen::Vector3d{0,0,0});
//      REQUIRE(t1.size()==2);

//      //Creating Spacetime

//      const auto box = rmf_traffic::geometry::Box(10.0, 1.0); //Centered at (0,0) with width 10m and height 1m
//      const auto final_box = rmf_traffic::geometry::make_final_convex(box);
//      lower_time_bound=time;
//      upper_time_bound=time+10s;

//      rotate=true;
//      rot_ang=90;
//      if(rotate)
//          tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg

//      //print out the translation compotent of tf
//      // for(int i=0;i<2;i++)
//      //   std::cout<<"Translation "<<i<<":"<<tf.translation()[i]<<std::endl;


//      rmf_traffic::internal::Spacetime region={
//        &lower_time_bound,
//        &upper_time_bound,
//        tf,
//        final_box
//      };
//      std::cout<<"\nT_Circle("<<shape.get_radius()<<") Shape_box:("<<box.get_x_length()<<","<<box.get_y_length()<<")";
//      std::cout<<" Rot:"<<rotate<<" ";
//      if(rotate) std::cout<<" Ang:"<<rot_ang<<"\n";

//      bool conflict= rmf_traffic::internal::detect_conflicts(t1,region,&output_iterators);
//      CHECK(conflict);
//      CHECK(output_iterators.size()==1);
//    }

    WHEN(
      "Trajectory profile is circle(0.5), space is 1x10, space box not rotated ") //this fails
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      rotate = false;
      rot_ang = 90;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        vertical_region_box_shape
      };

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          small_circle, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }


    WHEN(
      "Trajectory profile is circle(0.5), space is circle (1), space not rotated ") //this fails
    {
      // Creating trajectory
      rmf_traffic::Trajectory t1;
      t1.insert(time, {-5.0, 0, 0}, {0, 0, 0});
      t1.insert(time+10s, {5.0, 0, 0}, {0, 0, 0});
      REQUIRE(t1.size() == 2);

      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+10s;

      rotate = false;
      rot_ang = 0;
      if (rotate)
        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180));//conflict is only detected when angle is 69deg

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        unit_circle_shape
      };

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          small_circle, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }


    //+++++++++++++CHANGE TIME RANGE OF TRAJECTORY
//    WHEN("Trajectory profile is 1x1, space is 10x1, space box rotated by 90")
//    {
//      // Creating trajectory
//      rmf_traffic::Trajectory t1;
//      t1.insert(time, {-5, 0, 0}, {0, 0, 0});
//      t1.insert(time+60s, {5, 0, 0}, {0, 0, 0});
//      REQUIRE(t1.size()==2);

//      // Creating Spacetime
//      lower_time_bound=time;
//      upper_time_bound=time+60s;

//      bool rotate=true;
//      double rot_ang=69;
//      if(rotate)
//        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg

//      rmf_traffic::internal::Spacetime region={
//        &lower_time_bound,
//        &upper_time_bound,
//        tf,
//        region_box_shape
//      };

//      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
//      CHECK(rmf_traffic::internal::detect_conflicts(
//              unit_box, t1, region, &conflicts));
//      CHECK(conflicts.size() == 1);
//    }

//    WHEN("Trajectory profile is 1x1 and length is 1x1, space is 10x2, space box rotated by 90")
//    {
//      // Creating trajectory
//      rmf_traffic::Trajectory t1;
//      t1.insert(time, {-0.5, 0, 0}, {0, 0, 0});
//      t1.insert(time+10s, {0.5, 0, 0}, {0, 0, 0});
//      REQUIRE(t1.size()==2);

//      // Creating Spacetime
//      lower_time_bound=time;
//      upper_time_bound=time+10s;

//      bool rotate=true;
//      double rot_ang=90;
//      if(rotate)
//        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg

//      rmf_traffic::internal::Spacetime region={
//        &lower_time_bound,
//        &upper_time_bound,
//        tf,
//        double_region_box_shape
//      };

//      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
//      CHECK(rmf_traffic::internal::detect_conflicts(
//              unit_box, t1, region, &conflicts));
//      CHECK(conflicts.size() == 1);
//    }

//    WHEN("Trajectory profile is 2x2 and length is 2x2, space is 10x2, space box rotated by 90")
//    {
//      // Creating trajectory
//      rmf_traffic::Trajectory t1;
//      t1.insert(time, {-0.5, 0, 0}, {0, 0, 0});
//      t1.insert(time+10s, {0.5, 0, 0}, {0, 0, 0});
//      REQUIRE(t1.size()==2);

//      // Creating Spacetime
//      lower_time_bound=time;
//      upper_time_bound=time+10s;

//      bool rotate=true;
//      double rot_ang=90;
//      if(rotate)
//        tf.rotate(Eigen::Rotation2Dd(rot_ang*M_PI/180)); //conflict is only detected when angle is 69deg

//      rmf_traffic::internal::Spacetime region={
//        &lower_time_bound,
//        &upper_time_bound,
//        tf,
//        double_region_box_shape
//      };

//      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
//      CHECK(rmf_traffic::internal::detect_conflicts(
//              double_box, t1, region, &conflicts));
//      CHECK(conflicts.size() == 1);
//    }
  }
}
//=====================================================================================================================
//test trajectorties with 3 or more segments


SCENARIO("Testing intersection of curved trajectory with various spacetimes")
{
  using namespace std::chrono_literals;
  auto time = std::chrono::steady_clock::now();

  const auto unit_circle_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  rmf_traffic::Profile unit_circle{unit_circle_shape};

  const auto region_box_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(10.0, 10.0);

  const auto small_region_box_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Box>(2.0, 2.0);

  rmf_traffic::Trajectory t1;
  t1.insert(time, {-5, 0, 0}, {1, 0, 0});
  t1.insert(time+10s, {0, -5, 0}, {0, -1, 0});

  GIVEN("spacetime that encompasses the trajectory in space and time")
  {
    //creating space to create spacetime
    std::chrono::_V2::steady_clock::time_point lower_time_bound = time;
    std::chrono::_V2::steady_clock::time_point upper_time_bound = time+10s;
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    rmf_traffic::internal::Spacetime region = {
      &lower_time_bound,
      &upper_time_bound,
      tf,
      region_box_shape
    };

    CHECK(rmf_traffic::internal::detect_conflicts(unit_circle, t1, region));
  }

  GIVEN("spacetime that encompasses the trajectory in space but not time")
  {
    //creating space to create spacetime
    std::chrono::_V2::steady_clock::time_point lower_time_bound = time+20s;
    std::chrono::_V2::steady_clock::time_point upper_time_bound = time+30s;
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    rmf_traffic::internal::Spacetime region = {
      &lower_time_bound,
      &upper_time_bound,
      tf,
      region_box_shape
    };

    CHECK(!rmf_traffic::internal::detect_conflicts(unit_circle, t1, region));
  }

  GIVEN("spacetime that partially intersects the trajectory in space and time")
  {
    //creating space to create spacetime
    std::chrono::_V2::steady_clock::time_point lower_time_bound = time;
    std::chrono::_V2::steady_clock::time_point upper_time_bound = time+10s;
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    rmf_traffic::internal::Spacetime region = {
      &lower_time_bound,
      &upper_time_bound,
      tf,
      small_region_box_shape
    };

    CHECK(rmf_traffic::internal::detect_conflicts(unit_circle, t1, region));
  }
}


SCENARIO("Testing multi-waypoint trajectories with various spacetimes")
{
  using namespace std::chrono_literals;
  std::chrono::_V2::steady_clock::time_point lower_time_bound;
  std::chrono::_V2::steady_clock::time_point upper_time_bound;

  const auto small_circle_shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.5);
  rmf_traffic::Profile small_circle{small_circle_shape};

  GIVEN(
    "A Trajectory t1 with circular profile which traces the outline of a box")
  {
    auto time = std::chrono::steady_clock::now();

    rmf_traffic::Trajectory t1;
    //defining t1 as box with corners (-10,10), (-10,-10), (10,-10) and (10,10)
    t1.insert(time, {-10.0, 10.0, 0}, {0, 0, 0});
    t1.insert(time+20s, {-10.0, -10.0, 0}, {0, 0, 0});
    t1.insert(time+40s, {10.0, -10.0, 0}, {0, 0, 0});
    t1.insert(time+60s, {10.0, 10.0, 0}, {0, 0, 0});
    t1.insert(time+80s, {-10.0, 10.0, 0}, {0, 0, 0});

    REQUIRE(t1.size() == 5);

    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    WHEN("Checked with circular spacetime circumscribing t1")
    {
      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+81s;

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(15.0)
      };

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          small_circle, t1, region, &conflicts));
      CHECK(conflicts.size() == 4);
    }

    WHEN("Checked with circular spacetime intersecting 3 segments of t1")
    {
      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+81s;

      tf.translate(Eigen::Vector2d{10, 0});

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(15.0)
      };

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          small_circle, t1, region, &conflicts));
      CHECK(conflicts.size() == 3);
    }

    WHEN("Checked with circular spacetime intersecting 1 waypoint of t1")
    {
      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+81s;

      tf.translate(Eigen::Vector2d{5, 10});

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        small_circle_shape
      };

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          small_circle, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    WHEN("Checked with box spacetime encompassing t1")
    {
      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+81s;

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Box>(30.0, 30.0)
      };

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          small_circle, t1, region, &conflicts));
      CHECK(conflicts.size() == 4);
    }

    WHEN("Checked with box spacetime intersecting 3 segments of t1")
    {
      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+81s;

      tf.translate(Eigen::Vector2d{-10, 0});

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Box>(30.0, 30.0)
      };

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          small_circle, t1, region, &conflicts));
      CHECK(conflicts.size() == 3);
    }

    WHEN("Checked with box spacetime intersecting 1 waypoint of t1")
    {
      // Creating Spacetime
      lower_time_bound = time;
      upper_time_bound = time+81s;

      tf.translate(Eigen::Vector2d{-10, 0});

      rmf_traffic::internal::Spacetime region = {
        &lower_time_bound,
        &upper_time_bound,
        tf,
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Box>(1.0, 1.0)
      };

      rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
      CHECK(rmf_traffic::internal::detect_conflicts(
          small_circle, t1, region, &conflicts));
      CHECK(conflicts.size() == 1);
    }

    WHEN("Checked with box spacetime partially overlapping with t1")
    {
      //Creating Spacetime

      const auto big_region_shape = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Box>(30.0, 30.0);

      THEN("Single conflict when time bounds overlap with first waypoint only")
      {
        lower_time_bound = time;
        upper_time_bound = time+19s;

        rmf_traffic::internal::Spacetime region = {
          &lower_time_bound,
          &upper_time_bound,
          tf,
          big_region_shape
        };

        rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
        CHECK(rmf_traffic::internal::detect_conflicts(
            small_circle, t1, region, &conflicts));
        CHECK(conflicts.size() == 1);
      }

      THEN("Single conflict when time bounds overlap with second waypoint only")
      {
        lower_time_bound = time+21s;
        upper_time_bound = time+39s;

        rmf_traffic::internal::Spacetime region = {
          &lower_time_bound,
          &upper_time_bound,
          tf,
          big_region_shape
        };

        rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
        CHECK(rmf_traffic::internal::detect_conflicts(
            small_circle, t1, region, &conflicts));
        CHECK(conflicts.size() == 1);
      }

      THEN("Single conflict when time bounds overlap with third waypoint only")
      {
        lower_time_bound = time+41s;
        upper_time_bound = time+59s;

        rmf_traffic::internal::Spacetime region = {
          &lower_time_bound,
          &upper_time_bound,
          tf,
          big_region_shape
        };

        rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
        CHECK(rmf_traffic::internal::detect_conflicts(
            small_circle, t1, region, &conflicts));
        CHECK(conflicts.size() == 1);
      }

      THEN("Single conflict when time bounds overlap with fourth waypoint only")
      {
        lower_time_bound = time+61s;
        upper_time_bound = time+79s;

        rmf_traffic::internal::Spacetime region = {
          &lower_time_bound,
          &upper_time_bound,
          tf,
          big_region_shape
        };

        rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
        CHECK(rmf_traffic::internal::detect_conflicts(
            small_circle, t1, region, &conflicts));
        CHECK(conflicts.size() == 1);
      }
    }
  } //end of given


  GIVEN(
    "A Trajectory t1 with circular profile which traces the outline of a circle")
  {
    // TODO(MXG): It looks like this test was never finished.

    auto time = std::chrono::steady_clock::now();

    // defining t1 as circle with corners of radius 10m
    rmf_traffic::Trajectory t1;
    t1.insert(time, {0, 10.0, 0}, {-1, 0, 0});
    t1.insert(time+20s, {-10.0, 0, 0}, {1, 0, 0});
    t1.insert(time+40s, {10.0, -10.0, 0}, {0, 0, 0});
    t1.insert(time+60s, {10.0, 10.0, 0}, {0, 0, 0});
    t1.insert(time+80s, {-10.0, 10.0, 0}, {0, 0, 0});

    REQUIRE(t1.size() == 5);

    rmf_traffic::DetectConflict::Implementation::Conflicts conflicts;
    // Eigen::Isometry2d tf= Eigen::Isometry2d::Identity();
  }
}
