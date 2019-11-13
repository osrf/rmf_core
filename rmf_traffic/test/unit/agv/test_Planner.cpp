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

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_traffic/Conflict.hpp>

#include <rmf_utils/catch.hpp>

#include "../utils_Trajectory.hpp"
#include <src/rmf_traffic/utils.hpp>

#include <iostream>
#include <iomanip>

// TODO(MXG): Move performance testing content into a performance test folder
//const bool test_performance = false;
  const bool test_performance = true;
const std::size_t N = test_performance? 10 : 1;

void print_timing(const std::chrono::steady_clock::time_point& start_time)
{
  const auto finish_time = std::chrono::steady_clock::now();
  std::cout << Catch::getResultCapture().getCurrentTestName()
            << ": " << rmf_traffic::time::to_seconds(finish_time - start_time)
            << std::endl;
}

void display_path(rmf_traffic::Trajectory t, rmf_traffic::agv::Graph graph)
{
  //this is a very bad algorithm but will be improved
  const auto start_time = std::chrono::steady_clock::now();

  std::vector<Eigen::Vector2d> graph_locations;
  std::vector<int> path;
  for(std::size_t i=0;i<graph.num_waypoints();i++)
    graph_locations.push_back(graph.get_waypoint(i).get_location());

  for(auto it=t.begin();it!=t.end();it++)
  {
   auto it2 = std::find(graph_locations.begin(), graph_locations.end(),it->get_finish_position().block<2,1>(0,0));
   if(it2!=graph_locations.end())
     {
       int waypoint_index=std::distance(graph_locations.begin(),it2);
       if(path.empty())
        path.push_back(waypoint_index);
       else if(path.back()!=waypoint_index)
          path.push_back(waypoint_index);

     }

  }

    
  const auto end_time = std::chrono::steady_clock::now();
  std::cout<<"Display Path computed in: "<<std::setprecision(4)<<rmf_traffic::time::to_seconds(end_time-start_time)<<"s\n";

  for(auto it=path.begin();it!=path.end();it++)
  {
    std::cout<<*it;
    if(it!=--path.end())
      std::cout<<"->";
  }
  std::cout<<std::endl;

}

void print_trajectory_info(const rmf_traffic::Trajectory t,rmf_traffic::Time time,rmf_traffic::agv::Graph graph)
{
  int count =1;
  std::cout<<"Trajectory in: "<<t.get_map_name()<<" with "<<t.size()<<" segments\n";
  display_path(t,graph);
  for(auto it=t.begin();it!=t.end();it++)
    {
      auto position=it->get_finish_position();
      std::cout<<"Segment "<<count<<": {"<<position[0]<<","<<position[1]<<","<<position[2]<<"} "<<rmf_traffic::time::to_seconds(it->get_finish_time()-time)<<"s"<<std::endl;
      count++;
    }
  std::cout<<"__________________\n";

}

rmf_traffic::Trajectory test_with_obstacle(
    const std::string& parent,
    const rmf_traffic::agv::Plan& original_plan,
    rmf_traffic::schedule::Database& database,
    const std::vector<rmf_traffic::Trajectory>& obstacles,
    const std::size_t hold_index,
    const rmf_traffic::Time time,
    const bool expect_conflict=true,
    const bool check_holding=true,
    const bool print_info=false)
{
  rmf_traffic::Trajectory t_obs{""};

  for(const auto& obstacle : obstacles)
    database.insert(obstacle);

  rmf_traffic::agv::Plan plan;
  const auto start_time = std::chrono::steady_clock::now();
  for(std::size_t i=0; i < N; ++i)
  {
    plan = original_plan.replan(original_plan.get_start());
    REQUIRE(plan);
  }

  const auto end_time = std::chrono::steady_clock::now();
  if(test_performance)
  {
    const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
    std::cout << "\n" << parent << " w/ obstacle" << std::endl;
    std::cout << "Total: " << sec << std::endl;
    std::cout << "Per run: " << sec/N << std::endl;
  }

  const auto& graph = original_plan.get_configuration().graph();

  REQUIRE(plan.get_trajectories().size() == 1);
  t_obs = plan.get_trajectories().front();
  const std::size_t start_index = original_plan.get_start().waypoint();
  const auto initial_position = graph.get_waypoint(start_index).get_location();

  const std::size_t goal_index = original_plan.get_goal().waypoint();
  const auto goal_position = graph.get_waypoint(goal_index).get_location();

  const auto p_initial = t_obs.front().get_finish_position().block<2,1>(0,0);
  CHECK( (p_initial - initial_position).norm() == Approx(0.0) );

  const auto p_final = t_obs.back().get_finish_position().block<2,1>(0,0);
  CHECK( (p_final - goal_position).norm() == Approx(0.0) );
  
  const auto& original_trajectory = original_plan.get_trajectories().front();
  if (expect_conflict)
  {
    CHECK( original_trajectory.duration() < t_obs.duration() );
  }
  else
  {
    const auto time_diff = original_trajectory.duration() - t_obs.duration();
    CHECK(std::abs(rmf_traffic::time::to_seconds(time_diff)) < 1e-8);
  }

  // Confirm that the trajectory does not conflict with anything in the
  // schedule
  const auto query = database.query(rmf_traffic::schedule::query_everything());
  for (const auto& entry : query)
    CHECK(rmf_traffic::DetectConflict::between(t_obs, entry).empty());

  // Confirm that the vehicle pulled into holding point in order to avoid
  // the conflict
  if (check_holding)
  {
    bool used_holding_point = false;
    for (const auto& wp : plan.get_waypoints())
    {
      if (wp.graph_index() == hold_index)
      {
        used_holding_point = true;
        break;
      }
    }

    CHECK(used_holding_point);
  }

  if (print_info)
  {
    std::cout << "Parent: " << parent << std::endl;
    print_trajectory_info(t_obs, time, graph);
  }
  return t_obs;
}




SCENARIO("Test planning")
{
  using namespace std::chrono_literals;

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {-5, -5}); // 0
  graph.add_waypoint(test_map_name, { 0, -5}); // 1
  graph.add_waypoint(test_map_name, { 5, -5}); // 2
  graph.add_waypoint(test_map_name, {10, -5}); // 3
  graph.add_waypoint(test_map_name, {-5,  0}, true); // 4
  graph.add_waypoint(test_map_name, { 0,  0}, true); // 5
  graph.add_waypoint(test_map_name, { 5,  0}, true); // 6
  graph.add_waypoint(test_map_name, {10,  0}); // 7
  graph.add_waypoint(test_map_name, {10,  4}); // 8
  graph.add_waypoint(test_map_name, { 0,  8}); // 9
  graph.add_waypoint(test_map_name, { 5,  8}); // 10
  graph.add_waypoint(test_map_name, {10, 12}); // 11
  graph.add_waypoint(test_map_name, {12, 12}); // 12
  REQUIRE(graph.num_waypoints()==13);

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
  {
    graph.add_lane(w0, w1);
    graph.add_lane(w1, w0);
  };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(2, 3);
  add_bidir_lane(1, 5);
  add_bidir_lane(3, 7);
  add_bidir_lane(4, 5);
  add_bidir_lane(6, 10);
  add_bidir_lane(7, 8);
  add_bidir_lane(9, 10);
  add_bidir_lane(10, 11);
  add_bidir_lane(11, 12);

  const rmf_traffic::agv::VehicleTraits traits(
      {0.7, 0.3}, {1.0, 0.45}, make_test_profile(UnitCircle));

  rmf_traffic::schedule::Database database;

  const auto default_options = rmf_traffic::agv::Planner::Options{database};
  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options
  };

  //TODO abort planning that is impossible as lane does not exit in the graph

  /*WHEN("goal waypoint does not have a lane in the graph")
  {

    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const rmf_traffic::agv::VehicleTraits traits(
        {0.7, 0.3}, {1.0, 0.45}, make_test_profile(UnitCircle));
    rmf_traffic::schedule::Database database;
    rmf_traffic::agv::Planner::Options options(traits, graph, database);
    std::vector<rmf_traffic::Trajectory> solution;
    
    bool solved=rmf_traffic::agv::Planner::solve(time,3,0.0,9,nullptr,options,solution);
    CHECK_FALSE(solved);
    CHECK(solution.size()==0);

  } */

  WHEN("initial conditions satisfy the goals")
  {
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();

    auto plan = planner.plan(
          rmf_traffic::agv::Planner::Start(start_time, 3, 0.0),
          rmf_traffic::agv::Planner::Goal(3, 0.0));

    REQUIRE(plan);
    CHECK(plan.valid());
    CHECK(plan.get_trajectories().size()==1);
    CHECK(plan.get_trajectories().front().size()==0);
  }

  WHEN("initial and goal waypoints are same but goal_orientation is different")
  {
    rmf_traffic::agv::Plan plan;
    CHECK(!plan);
    const double goal_orientation = M_PI/2.0;
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();

    for(std::size_t i = 0; i < N; ++i)
    {
      plan = planner.plan(
          rmf_traffic::agv::Planner::Start{start_time, 3, 0.0},
          rmf_traffic::agv::Planner::Goal{3, goal_orientation});

      REQUIRE(plan);
      CHECK(plan.valid());
    }

    const auto end_time = std::chrono::steady_clock::now();
    if(test_performance)
    {
      const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
      std::cout << "\nUnconstrained" << std::endl;
      std::cout << "Total: " << sec << std::endl;
      std::cout << "Per run: " << sec/N << std::endl;
    }

    CHECK(plan.get_trajectories().size()==1);
    const auto t = plan.get_trajectories().front();
    const auto final_p = t.front().get_finish_position().block<2,1>(0,0);
    const auto err = (final_p - Eigen::Vector2d(10, -5)).norm();
    CHECK(err == Approx(0.0) );
    CHECK(t.back().get_finish_position()[2] - goal_orientation == Approx(0));
    CHECK(t.back().get_finish_time() > start_time);
  }

  WHEN("goal waypoint is an adjacent node")
  {
    rmf_traffic::agv::Plan plan;
    CHECK(!plan);
    const double goal_orientation = M_PI;
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();

    for(std::size_t i = 0; i < N; ++i)
    {
      plan = planner.plan(
            rmf_traffic::agv::Planner::Start{start_time, 3, M_PI},
            rmf_traffic::agv::Planner::Goal{2, goal_orientation});
      REQUIRE(plan);
      CHECK(plan.valid());
    }

    const auto end_time = std::chrono::steady_clock::now();
    if(test_performance)
    {
      const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
      std::cout << "\nUnconstrained" << std::endl;
      std::cout << "Total: " << sec << std::endl;
      std::cout << "Per run: " << sec/N << std::endl;
    }

    CHECK(plan.get_trajectories().size()==1);
    const auto t = plan.get_trajectories().front();
    REQUIRE(t.size() == 4); //start, stop acc, start decc, goal

    const auto initial_p = t.front().get_finish_position().block<2,1>(0, 0);
    CHECK((initial_p - Eigen::Vector2d(10, -5)).norm() == Approx(0.0) );

    const auto final_p = t.back().get_finish_position().block<2,1>(0, 0);
    CHECK((final_p - Eigen::Vector2d(5, -5)).norm() == Approx(0.0));

    CHECK(t.back().get_finish_position()[2] - goal_orientation==Approx(0));
    CHECK(t.back().get_finish_time() > start_time);
  }

  GIVEN("Goal from 12->5 and obstacle from 5->12")
  {
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const auto start = rmf_traffic::agv::Planner::Start{time, 12, 0.0};
    const auto goal = rmf_traffic::agv::Planner::Goal{5};

    std::vector<rmf_traffic::Trajectory> obstacles;

    rmf_traffic::Trajectory obstacle{test_map_name};
    obstacle.insert(
          time + 19s,
          make_test_profile(UnitCircle),
          {0.0, 8.0, 0.0},
          {0.0, 0.0, 0.0});
    obstacle.insert(
          time + 40s,
          make_test_profile(UnitCircle),
          {5.0, 8.0, 0.0},
          {0.0, 0.0, 0.0});
    obstacle.insert(
          time + 50s,
          make_test_profile(UnitCircle),
          {10.0, 12.0, 0.0},
          {0.0, 0.0, 0.0});
    REQUIRE(obstacle.size() == 3);
    obstacles.push_back(obstacle);

    WHEN("Docking is not constrained")
    {
      add_bidir_lane(5, 9);
      add_bidir_lane(11, 12);

      planner = rmf_traffic::agv::Planner{
          rmf_traffic::agv::Planner::Configuration{graph, traits},
          default_options
      };

      rmf_traffic::agv::Plan plan;
      const auto start_time = std::chrono::steady_clock::now();

      for(std::size_t i = 0; i < N; ++i)
      {
        plan = planner.plan(start, goal);

        REQUIRE(plan);
        CHECK(plan.valid());
      }

      const auto end_time = std::chrono::steady_clock::now();
      if(test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nUnconstrained" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan.get_trajectories().size() == 1);
      const auto t = plan.get_trajectories().front();

      const auto initial_p = t.front().get_finish_position().block<2,1>(0, 0);
      CHECK( (initial_p - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );

      const auto final_p = t.back().get_finish_position().block<2,1>(0, 0);
      CHECK( (final_p - Eigen::Vector2d(0, 0)).norm() == Approx(0.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
              "Unconstrained", plan, database, obstacles, 6, time);
      }
    }

    WHEN("Docking must be at 90-degrees")
    {
      using namespace rmf_traffic::agv;
      add_bidir_lane(11, 12);
      graph.add_lane(9, {5, Graph::OrientationConstraint::make({M_PI_2})});
      graph.add_lane({5, Graph::OrientationConstraint::make({M_PI_2})}, 9);

      planner = rmf_traffic::agv::Planner{
          rmf_traffic::agv::Planner::Configuration{graph, traits},
          default_options
      };

      rmf_traffic::agv::Plan plan;
      const auto start_time = std::chrono::steady_clock::now();
      for(std::size_t i=0; i < N; ++i)
      {
        plan = planner.plan(start, goal);
        REQUIRE(plan);
      }
 
      const auto end_time = std::chrono::steady_clock::now();
      if(test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nConstrained to 0.0" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan.get_trajectories().size() == 1);
      const auto& t = plan.get_trajectories().front();

      const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
      CHECK( (p_initial - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );

      const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
      CHECK( (p_final - Eigen::Vector2d(0, 0)).norm() == Approx(0.0) );
      CHECK( t.back().get_finish_position()[2] == Approx(M_PI/2.0) );

     WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
              "Constrained to 0.0", plan, database, obstacles, 6, time);
      }
    }
  }

  GIVEN("Goal from 2->12 and obstacle from 9->1")
  {
    add_bidir_lane(5, 9);
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const auto start = rmf_traffic::agv::Planner::Start(time, 2, 0.0);
    const auto goal = rmf_traffic::agv::Planner::Goal(12);

    std::vector<rmf_traffic::Trajectory> obstacles;

    rmf_traffic::Trajectory obstacle{test_map_name};
    obstacle.insert(
          time + 24s,
          make_test_profile(UnitCircle),
          {0.0, 8.0, 0.0},
          {0.0, 0.0, 0.0});
    obstacle.insert(
          time + 50s,
          make_test_profile(UnitCircle),
          {0.0, 0.0, 0.0},
          {0.0, 0.0, 0.0});
    obstacle.insert(
          time + 70s,
          make_test_profile(UnitCircle),
          {0.0, -5.0, 0.0},
          {0.0, 0.0, 0.0});
    obstacles.push_back(obstacle);

    WHEN("Docking is not constrained")
    {
      using namespace rmf_traffic::agv;
      add_bidir_lane(11, 12);

      planner = rmf_traffic::agv::Planner{
          rmf_traffic::agv::Planner::Configuration{graph, traits},
          default_options
      };

      rmf_traffic::agv::Plan plan;
      const auto start_time = std::chrono::steady_clock::now();

      for(std::size_t i=0; i < N; ++i)
      {
        plan = planner.plan(start, goal);
        REQUIRE(plan);
      }

      const auto end_time = std::chrono::steady_clock::now();
      if(test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nUnconstrained" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan.get_trajectories().size() == 1);
      const auto t = plan.get_trajectories().front();

      const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
      CHECK( (p_initial - Eigen::Vector2d(5, -5)).norm() == Approx(0.0) );

      const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
      CHECK( (p_final - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
              "Unconstrained", plan, database, obstacles, 4, time);
      }
    }

    WHEN("Docking must be at 0-degrees")
    {
      using namespace rmf_traffic::agv;
      graph.add_lane(11, {12, Graph::OrientationConstraint::make({0.0})});
      graph.add_lane({12, Graph::OrientationConstraint::make({0.0})}, 11);

      planner = rmf_traffic::agv::Planner{
          rmf_traffic::agv::Planner::Configuration{graph, traits},
          default_options
      };

      rmf_traffic::agv::Plan plan;
      const auto start_time = std::chrono::steady_clock::now();

      for(std::size_t i=0; i < N; ++i)
      {
        plan = planner.plan(start, goal);
        REQUIRE(plan);
      }

      const auto end_time = std::chrono::steady_clock::now();
      if(test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nConstrained to 0.0" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan.get_trajectories().size() == 1);
      const auto& t = plan.get_trajectories().front();

      const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
      CHECK( (p_initial - Eigen::Vector2d(5, -5)).norm() == Approx(0.0) );

      const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
      CHECK( (p_final - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );
      CHECK( t.back().get_finish_position()[2] == Approx(0.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
              "Constrained to 0.0", plan, database, obstacles, 4, time);
      }
    }

    WHEN("Docking must be at 180-degrees")
    {
      using namespace rmf_traffic::agv;
      graph.add_lane(11, {12, Graph::OrientationConstraint::make({M_PI})});
      graph.add_lane({12, Graph::OrientationConstraint::make({M_PI})}, 11);

      planner = rmf_traffic::agv::Planner{
          rmf_traffic::agv::Planner::Configuration{graph, traits},
          default_options
      };

      rmf_traffic::agv::Plan plan;
      const auto start_time = std::chrono::steady_clock::now();
      for(std::size_t i=0; i < N; ++i)
      {
        plan = planner.plan(start, goal);
        REQUIRE(plan);
      }

      const auto end_time = std::chrono::steady_clock::now();
      if(test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nConstrained to 180.0" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan.get_trajectories().size() == 1);
      const auto& t = plan.get_trajectories().front();

      const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
      CHECK( (p_initial - Eigen::Vector2d(5, -5)).norm() == Approx(0.0) );

      const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
      CHECK( (p_final - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );

      const double err = rmf_traffic::internal::wrap_to_pi(
            t.back().get_finish_position()[2] - M_PI);
      CHECK(err  == Approx(0.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
              "Constrained to 180.0", plan, database, obstacles, 4, time);
      }
    }
  } //end of GIVEN



  GIVEN("Goal from 12->0 and two obstacles : 9->11 and 1->9")
  {
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    // expect robot to wait at holding point 6 and 4
    const std::size_t start_index = 12;
    const auto start = rmf_traffic::agv::Plan::Start{time, start_index, 0.0};

    const std::size_t goal_index = 0;
    const auto goal = rmf_traffic::agv::Plan::Goal{goal_index};

    std::vector<rmf_traffic::Trajectory> obstacles;

    rmf_traffic::Trajectory obstacle_1{test_map_name};
    obstacle_1.insert(
         time + 19s,
         make_test_profile(UnitCircle),
         {0.0, 8.0, 0.0},
         {0.0, 0.0, 0.0});
    obstacle_1.insert(
         time + 40s,
         make_test_profile(UnitCircle),
         {5.0, 8.0, 0.0},
         {0.0, 0.0, 0.0});
    obstacle_1.insert(
         time + 50s,
         make_test_profile(UnitCircle),
         {10.0, 12.0, 0.0},
         {0.0, 0.0, 0.0});
    REQUIRE(obstacle_1.size() == 3);

    WHEN("Docking is not constrained")
    {
      using namespace rmf_traffic::agv;
      add_bidir_lane(5, 9);
      add_bidir_lane(11, 12);

      planner = Planner{Planner::Configuration{graph, traits}, default_options};

      rmf_traffic::agv::Plan plan;

      const auto start_time = std::chrono::steady_clock::now();
      for(std::size_t i=0; i < N; ++i)
      {
        plan = planner.plan(start, goal);
        REQUIRE(plan);
      }

      const auto end_time = std::chrono::steady_clock::now();
      if(test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nUnconstrained" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan.get_trajectories().size() == 1);
      const auto t = plan.get_trajectories().front();

      const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
      const auto p_initial_g = graph.get_waypoint(start_index).get_location();
      CHECK( (p_initial - p_initial_g).norm() == Approx(0.0) );

      const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
      const auto p_final_g = graph.get_waypoint(goal_index).get_location();
      CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

      WHEN("First obstacle is introduced")
      {
        CHECK(rmf_traffic::DetectConflict::between(t,obstacle_1).size() > 0);
        obstacles.push_back(obstacle_1);

        auto t_obs=test_with_obstacle(
              "Unconstrained", plan, database, obstacles, 6, time);
      }

      WHEN("Second obstacle is introduced")
      {
        REQUIRE(graph.get_waypoint(4).is_holding_point());
        rmf_traffic::Trajectory obstacle_2{test_map_name};
        obstacle_2.insert(
              time + 49s,
              make_test_profile(UnitCircle),
              {0.0, -5.0, M_PI_2},
              {0.0, 0.0, 0.0});
        obstacle_2.insert(
              time + 60s,
              make_test_profile(UnitCircle),
              {0.0, 0.0, M_PI_2},
              {0.0, 0.0, 0.0});
        obstacle_2.insert(
              time + 87s,
              make_test_profile(UnitCircle),
              {0.0, 8.0, M_PI_2},
              {0.0, 0.0, 0.0});
        REQUIRE(obstacle_2.size()==3);
        REQUIRE(rmf_traffic::DetectConflict::between(obstacle_1,obstacle_2).size()==0);
        CHECK(rmf_traffic::DetectConflict::between(t,obstacle_2).size()>0);

        obstacles.push_back(obstacle_2);
        auto t_obs = test_with_obstacle(
              "Unconstrained", plan, database, obstacles, 4, time);
      }

      WHEN("Both obstacles are introduced")
      {
        rmf_traffic::Trajectory obstacle_2{test_map_name};
        obstacle_2.insert(
              time + 81s,
              make_test_profile(UnitCircle),
              {0.0, -5.0, 0.0},
              {0.0, 0.0, 0.0});
        obstacle_2.insert(
              time + 92s,
              make_test_profile(UnitCircle),
              {0.0, 0.0, 0.0},
              {0.0, 0.0, 0.0});
        obstacle_2.insert(
              time + 110s,
              make_test_profile(UnitCircle),
              {0.0, 8.0, 0.0},
              {0.0, 0.0, 0.0});
        REQUIRE(obstacle_2.size()==3);
        obstacles.push_back(obstacle_1);
        obstacles.push_back(obstacle_2);

        auto t_obs=test_with_obstacle(
              "Unconstrained", plan, database, obstacles, 6, time);
      }
    }
  }
}




SCENARIO("DP1 Graph")
{
  using namespace std::chrono_literals;

  //initialize graph
  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {12, -12});       // 0
  graph.add_waypoint(test_map_name, {18, -12}, true); // 1
  graph.add_waypoint(test_map_name, {-10, -8});       // 2
  graph.add_waypoint(test_map_name, {-2, -8}, true);  // 3
  graph.add_waypoint(test_map_name, { 3,  -8});       // 4
  graph.add_waypoint(test_map_name, {12,  -8});       // 5
  graph.add_waypoint(test_map_name, {18,  -8}, true); // 6
  graph.add_waypoint(test_map_name, {-15,  -4},true); // 7
  graph.add_waypoint(test_map_name, {-10,  -4});      // 8
  graph.add_waypoint(test_map_name, { -2,  -4},true); // 9
  graph.add_waypoint(test_map_name, { 3,  -4});       // 10
  graph.add_waypoint(test_map_name, {6, -4});         // 11
  graph.add_waypoint(test_map_name, {9, -4});         // 12
  graph.add_waypoint(test_map_name, {-15,  0});       // 13
  graph.add_waypoint(test_map_name, {-10,  0});       // 14
  graph.add_waypoint(test_map_name, { 0,  0});        // 15 DOOR (not implemented)
  graph.add_waypoint(test_map_name, { 3,  0});        // 16
  graph.add_waypoint(test_map_name, {6, 0});          // 17
  graph.add_waypoint(test_map_name, {9, 0});          // 18
  graph.add_waypoint(test_map_name, {15,  0},true);   // 19
  graph.add_waypoint(test_map_name, {18,  0},true);   // 20
  graph.add_waypoint(test_map_name, { -2,  4},true);  // 21
  graph.add_waypoint(test_map_name, { 3,  4});        // 22
  graph.add_waypoint(test_map_name, {6, 4});          // 23
  graph.add_waypoint(test_map_name, {9, 4});          // 24
  graph.add_waypoint(test_map_name, {15,  4});        // 25
  graph.add_waypoint(test_map_name, {18,  4});        // 26
  graph.add_waypoint(test_map_name, { -15,  8},true); // 27
  graph.add_waypoint(test_map_name, {-10,  8},true);  // 28
  graph.add_waypoint(test_map_name, {3, 8}, true);    // 29
  graph.add_waypoint(test_map_name, {6, 8}, true);    // 30
  graph.add_waypoint(test_map_name, {15,  8}, true);  // 31
  graph.add_waypoint(test_map_name, {18,  8}, true);  // 32

  REQUIRE(graph.num_waypoints()==33);

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
  {
    graph.add_lane(w0, w1);
    graph.add_lane(w1, w0);
  };
  //horizontal lates
  add_bidir_lane(0, 1);
  add_bidir_lane(2, 3);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(7, 8);
  add_bidir_lane(8, 9);
  add_bidir_lane(10, 11);
  add_bidir_lane(11, 12);
  add_bidir_lane(13, 14);
  add_bidir_lane(14, 15);
  add_bidir_lane(15, 16);
  add_bidir_lane(16, 17);
  add_bidir_lane(17, 18);
  add_bidir_lane(21, 22);
  add_bidir_lane(23, 24);
  add_bidir_lane(24, 25);
  add_bidir_lane(25, 26);


  //vertical lanes
  add_bidir_lane(0, 5);
  add_bidir_lane(2, 8);
  add_bidir_lane(4, 10);
  add_bidir_lane(8, 14);
  add_bidir_lane(10, 16);
  add_bidir_lane(11, 17);
  add_bidir_lane(12, 18);
  add_bidir_lane(13, 27);
  add_bidir_lane(14, 28);
  add_bidir_lane(16, 22);
  add_bidir_lane(17, 23);
  add_bidir_lane(18, 24);
  add_bidir_lane(19, 25);
  add_bidir_lane(20, 26);
  add_bidir_lane(22, 29);
  add_bidir_lane(23, 30);
  add_bidir_lane(25, 31);
  add_bidir_lane(26, 32);

//  std::size_t start_index=17;
//  std::size_t goal_index=12;

  rmf_traffic::schedule::Database database;
  const rmf_traffic::agv::VehicleTraits traits{
      {1.0, 0.4},
      {1.0, 0.5},
      make_test_profile(UnitCircle)
  };
  const rmf_traffic::Time time = std::chrono::steady_clock::now();
  const rmf_traffic::agv::Planner::Options default_options{database};

  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options
  };

  std::vector<rmf_traffic::Trajectory> obstacles;

  using rmf_traffic::DetectConflict;

  WHEN("Robot moves from 1->30 given multiple non-conflicting "
       "obstacles that partially overlap in time")
  {
    const std::size_t start_index = 1;
    const auto start = rmf_traffic::agv::Plan::Start{time, start_index, 0.0};
    const std::size_t goal_index = 30;
    const auto goal = rmf_traffic::agv::Plan::Goal{goal_index};

    const auto start_time = std::chrono::steady_clock::now();
    const auto plan = planner.plan(start, goal);
    REQUIRE(plan);
    print_timing(start_time);

    CHECK(plan.get_trajectories().size() == 1);
    const auto t= plan.get_trajectories().front();

    const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
    const auto p_initial_g = graph.get_waypoint(start_index).get_location();
    CHECK( (p_initial - p_initial_g).norm()==Approx(0.0));

    const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
    const auto p_final_g = graph.get_waypoint(goal_index).get_location();
    CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

    WHEN("Obstacle 28->3 that partially overlaps in time")
    {
      rmf_traffic::Trajectory obstacle_1(test_map_name);
      obstacle_1.insert(
            time,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{-10,8,-M_PI_2},
            Eigen::Vector3d{0,0,0});
      obstacle_1.insert(
            time + 20s,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{-10,-8,-M_PI_2},
            Eigen::Vector3d{0,0,0});
      obstacle_1.insert(
            time + 25s,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{-10,-8,0},
            Eigen::Vector3d{0,0,0});
      obstacle_1.insert(
            time + 35s,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{-2,-8,0},
            Eigen::Vector3d{0,0,0});

      REQUIRE(rmf_traffic::DetectConflict::between(obstacle_1, t).size() == 0);
      obstacles.push_back(obstacle_1);

      test_with_obstacle(
            "Partial 28->3", plan, database, obstacles, 0, time, false);

      WHEN("Obstacle 28->3, 16-29 added")
      {
        rmf_traffic::Trajectory obstacle_2(test_map_name);
        obstacle_2.insert(
              time+20s,
              make_test_profile(UnitCircle),
              Eigen::Vector3d{3,0,M_PI_2},
              Eigen::Vector3d{0,0,0});
        obstacle_2.insert(
              time+30s,
              make_test_profile(UnitCircle),
              Eigen::Vector3d{3,8,M_PI_2},
              Eigen::Vector3d{0,0,0});

        const auto view =
            database.query(rmf_traffic::schedule::query_everything());
        for(const auto& _t : view)
          REQUIRE(DetectConflict::between(obstacle_2, _t).empty());

        REQUIRE(DetectConflict::between(obstacle_2, t).empty());
        obstacles.push_back(obstacle_2);
        test_with_obstacle(
              "Partial 28->3, 16-29",
              plan, database, obstacles, 0, time, false);

        WHEN("Obstacle 28->3, 16-29, 24->26 added")
        {
          rmf_traffic::Trajectory obstacle_3(test_map_name);
          obstacle_3.insert(
                time+40s,
                make_test_profile(UnitCircle),
                Eigen::Vector3d{9,4, 0},
                Eigen::Vector3d{0,0,0});
          obstacle_3.insert(
                time+60s,
                make_test_profile(UnitCircle),
                Eigen::Vector3d{18,4,0},
                Eigen::Vector3d{0,0,0});

          const auto view =
              database.query(rmf_traffic::schedule::query_everything());
          for(const auto& _t : view)
            REQUIRE(DetectConflict::between(obstacle_3, _t).empty());

          REQUIRE(DetectConflict::between(obstacle_3, t).empty());
          obstacles.push_back(obstacle_3);

          test_with_obstacle(
                "Partial 28->3, 16-29, 24->26", plan, database,
                obstacles, 0, time, false);

          WHEN("Obstacle 28->3, 16-29, 24->26, 21->22, 13->14, 5->6 added")
          {
            rmf_traffic::Trajectory obstacle_4(test_map_name);
            obstacle_4.insert(
                  time + 10s,
                  make_test_profile(UnitCircle),
                  Eigen::Vector3d{-2, -4, 0},
                  Eigen::Vector3d{0, 0, 0});
            obstacle_4.insert(
                  time + 20s,
                  make_test_profile(UnitCircle),
                  Eigen::Vector3d{3, 4, 0},
                  Eigen::Vector3d{0, 0, 0});

            const auto view =
                database.query(rmf_traffic::schedule::query_everything());
            for(const auto& _t : view)
              REQUIRE(DetectConflict::between(obstacle_4,_t).empty());

            REQUIRE(DetectConflict::between(obstacle_4, t).empty());
            obstacles.push_back(obstacle_4);

            rmf_traffic::Trajectory obstacle_5(test_map_name);
            obstacle_5.insert(
                  time + 15s,
                  make_test_profile(UnitCircle),
                  Eigen::Vector3d{-15, 0, 0},
                  Eigen::Vector3d{0, 0, 0});

            obstacle_5.insert(
                  time + 45s,
                  make_test_profile(UnitCircle),
                  Eigen::Vector3d{-10, 0, 0},
                  Eigen::Vector3d{0, 0, 0});

            for(const auto& _t : view)
              REQUIRE(DetectConflict::between(obstacle_5, _t).empty());

            REQUIRE(DetectConflict::between(obstacle_5, t).empty());
            obstacles.push_back(obstacle_5);

            rmf_traffic::Trajectory obstacle_6(test_map_name);
            obstacle_6.insert(
                  time + 60s,
                  make_test_profile(UnitCircle),
                  Eigen::Vector3d{-12, -8, 0},
                  Eigen::Vector3d{0, 0, 0});
            obstacle_6.insert(
                  time + 75s,
                  make_test_profile(UnitCircle),
                  Eigen::Vector3d{-18, -8, 0},
                  Eigen::Vector3d{0, 0, 0});

            for(const auto& _t : view)
              REQUIRE(DetectConflict::between(obstacle_6, _t).empty());
            REQUIRE(DetectConflict::between(obstacle_6,t).empty());
            obstacles.push_back(obstacle_6);

            test_with_obstacle(
                  "Partial 28->3, 16-29, 24->26, 21->22, 13->14, 5->6",
                  plan, database, obstacles, 0, time, false);
          }
        }
      }
    }
  }

  WHEN("Robot moves from 1->30 given multiple non-conflicting obstacles that fully overlap in time")
  {
    const auto time = std::chrono::steady_clock::now();
    const std::size_t start_index = 1;
    const auto start = rmf_traffic::agv::Plan::Start{time, start_index, 0.0};

    const std::size_t goal_index = 30;
    const auto goal = rmf_traffic::agv::Plan::Goal{goal_index};

    const auto start_time = std::chrono::steady_clock::now();
    const auto plan = planner.plan(start, goal);
    REQUIRE(plan);
    print_timing(start_time);

    CHECK(plan.get_trajectories().size()==1);
    auto t = plan.get_trajectories().front();

    const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
    const auto p_initial_g = graph.get_waypoint(start_index).get_location();
    CHECK( (p_initial - p_initial_g).norm() == Approx(0.0) );

    const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
    const auto p_final_g = graph.get_waypoint(goal_index).get_location();
    CHECK( (p_final - p_final_g).norm()==Approx(0.0) );

    WHEN("Obstacle 28->3 that partially overlaps in time")
    {
      rmf_traffic::Trajectory obstacle_1(test_map_name);
      obstacle_1.insert(
            time,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{-10,8,-M_PI_2},
            Eigen::Vector3d{0,0,0});
      obstacle_1.insert(
            time + 30s,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{-10,-8,-M_PI_2},
            Eigen::Vector3d{0,0,0});
      obstacle_1.insert(
            time + 50s,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{-10,-8,0},
            Eigen::Vector3d{0,0,0});
      obstacle_1.insert(
            time + 76s,
            make_test_profile(UnitCircle),
            Eigen::Vector3d{-2,-8,0},
            Eigen::Vector3d{0,0,0});

      REQUIRE(DetectConflict::between(obstacle_1,t).empty());
      obstacles.push_back(obstacle_1);
      test_with_obstacle(
            "Full 28->3", plan, database, obstacles, 0, time, false);

      WHEN("Obstacle 28->3, 16-29 added")
      {
        rmf_traffic::Trajectory obstacle_2(test_map_name);
        obstacle_2.insert(
           time,
           make_test_profile(UnitCircle),
           Eigen::Vector3d{3,0,M_PI_2},
           Eigen::Vector3d{0,0,0});
        obstacle_2.insert(
           time+76s,
           make_test_profile(UnitCircle),
           Eigen::Vector3d{3,8,M_PI_2},
           Eigen::Vector3d{0,0,0});

        const auto view =
            database.query(rmf_traffic::schedule::query_everything());
        for(const auto& _t : view)
          REQUIRE(DetectConflict::between(obstacle_2,_t).empty());

        REQUIRE(DetectConflict::between(obstacle_2,t).empty());
        obstacles.push_back(obstacle_2);
        test_with_obstacle(
              "Full 28->3, 16-29", plan, database, obstacles, 0, time, false);

        WHEN("Obstacle 28->3, 16-29, 24->26 added")
        {
          rmf_traffic::Trajectory obstacle_3(test_map_name);
          obstacle_3.insert(
                time,
                make_test_profile(UnitCircle),
                Eigen::Vector3d{9,4, 0},
                Eigen::Vector3d{0,0,0});
          obstacle_3.insert(
                time + 76s,
                make_test_profile(UnitCircle),
                Eigen::Vector3d{18,4,0},
                Eigen::Vector3d{0,0,0});

          const auto view =
              database.query(rmf_traffic::schedule::query_everything());
          for(const auto& _t : view)
            REQUIRE(DetectConflict::between(obstacle_3,_t).empty());

          REQUIRE(DetectConflict::between(obstacle_3,t).empty());
          obstacles.push_back(obstacle_3);

          test_with_obstacle(
                "Full 28->3, 16-29, 24->26",
                plan, database, obstacles, 0, time, false);

          WHEN("Obstacle 28->3, 16-29, 24->26, 21->22, 13->14, 5->6 added")
          {
            rmf_traffic::Trajectory obstacle_4(test_map_name);
            obstacle_4.insert(
                 time,
                 make_test_profile(UnitCircle),
                 Eigen::Vector3d{-2, 4, 0},
                 Eigen::Vector3d{0, 0, 0});
            obstacle_4.insert(
                 time + 76s,
                 make_test_profile(UnitCircle),
                 Eigen::Vector3d{3, 4, 0},
                 Eigen::Vector3d{0, 0, 0});

            const auto view =
                database.query(rmf_traffic::schedule::query_everything());
            for(const auto& _t : view)
              REQUIRE(DetectConflict::between(obstacle_4,_t).empty());

            REQUIRE(DetectConflict::between(obstacle_4,t).empty());
            obstacles.push_back(obstacle_4);

            rmf_traffic::Trajectory obstacle_5(test_map_name);
            obstacle_5.insert(
                  time,
                  make_test_profile(UnitCircle),
                  Eigen::Vector3d{-15,0, 0},
                  Eigen::Vector3d{0,0,0});
            obstacle_5.insert(
                  time + 76s,
                  make_test_profile(UnitCircle),
                  Eigen::Vector3d{-10,0,0},
                  Eigen::Vector3d{0,0,0});

            for(const auto& _t : view)
              REQUIRE(DetectConflict::between(obstacle_5, _t).empty());

            REQUIRE(DetectConflict::between(obstacle_5, t).empty());
            obstacles.push_back(obstacle_5);

            rmf_traffic::Trajectory obstacle_6(test_map_name);
            obstacle_6.insert(
               time,
               make_test_profile(UnitCircle),
               Eigen::Vector3d{-12,-8, 0},
               Eigen::Vector3d{0,0,0});
            obstacle_6.insert(
               time + 76s,
               make_test_profile(UnitCircle),
               Eigen::Vector3d{-18,-8,0},
               Eigen::Vector3d{0,0,0});

            for(const auto& _t : view)
              REQUIRE(DetectConflict::between(obstacle_6, _t).empty());

            REQUIRE(DetectConflict::between(obstacle_6, t).empty());
            obstacles.push_back(obstacle_6);

            test_with_obstacle(
                  "Full 28->3, 16-29, 24->26, 2->3, 13->14, 5->6",
                  plan, database, obstacles, 0, time, false);
          }
        }
      }
    }
  }


  WHEN("Robot moves from 20->23 and obstacle moves from 23->20")
  {
    const auto time = std::chrono::steady_clock::now();
    const std::size_t start_index = 20;
    const auto start = rmf_traffic::agv::Plan::Start{time, start_index, M_PI/2};

    const std::size_t goal_index = 23;
    const auto goal = rmf_traffic::agv::Plan::Goal{goal_index};

    const auto start_time = std::chrono::steady_clock::now();
    const auto plan = planner.plan(start, goal);
    REQUIRE(plan);
    print_timing(start_time);

    CHECK(plan.get_trajectories().size() == 1);
    const auto t = plan.get_trajectories().front();

    const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
    const auto p_initial_g = graph.get_waypoint(start_index).get_location();
    CHECK( (p_initial - p_initial_g).norm() == Approx(0.0) );

    const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
    const auto p_final_g = graph.get_waypoint(goal_index).get_location();
    CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

    rmf_traffic::Trajectory obstacle(test_map_name);
    obstacle.insert(
          time + 6s,
          make_test_profile(UnitCircle),
          Eigen::Vector3d{6,4,0},
          Eigen::Vector3d{0,0,0});
    obstacle.insert(
          time + 16s,
          make_test_profile(UnitCircle),
          Eigen::Vector3d{18,4,0},
          Eigen::Vector3d{0,0,0});
    obstacle.insert(
          time + 26s,
          make_test_profile(UnitCircle),
          Eigen::Vector3d{18,0,0},
          Eigen::Vector3d{0,0,0});

    WHEN("First obstacle is introduced")
    {
       REQUIRE(!rmf_traffic::DetectConflict::between(obstacle, t).empty());
       obstacles.push_back(obstacle);
       test_with_obstacle("Unconstrained", plan, database, obstacles, 32, time);
    }
  }

  WHEN("Robot moves from 27->32 with multiple obstacles along the way")
  {
    const auto time = std::chrono::steady_clock::now();
    const std::size_t start_index = 27;
    const auto start = rmf_traffic::agv::Plan::Start{time, start_index, 0.0};

    const std::size_t goal_index = 32;
    const auto goal = rmf_traffic::agv::Plan::Goal{goal_index};

    const auto start_time = std::chrono::steady_clock::now();
    const auto plan = planner.plan(start, goal);
    CHECK(plan);
    print_timing(start_time);

    CHECK(plan.get_trajectories().size()==1);
    auto t = plan.get_trajectories().front();

    const auto p_initial = t.front().get_finish_position().block<2,1>(0,0);
    const auto p_initial_g = graph.get_waypoint(start_index).get_location();
    CHECK( (p_initial - p_initial_g).norm() == Approx(0.0) );

    const auto p_final = t.back().get_finish_position().block<2,1>(0,0);
    const auto p_final_g = graph.get_waypoint(goal_index).get_location();
    CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

    rmf_traffic::Trajectory obstacle_1(test_map_name);
    obstacle_1.insert(
          time,
          make_test_profile(UnitCircle),
          Eigen::Vector3d{-10, 8, -M_PI/2.0},
          Eigen::Vector3d{0, 0, 0});
    obstacle_1.insert(
          time + 25s,
          make_test_profile(UnitCircle),
          Eigen::Vector3d{-10, 0, -M_PI/2.0},
          Eigen::Vector3d{0,0,0});
    obstacle_1.insert(
          time + 50s,
          make_test_profile(UnitCircle),
          Eigen::Vector3d{-10, -8, -M_PI/2.0},
          Eigen::Vector3d{0, 0, 0});

    REQUIRE(obstacle_1.size() == 3);
    REQUIRE(DetectConflict::between(t,obstacle_1).size() > 0);

    WHEN("Obstacle 28->2")
    {
      obstacles.push_back(obstacle_1);

      const auto t_obs1 = test_with_obstacle(
            "Obstacle 28->2", plan, database, obstacles, 27, time);

      WHEN("Obstacle 28->2 , 29->4")
      {
        //robot waits 10s at 27 and then rotates on the spot at 13 for another 5s
        rmf_traffic::Trajectory obstacle_2(test_map_name);
        obstacle_2.insert(
              time,
              make_test_profile(UnitCircle),
              Eigen::Vector3d{3, 8,-M_PI_2},
              Eigen::Vector3d{0,0,0});
        obstacle_2.insert(
              time + 53s,
              make_test_profile(UnitCircle),
              Eigen::Vector3d{3, 0,-M_PI_2},
              Eigen::Vector3d{0,0,0});
        obstacle_2.insert(
              time + 60s,
              make_test_profile(UnitCircle),
              Eigen::Vector3d{3,-4,-M_PI_2},
              Eigen::Vector3d{0,0,0});

        REQUIRE(obstacle_2.size() == 3);
        REQUIRE(DetectConflict::between(t_obs1, obstacle_2).size() > 0);

        obstacles.push_back(obstacle_2);

        const auto t_obs2=test_with_obstacle(
               "Obstacle 28->2 , 29->4", plan, database, obstacles, 27, time);

        WHEN("Obstacle 28->2 , 29->4, 23->26")
        {
          //robot waits 10s at 27 and then rotates on the spot at 13, 16

          rmf_traffic::Trajectory obstacle_3(test_map_name);
          obstacle_3.insert(
                time + 50s,
                make_test_profile(UnitCircle),
                Eigen::Vector3d{6, 4, 0},
                Eigen::Vector3d{0, 0, 0});
          obstacle_3.insert(
                time + 85s,
                make_test_profile(UnitCircle),
                Eigen::Vector3d{9, 4, 0},
                Eigen::Vector3d{0, 0, 0});
          obstacle_3.insert(
                time + 95s,
                make_test_profile(UnitCircle),
                Eigen::Vector3d{18, 4, 0},
                Eigen::Vector3d{0 , 0 , 0});
          REQUIRE(obstacle_3.size() == 3);
          REQUIRE(DetectConflict::between(t_obs2, obstacle_3).size() > 0);

          obstacles.push_back(obstacle_3);
          //std::cout<<"Obstacle Size: "<<obstacles.size()<<std::endl;

          test_with_obstacle(
                "Obstacle 28->2 , 29->4, 23->26",
                plan, database, obstacles, 27, time);
        }
      }
    }
  }
}
