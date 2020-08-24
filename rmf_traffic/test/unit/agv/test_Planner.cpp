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

#include <rmf_traffic/DetectConflict.hpp>

#include <rmf_utils/catch.hpp>
#include <rmf_utils/math.hpp>

#include "../utils_Trajectory.hpp"

#include <iostream>
#include <iomanip>
#include <thread>

// TODO(MXG): Move performance testing content into a performance test folder
const bool test_performance = false;
// const bool test_performance = true;
const std::size_t N = test_performance ? 10 : 1;

void print_timing(const std::chrono::steady_clock::time_point& start_time)
{
  if (test_performance)
  {
    const auto finish_time = std::chrono::steady_clock::now();
    std::cout << Catch::getResultCapture().getCurrentTestName()
              << ": " << rmf_traffic::time::to_seconds(finish_time - start_time)
              << std::endl;
  }
}

rmf_utils::clone_ptr<rmf_traffic::agv::ScheduleRouteValidator>
make_test_schedule_validator(
  const rmf_traffic::schedule::Viewer& viewer,
  rmf_traffic::Profile profile)
{
  return rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
    viewer,
    std::numeric_limits<rmf_traffic::schedule::ParticipantId>::max(),
    std::move(profile));
}

void display_path(const rmf_traffic::agv::Plan::Result& plan)
{
  std::vector<std::size_t> plan_indices;
  for (const auto& wp : plan->get_waypoints())
  {
    if (wp.graph_index())
      plan_indices.push_back(*wp.graph_index());
  }
  auto ip = std::unique(plan_indices.begin(), plan_indices.end());
  plan_indices.resize(std::distance(plan_indices.begin(), ip));
  std::cout<<"Path: ";
  for (auto it = plan_indices.begin(); it != plan_indices.end(); it++)
  {
    std::cout<<*it;
    if (it != --plan_indices.end())
      std::cout<<"-> ";
  }
  std::cout<<std::endl;
}

void print_trajectory_info(
  const rmf_traffic::agv::Plan::Result& plan,
  rmf_traffic::Time time)
{
  display_path(plan);
  std::cout << "Itinerary count: " << plan->get_itinerary().size() 
            << std::endl;
  int trajectory_count = 1; 
  for (const auto& r : plan->get_itinerary())
  {
    int waypoint_count = 1;
    const auto& t = r.trajectory();
    std::cout << "Trajectory [" << trajectory_count << "] in " << r.map()
              << " with " << t.size() << " waypoints\n";
    for (auto it = t.begin(); it != t.end(); it++)
    {
      auto position = it->position();
      std::cout << "  Waypoint "<< waypoint_count << ": {" << position[0]
                << ","<<position[1] << "," << position[2] << "} "
                << rmf_traffic::time::to_seconds(it->time() - time) << "s"
                << std::endl;
      waypoint_count++;
    }
    trajectory_count++;
  }
}

rmf_traffic::Trajectory test_with_obstacle(
  const std::string& parent,
  const rmf_traffic::agv::Planner::Result& original_result,
  rmf_traffic::schedule::Database& database,
  const std::vector<rmf_traffic::Trajectory>& obstacles,
  const std::size_t hold_index,
  const rmf_traffic::Time time,
  const bool expect_conflict = true,
  const bool check_holding = true,
  const bool print_info = false)
{
  const rmf_traffic::Profile profile = create_test_profile(UnitCircle);

  rmf_traffic::Trajectory t_obs;
  const rmf_traffic::schedule::ParticipantId p_obs =
    database.register_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "obstacle",
      "test_Planner",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      create_test_profile(UnitCircle)
    });

  rmf_traffic::RouteId rid = 0;
  rmf_traffic::schedule::ItineraryVersion iv = 0;
  for (const auto& obstacle : obstacles)
  {
    const auto r = std::make_shared<rmf_traffic::Route>("test_map", obstacle);
    database.extend(p_obs, {{rid++, r}}, iv++);
  }

  rmf_utils::optional<rmf_traffic::agv::Planner::Result> result;
  const auto start_time = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < N; ++i)
  {
    result = original_result.replan(original_result->get_start());
    REQUIRE(*result);
  }

  const auto plan = **result;

  const auto end_time = std::chrono::steady_clock::now();
  if (test_performance)
  {
    const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
    std::cout << "\n" << parent << " w/ obstacle" << std::endl;
    std::cout << "Total: " << sec << std::endl;
    std::cout << "Per run: " << sec/N << std::endl;
  }

  const auto& graph = original_result.get_configuration().graph();

  REQUIRE(plan.get_itinerary().size() == 1);
  t_obs = plan.get_itinerary().front().trajectory();
  const Eigen::Vector2d initial_position = [&]() -> Eigen::Vector2d
    {
      if (original_result->get_start().location())
        return *original_result->get_start().location();

      const std::size_t start_index = original_result->get_start().waypoint();
      return graph.get_waypoint(start_index).get_location();
    } ();


  const std::size_t goal_index = original_result.get_goal().waypoint();
  const auto goal_position = graph.get_waypoint(goal_index).get_location();

  const Eigen::Vector2d p_initial =
    t_obs.front().position().block<2, 1>(0, 0);
  CHECK( (p_initial - initial_position).norm() == Approx(0.0) );

  const Eigen::Vector2d p_final =
    t_obs.back().position().block<2, 1>(0, 0);
  CHECK( (p_final - goal_position).norm() == Approx(0.0) );

  const auto& original_trajectory =
    original_result->get_itinerary().front().trajectory();
  if (expect_conflict)
  {
    CHECK(original_trajectory.duration() < t_obs.duration() );
  }
  else
  {
    const auto time_diff = original_trajectory.duration() - t_obs.duration();
    CHECK(std::abs(rmf_traffic::time::to_seconds(time_diff)) < 1e-8);
  }

  // Confirm that the trajectory does not conflict with anything in the
  // schedule
  const auto query = database.query(rmf_traffic::schedule::query_all());
  for (const auto& entry : query)
  {
    const auto& p_obs = database.get_participant(entry.participant)->profile();
    CHECK(!rmf_traffic::DetectConflict::between(
        profile, t_obs, p_obs, entry.route.trajectory()));
  }

  // Confirm that the vehicle pulled into holding point in order to avoid
  // the conflict
  if (check_holding)
  {
    bool used_holding_point = false;
    for (const auto& wp : plan.get_waypoints())
    {
      if (!wp.graph_index())
        continue;

      if (*wp.graph_index() == hold_index)
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
    print_trajectory_info(*result, time);
  }
  return t_obs;
}

void test_ignore_obstacle(
  const rmf_traffic::agv::Planner::Result& original_result,
  const rmf_traffic::schedule::Version database_version)
{
  REQUIRE(database_version > 0);
  const auto& start = original_result->get_start();
  rmf_traffic::agv::Plan::Options options = original_result.options();
  std::unordered_set<rmf_traffic::schedule::ParticipantId> ignore_ids;
  for (rmf_traffic::schedule::Version v = 0; v <= database_version; ++v)
    ignore_ids.insert(v);

  options.validator(nullptr);

  const auto new_plan = original_result.replan(start, std::move(options));

  // The new plan which ignores the conflicts should be the same as the original
  REQUIRE(new_plan->get_itinerary().size() == 1);
  CHECK(new_plan->get_itinerary().front().trajectory().duration()
    == original_result->get_itinerary().front().trajectory().duration());

  REQUIRE(new_plan->get_waypoints().size()
    == original_result->get_waypoints().size());

  for (std::size_t i = 0; i < new_plan->get_waypoints().size(); ++i)
  {
    const auto& new_wp = new_plan->get_waypoints()[i];
    const auto& old_wp = original_result->get_waypoints()[i];

    if (new_wp.graph_index())
    {
      CHECK(*new_wp.graph_index() == *old_wp.graph_index());
    }
    else
    {
      CHECK(!old_wp.graph_index());
    }

    const Eigen::Vector3d new_p = new_wp.position();
    const Eigen::Vector3d old_p = old_wp.position();
    CHECK( (new_p.block<2, 1>(0, 0) - old_p.block<2, 1>(0, 0)).norm()
      == Approx(0.0) );
    CHECK(rmf_utils::wrap_to_pi(new_p[2] - old_p[2]) == Approx(0.0) );
  }
}

inline void CHECK_TRAITS(
  const rmf_traffic::agv::VehicleTraits& t1,
  const rmf_traffic::agv::VehicleTraits& t2)
{
  CHECK((t1.get_differential()->get_forward()
    - t2.get_differential()->get_forward()).norm()
    == Approx(0).margin(1e-6));
  CHECK(t1.get_differential()->is_reversible()
    == t2.get_differential()->is_reversible());
  CHECK(t1.linear().get_nominal_acceleration()
    == t2.linear().get_nominal_acceleration());
  CHECK(t1.linear().get_nominal_velocity()
    == t2.linear().get_nominal_velocity());
  CHECK(t1.rotational().get_nominal_acceleration()
    == t2.rotational().get_nominal_acceleration());
  CHECK(t1.rotational().get_nominal_velocity()
    == t2.rotational().get_nominal_velocity());
}

inline void CHECK_INTERPOLATION(
  const rmf_traffic::agv::Interpolate::Options& o1,
  const rmf_traffic::agv::Interpolate::Options& o2)
{
  CHECK(o1.always_stop() == o2.always_stop());
  CHECK((o1.get_translation_threshold()-
    o2.get_translation_threshold())
    == Approx(0).margin(1e-6));
  CHECK((o1.get_rotation_threshold()-
    o2.get_rotation_threshold())
    == Approx(0).margin(1e-6));
  CHECK((o1.get_corner_angle_threshold()-
    o2.get_corner_angle_threshold())
    == Approx(0).margin(1e-6));
}

inline void CHECK_PLAN(
  const rmf_traffic::agv::Plan::Result& plan,
  const Eigen::Vector2d first_location,
  const double first_orientation,
  const Eigen::Vector2d last_location,
  const std::vector<std::size_t> wp_indices,
  const double* last_orientation = nullptr)
{
  REQUIRE(plan);
  REQUIRE(plan->get_itinerary().size() > 0);
  const auto& first_trajectory = plan->get_itinerary().front().trajectory();
  const auto& last_trajectory = plan->get_itinerary().back().trajectory();
  // check locations
  CHECK((first_trajectory.front().position().block<2, 1>(0, 0)
    - first_location).norm() == Approx(0.0).margin(1e-6));
  CHECK((last_trajectory.back().position().block<2, 1>(0, 0)
    - last_location).norm()  == Approx(0.0).margin(1e-6));
  // check orientations
  CHECK((first_trajectory.front().position()[2] - first_orientation)
    == Approx(0.0).margin(1e-6));
  if (last_orientation != nullptr)
    CHECK((last_trajectory.back().position()[2] - *last_orientation)
      == Approx(0.0).margin(1e-6));
  // check waypoints
  const auto& wps = plan->get_waypoints();
  // removing consecutive duplicates of waypoints
  // when robot is waiting at holding point
  std::vector<std::size_t> plan_indices;
  for (const auto& wp : wps)
  {
    if (wp.graph_index())
      plan_indices.push_back(*wp.graph_index());
  }

  auto ip = std::unique(plan_indices.begin(), plan_indices.end());
  plan_indices.resize(std::distance(plan_indices.begin(), ip));
  REQUIRE(plan_indices.size() == wp_indices.size());
  for (const auto& i : plan_indices)
    CHECK(std::find(
        wp_indices.begin(), wp_indices.end(), i)
      != wp_indices.end());
}
// ____________________________________________________________________________

SCENARIO("Test Configuration", "[config]")
{
  using namespace std::chrono_literals;
  using Graph = rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using Interpolate = rmf_traffic::agv::Interpolate;
  using Planner = rmf_traffic::agv::Planner;

  const std::string test_map_name = "test_map";
  Graph graph;
  graph.add_waypoint(test_map_name, {0, -5}); // 0
  REQUIRE(graph.num_waypoints() == 1);

  const VehicleTraits traits(
    {0.7, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));

  const Interpolate::Options options = Interpolate::Options();
  Planner::Configuration config(graph, traits, options);

  WHEN("Get the graph in config")
  {
    Graph& graph_ = config.graph();
    REQUIRE(graph_.num_waypoints() == graph.num_waypoints());
    for (std::size_t i = 0; i < graph.num_waypoints(); i++)
      CHECK((graph.get_waypoint(i).get_location() -
        graph_.get_waypoint(i).get_location()).norm()
        == Approx(0).margin(1e-6));
  }

  WHEN("Set the graph")
  {
    Graph& graph_ = config.graph();
    graph_.add_waypoint(test_map_name, {5, 5});
    REQUIRE(graph_.num_waypoints() == graph.num_waypoints() + 1);
    REQUIRE(config.graph().num_waypoints() == graph.num_waypoints() + 1);
    CHECK((config.graph().get_waypoint(graph.num_waypoints()).get_location()
      - Eigen::Vector2d{5, 5}).norm() == Approx(0).margin(1e-6));
  }

  WHEN("Get the vechile_traits")
  {
    VehicleTraits& traits_ = config.vehicle_traits();
    CHECK_TRAITS(traits_, traits);
  }

  WHEN("Set the vechile_traits")
  {
    VehicleTraits& traits_ = config.vehicle_traits();
    VehicleTraits traits_new(
      {1.0, 0.5}, {1.0, 0.6}, create_test_profile(UnitCircle));
    traits_ = traits_new;
    CHECK_TRAITS(config.vehicle_traits(), traits_);
  }

  WHEN("Get the interpolation")
  {
    Interpolate::Options& options_ = config.interpolation();
    CHECK_INTERPOLATION(options_, options);
  }

  WHEN("Set the interpolation")
  {
    Interpolate::Options& options_ = config.interpolation();
    options_ = Interpolate::Options(
      true, 1e-2, 5.0 * M_PI/180.0, 5.0 * M_PI/180.0);
    CHECK_INTERPOLATION(config.interpolation(), options_);
  }
}

SCENARIO("Test Options", "[options]")
{
  using namespace std::chrono_literals;
  using Planner = rmf_traffic::agv::Planner;
  using Duration = std::chrono::nanoseconds;

  auto interrupt_flag = std::make_shared<bool>(false);
  Duration hold_time = std::chrono::seconds(6);

  Planner::Options default_options(nullptr, hold_time, interrupt_flag);
  WHEN("Get the minimum_holding_time")
  {
    CHECK(rmf_traffic::time::to_seconds(
        default_options.minimum_holding_time()- hold_time)
      == Approx(0).margin(1e-6));
  }

  WHEN("Set the minimum_holding_time")
  {
    Duration hold_time_ = std::chrono::seconds(5);
    default_options.minimum_holding_time(hold_time_);
    CHECK(rmf_traffic::time::to_seconds(
        default_options.minimum_holding_time()- hold_time_)
      == Approx(0).margin(1e-6));
  }

  WHEN("Get the interrupt_flag")
  {
    CHECK_FALSE(*default_options.interrupt_flag());
  }

  WHEN("Set the interrupt_flag")
  {
    *interrupt_flag = true;
    CHECK(*default_options.interrupt_flag());
  }

}

SCENARIO("Test Start")
{
  using namespace std::chrono_literals;
  using Planner = rmf_traffic::agv::Planner;

  auto start_time = std::chrono::steady_clock::now();

  Planner::Start start(
    start_time,
    1,
    0.0);

  CHECK_FALSE(start.lane());
  CHECK_FALSE(start.location());

  CHECK(rmf_traffic::time::to_seconds(start.time()-start_time)
    == Approx(0.0).margin(1e-12));
  CHECK(start.waypoint() == 1);
  CHECK((start.orientation() - 0.0) == Approx(0.0).margin(1e-6));

  rmf_utils::optional<Eigen::Vector2d> initial_location = Eigen::Vector2d{0, 0};
  rmf_utils::optional<std::size_t> initial_lane = std::size_t(0);

  start.location(initial_location);
  CHECK(start.location());

  start.lane(initial_lane);
  CHECK(start.lane());
}

SCENARIO("Test Goal")
{
  using Planner = rmf_traffic::agv::Planner;
  Planner::Goal goal{1};
  CHECK(goal.orientation() == nullptr);
  CHECK(goal.waypoint() == 1);
  goal.waypoint(2);
  CHECK(goal.waypoint() == 2);

  goal.orientation(M_PI_2);
  CHECK((*goal.orientation() - M_PI_2) == Approx(0.0));

  goal = Planner::Goal{3, 0.0};
  CHECK(goal.waypoint() == 3);
  CHECK((*goal.orientation() - 0.0) == Approx(0.0));
}

SCENARIO("Test planning")
{
  using namespace std::chrono_literals;

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {-5, -5}).set_passthrough_point(true); // 0
  graph.add_waypoint(test_map_name, { 0, -5}).set_passthrough_point(true); // 1
  graph.add_waypoint(test_map_name, { 5, -5}).set_passthrough_point(true); // 2
  graph.add_waypoint(test_map_name, {10, -5}).set_passthrough_point(true); // 3
  graph.add_waypoint(test_map_name, {-5, 0}); // 4
  graph.add_waypoint(test_map_name, { 0, 0}); // 5
  graph.add_waypoint(test_map_name, { 5, 0}); // 6
  graph.add_waypoint(test_map_name, {10, 0}).set_passthrough_point(true); // 7
  graph.add_waypoint(test_map_name, {10, 4}).set_passthrough_point(true); // 8
  graph.add_waypoint(test_map_name, { 0, 8}).set_passthrough_point(true); // 9
  graph.add_waypoint(test_map_name, { 5, 8}).set_passthrough_point(true); // 10
  graph.add_waypoint(test_map_name, {10, 12}).set_passthrough_point(true); // 11
  graph.add_waypoint(test_map_name, {12, 12}).set_passthrough_point(true); // 12
  REQUIRE(graph.num_waypoints() == 13);

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

  const rmf_traffic::Profile profile = create_test_profile(UnitCircle);
  const rmf_traffic::agv::VehicleTraits traits(
    {0.7, 0.3}, {1.0, 0.45}, profile);

  rmf_traffic::schedule::Database database;

  const auto default_options = rmf_traffic::agv::Planner::Options{
    make_test_schedule_validator(database, profile)};

  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options
  };

  //TODO abort planning that is impossible as lane does not exit in the graph

  // WHEN("goal waypoint does not have a lane in the graph")
  // {
  //   const rmf_traffic::Time start_time = std::chrono::steady_clock::now();
  //   auto plan = planner.plan(
  //       rmf_traffic::agv::Planner::Start(start_time, 3, 0.0),
  //       rmf_traffic::agv::Planner::Goal(9));
  // }

  WHEN("initial conditions satisfy the goals")
  {
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();

    auto plan = planner.plan(
      rmf_traffic::agv::Planner::Start(start_time, 3, 0.0),
      rmf_traffic::agv::Planner::Goal(3, 0.0));

    REQUIRE(plan);
    CHECK(plan->get_itinerary().size() == 0);
    CHECK(plan->get_waypoints().size() == 1);
  }

  WHEN("initial and goal waypoints are same but goal_orientation is different")
  {
    rmf_utils::optional<rmf_traffic::agv::Plan::Result> result;
    const double goal_orientation = M_PI/2.0;
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();

    for (std::size_t i = 0; i < N; ++i)
    {
      result = planner.plan(
        rmf_traffic::agv::Planner::Start{start_time, 3, 0.0},
        rmf_traffic::agv::Planner::Goal{3, goal_orientation});

      REQUIRE(*result);
    }

    auto plan = *result;

    const auto end_time = std::chrono::steady_clock::now();
    if (test_performance)
    {
      const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
      std::cout << "\nUnconstrained" << std::endl;
      std::cout << "Total: " << sec << std::endl;
      std::cout << "Per run: " << sec/N << std::endl;
    }

    CHECK(plan->get_itinerary().size() == 1);
    REQUIRE(plan->get_itinerary().front().trajectory().size() > 0);
    const auto t = plan->get_itinerary().front().trajectory();
    const auto final_p = t.front().position().block<2, 1>(0, 0);
    const auto err = (final_p - Eigen::Vector2d(10, -5)).norm();
    CHECK(err == Approx(0.0) );
    CHECK(t.back().position()[2] - goal_orientation == Approx(0));
    CHECK(t.back().time() > start_time);
  }

  WHEN("goal waypoint is an adjacent node")
  {
    rmf_utils::optional<rmf_traffic::agv::Plan::Result> result;
    const double goal_orientation = M_PI;
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();

    for (std::size_t i = 0; i < N; ++i)
    {
      result = planner.plan(
        rmf_traffic::agv::Planner::Start{start_time, 3, M_PI},
        rmf_traffic::agv::Planner::Goal{2, goal_orientation});
      REQUIRE(*result);
    }

    auto plan = *result;

    const auto end_time = std::chrono::steady_clock::now();
    if (test_performance)
    {
      const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
      std::cout << "\nUnconstrained" << std::endl;
      std::cout << "Total: " << sec << std::endl;
      std::cout << "Per run: " << sec/N << std::endl;
    }

    const auto expected_t = rmf_traffic::agv::Interpolate::positions(
      traits, start_time, {{10.0, -5.0, M_PI}, {5.0, -5.0, M_PI}});

    REQUIRE(plan->get_itinerary().size() == 1);
    const rmf_traffic::Trajectory t =
      plan->get_itinerary().front().trajectory();
    REQUIRE(t.size() == expected_t.size());

    const Eigen::Vector2d initial_p = t.front().position().block<2, 1>(0, 0);
    CHECK((initial_p - Eigen::Vector2d(10, -5)).norm() == Approx(0.0) );

    const Eigen::Vector2d final_p = t.back().position().block<2, 1>(0, 0);
    CHECK((final_p - Eigen::Vector2d(5, -5)).norm() == Approx(0.0));

    CHECK(t.back().position()[2] - goal_orientation == Approx(0));
    CHECK(t.back().time() > start_time);
  }

  GIVEN("Goal from 12->5 and obstacle from 5->12")
  {
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const auto start = rmf_traffic::agv::Planner::Start{time, 12, 0.0};
    const auto goal = rmf_traffic::agv::Planner::Goal{5};

    std::vector<rmf_traffic::Trajectory> obstacles;

    rmf_traffic::Trajectory obstacle;
    obstacle.insert(
      time + 19s,
      {0.0, 8.0, 0.0},
      {0.0, 0.0, 0.0});
    obstacle.insert(
      time + 40s,
      {5.0, 8.0, 0.0},
      {0.0, 0.0, 0.0});
    obstacle.insert(
      time + 50s,
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

      const auto start_time = std::chrono::steady_clock::now();
      rmf_utils::optional<rmf_traffic::agv::Plan::Result> result;

      for (std::size_t i = 0; i < N; ++i)
      {
        result = planner.plan(start, goal);
        REQUIRE(*result);
      }

      auto plan = *result;

      const auto end_time = std::chrono::steady_clock::now();
      if (test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nUnconstrained 12->5" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan->get_itinerary().size() == 1);
      const auto t = plan->get_itinerary().front().trajectory();

      const Eigen::Vector2d initial_p = t.front().position().block<2, 1>(0, 0);
      CHECK( (initial_p - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );

      const Eigen::Vector2d final_p = t.back().position().block<2, 1>(0, 0);
      CHECK( (final_p - Eigen::Vector2d(0, 0)).norm() == Approx(0.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
          "Unconstrained 12->5", plan, database, obstacles, 6, time);

        test_ignore_obstacle(plan, database.latest_version());
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

      rmf_utils::optional<rmf_traffic::agv::Plan::Result> result;
      const auto start_time = std::chrono::steady_clock::now();
      for (std::size_t i = 0; i < N; ++i)
      {
        result = planner.plan(start, goal);
        REQUIRE(*result);
      }

      auto plan = *result;

      const auto end_time = std::chrono::steady_clock::now();
      if (test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nConstrained to 0.0  12->5" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan->get_itinerary().size() == 1);
      const auto& t = plan->get_itinerary().front().trajectory();

      const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
      CHECK( (p_initial - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );

      const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
      CHECK( (p_final - Eigen::Vector2d(0, 0)).norm() == Approx(0.0) );
      CHECK(t.back().position()[2] == Approx(M_PI/2.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
          "Constrained to 0.0  12->5", plan, database, obstacles, 6, time);

        test_ignore_obstacle(plan, database.latest_version());
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

    rmf_traffic::Trajectory obstacle;
    obstacle.insert(
      time + 24s,
      {0.0, 8.0, 0.0},
      {0.0, 0.0, 0.0});
    obstacle.insert(
      time + 50s,
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0});
    obstacle.insert(
      time + 70s,
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

      rmf_utils::optional<rmf_traffic::agv::Plan::Result> result;
      const auto start_time = std::chrono::steady_clock::now();

      for (std::size_t i = 0; i < N; ++i)
      {
        result = planner.plan(start, goal);
        REQUIRE(*result);
      }

      auto plan = *result;

      const auto end_time = std::chrono::steady_clock::now();
      if (test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nUnconstrained 2->12" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan->get_itinerary().size() == 1);
      const auto t = plan->get_itinerary().front().trajectory();

      const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
      CHECK( (p_initial - Eigen::Vector2d(5, -5)).norm() == Approx(0.0) );

      const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
      CHECK( (p_final - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
          "Unconstrained  2->12", plan, database, obstacles, 4, time);

        test_ignore_obstacle(plan, database.latest_version());
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

      rmf_utils::optional<Planner::Result> result;
      const auto start_time = std::chrono::steady_clock::now();

      for (std::size_t i = 0; i < N; ++i)
      {
        result = planner.plan(start, goal);
        REQUIRE(*result);
      }

      auto plan = *result;

      const auto end_time = std::chrono::steady_clock::now();
      if (test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nConstrained to 0.0  2->12" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan->get_itinerary().size() == 1);
      const auto& t = plan->get_itinerary().front().trajectory();

      const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
      CHECK( (p_initial - Eigen::Vector2d(5, -5)).norm() == Approx(0.0) );

      const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
      CHECK( (p_final - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );
      CHECK(t.back().position()[2] == Approx(0.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
          "Constrained to 0.0  2->12", plan, database, obstacles, 4, time);

        test_ignore_obstacle(plan, database.latest_version());
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

      rmf_utils::optional<rmf_traffic::agv::Plan::Result> result;
      const auto start_time = std::chrono::steady_clock::now();
      for (std::size_t i = 0; i < N; ++i)
      {
        result = planner.plan(start, goal);
        REQUIRE(*result);
      }

      auto plan = *result;

      const auto end_time = std::chrono::steady_clock::now();
      if (test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nConstrained to 180.0  2->12" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan->get_itinerary().size() == 1);
      const auto& t = plan->get_itinerary().front().trajectory();

      const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
      CHECK( (p_initial - Eigen::Vector2d(5, -5)).norm() == Approx(0.0) );

      const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
      CHECK( (p_final - Eigen::Vector2d(12, 12)).norm() == Approx(0.0) );

      const double err = rmf_utils::wrap_to_pi(
        t.back().position()[2] - M_PI);
      CHECK(err == Approx(0.0) );

      WHEN("An obstacle is introduced")
      {
        test_with_obstacle(
          "Constrained to 180.0  2->12",
          plan, database, obstacles, 4, time);

        test_ignore_obstacle(plan, database.latest_version());
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

    rmf_traffic::Trajectory obstacle_1;
    obstacle_1.insert(
      time + 19s,
      {0.0, 8.0, 0.0},
      {0.0, 0.0, 0.0});
    obstacle_1.insert(
      time + 40s,
      {5.0, 8.0, 0.0},
      {0.0, 0.0, 0.0});
    obstacle_1.insert(
      time + 50s,
      {10.0, 12.0, 0.0},
      {0.0, 0.0, 0.0});
    REQUIRE(obstacle_1.size() == 3);

    WHEN("Docking is not constrained")
    {
      using namespace rmf_traffic::agv;
      add_bidir_lane(5, 9);
      add_bidir_lane(11, 12);

      planner = Planner{Planner::Configuration{graph, traits}, default_options};

      rmf_utils::optional<rmf_traffic::agv::Plan::Result> result;

      const auto start_time = std::chrono::steady_clock::now();
      for (std::size_t i = 0; i < N; ++i)
      {
        result = planner.plan(start, goal);
        REQUIRE(*result);
      }

      auto plan = *result;

      const auto end_time = std::chrono::steady_clock::now();
      if (test_performance)
      {
        const double sec = rmf_traffic::time::to_seconds(end_time - start_time);
        std::cout << "\nUnconstrained  12->0" << std::endl;
        std::cout << "Total: " << sec << std::endl;
        std::cout << "Per run: " << sec/N << std::endl;
      }

      REQUIRE(plan->get_itinerary().size() == 1);
      const auto t = plan->get_itinerary().front().trajectory();

      const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
      const Eigen::Vector2d p_initial_g =
        graph.get_waypoint(start_index).get_location();
      CHECK( (p_initial - p_initial_g).norm() == Approx(0.0) );

      const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
      const Eigen::Vector2d p_final_g =
        graph.get_waypoint(goal_index).get_location();
      CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

      WHEN("First obstacle is introduced")
      {
        CHECK(rmf_traffic::DetectConflict::between(
            profile, t, profile, obstacle_1));
        obstacles.push_back(obstacle_1);

        test_with_obstacle(
          "Unconstrained (1)  12->0", plan, database, obstacles, 6, time);

        test_ignore_obstacle(plan, database.latest_version());
      }

      WHEN("Second obstacle is introduced")
      {
        rmf_traffic::Trajectory obstacle_2;
        obstacle_2.insert(
          time + 49s,
          {0.0, -5.0, M_PI_2},
          {0.0, 0.0, 0.0});
        obstacle_2.insert(
          time + 60s,
          {0.0, 0.0, M_PI_2},
          {0.0, 0.0, 0.0});
        obstacle_2.insert(
          time + 87s,
          {0.0, 8.0, M_PI_2},
          {0.0, 0.0, 0.0});
        REQUIRE(obstacle_2.size() == 3);
        REQUIRE_FALSE(rmf_traffic::DetectConflict::between(
            profile, obstacle_1, profile, obstacle_2));
        CHECK(rmf_traffic::DetectConflict::between(
            profile, t, profile, obstacle_2));

        obstacles.push_back(obstacle_2);
        test_with_obstacle(
          "Unconstrained (2)  12->0", plan, database, obstacles, 4, time);

        test_ignore_obstacle(plan, database.latest_version());
      }

      WHEN("Both obstacles are introduced")
      {
        rmf_traffic::Trajectory obstacle_2;
        obstacle_2.insert(
          time + 81s,
          {0.0, -5.0, 0.0},
          {0.0, 0.0, 0.0});
        obstacle_2.insert(
          time + 92s,
          {0.0, 0.0, 0.0},
          {0.0, 0.0, 0.0});
        obstacle_2.insert(
          time + 110s,
          {0.0, 8.0, 0.0},
          {0.0, 0.0, 0.0});
        REQUIRE(obstacle_2.size() == 3);
        obstacles.push_back(obstacle_1);
        obstacles.push_back(obstacle_2);

        test_with_obstacle(
          "Unconstrained (3)  12->0", plan, database, obstacles, 6, time);

        test_ignore_obstacle(plan, database.latest_version());
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
  graph.add_waypoint(test_map_name, {12, -12}).set_passthrough_point(true); // 0
  graph.add_waypoint(test_map_name, {18, -12}).set_holding_point(true); // 1
  graph.add_waypoint(test_map_name, {-10, -8}).set_passthrough_point(true); // 2
  graph.add_waypoint(test_map_name, {-2, -8}).set_holding_point(true);  // 3
  graph.add_waypoint(test_map_name, { 3, -8}).set_passthrough_point(true); // 4
  graph.add_waypoint(test_map_name, {12, -8}).set_passthrough_point(true); // 5
  graph.add_waypoint(test_map_name, {18, -8}).set_holding_point(true); // 6
  graph.add_waypoint(test_map_name, {-15, -4}).set_holding_point(true); // 7
  graph.add_waypoint(test_map_name, {-10, -4}).set_passthrough_point(true); // 8
  graph.add_waypoint(test_map_name, { -2, -4}).set_holding_point(true); // 9
  graph.add_waypoint(test_map_name, { 3, -4}).set_passthrough_point(true); // 10
  graph.add_waypoint(test_map_name, {6, -4}).set_passthrough_point(true); // 11
  graph.add_waypoint(test_map_name, {9, -4}).set_passthrough_point(true); // 12
  graph.add_waypoint(test_map_name, {-15, 0}).set_passthrough_point(true); // 13
  graph.add_waypoint(test_map_name, {-10, 0}).set_passthrough_point(true); // 14
  graph.add_waypoint(test_map_name, { 0, 0}).set_passthrough_point(true); // 15 DOOR (not implemented)
  graph.add_waypoint(test_map_name, { 3, 0}).set_passthrough_point(true); // 16
  graph.add_waypoint(test_map_name, {6, 0}).set_passthrough_point(true); // 17
  graph.add_waypoint(test_map_name, {9, 0}).set_passthrough_point(true); // 18
  graph.add_waypoint(test_map_name, {15, 0}).set_holding_point(true);   // 19
  graph.add_waypoint(test_map_name, {18, 0}).set_holding_point(true);   // 20
  graph.add_waypoint(test_map_name, { -2, 4}).set_holding_point(true);  // 21
  graph.add_waypoint(test_map_name, { 3, 4}).set_passthrough_point(true); // 22
  graph.add_waypoint(test_map_name, {6, 4}).set_passthrough_point(true); // 23
  graph.add_waypoint(test_map_name, {9, 4}).set_passthrough_point(true); // 24
  graph.add_waypoint(test_map_name, {15, 4}).set_passthrough_point(true); // 25
  graph.add_waypoint(test_map_name, {18, 4}).set_passthrough_point(true); // 26
  graph.add_waypoint(test_map_name, { -15, 8}).set_holding_point(true); // 27
  graph.add_waypoint(test_map_name, {-10, 8}).set_holding_point(true);  // 28
  graph.add_waypoint(test_map_name, {3, 8}).set_holding_point(true);    // 29
  graph.add_waypoint(test_map_name, {6, 8}).set_holding_point(true);    // 30
  graph.add_waypoint(test_map_name, {15, 8}).set_holding_point(true);  // 31
  graph.add_waypoint(test_map_name, {18, 8}).set_holding_point(true);  // 32

  REQUIRE(graph.num_waypoints() == 33);

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

  const rmf_traffic::Profile profile = create_test_profile(UnitCircle);
  rmf_traffic::schedule::Database database;
  const rmf_traffic::agv::VehicleTraits traits{
    {1.0, 0.4},
    {1.0, 0.5},
    profile
  };
  const rmf_traffic::Time time = std::chrono::steady_clock::now();
  const auto interrupt_flag = std::make_shared<bool>(false);
  const rmf_traffic::agv::Planner::Options default_options{
    make_test_schedule_validator(database, profile),
    std::chrono::seconds(5),
    interrupt_flag};

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

    CHECK(plan->get_itinerary().size() == 1);
    const auto t = plan->get_itinerary().front().trajectory();

    const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_initial_g =
      graph.get_waypoint(start_index).get_location();
    CHECK( (p_initial - p_initial_g).norm() == Approx(0.0));

    const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_final_g =
      graph.get_waypoint(goal_index).get_location();
    CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

    WHEN("Obstacle 28->3 that partially overlaps in time")
    {
      rmf_traffic::Trajectory obstacle_1;
      obstacle_1.insert(
        time,
        Eigen::Vector3d{-10, 8, -M_PI_2},
        Eigen::Vector3d{0, 0, 0});
      obstacle_1.insert(
        time + 20s,
        Eigen::Vector3d{-10, -8, -M_PI_2},
        Eigen::Vector3d{0, 0, 0});
      obstacle_1.insert(
        time + 25s,
        Eigen::Vector3d{-10, -8, 0},
        Eigen::Vector3d{0, 0, 0});
      obstacle_1.insert(
        time + 35s,
        Eigen::Vector3d{-2, -8, 0},
        Eigen::Vector3d{0, 0, 0});

      REQUIRE_FALSE(rmf_traffic::DetectConflict::between(
          profile, obstacle_1, profile, t));
      obstacles.push_back(obstacle_1);

      test_with_obstacle(
        "Partial 28->3", plan, database, obstacles, 0, time, false);

      test_ignore_obstacle(plan, database.latest_version());

      WHEN("Obstacle 28->3, 16-29 added")
      {
        rmf_traffic::Trajectory obstacle_2;
        obstacle_2.insert(
          time+20s,
          Eigen::Vector3d{3, 0, M_PI_2},
          Eigen::Vector3d{0, 0, 0});
        obstacle_2.insert(
          time+30s,
          Eigen::Vector3d{3, 8, M_PI_2},
          Eigen::Vector3d{0, 0, 0});

        const auto view = database.query(rmf_traffic::schedule::query_all());
        for (const auto& _t : view)
        {
          REQUIRE_FALSE(DetectConflict::between(
              profile, obstacle_2, profile, _t.route.trajectory()));
        }

        REQUIRE_FALSE(DetectConflict::between(profile, obstacle_2, profile, t));
        obstacles.push_back(obstacle_2);
        test_with_obstacle(
          "Partial 28->3, 16-29",
          plan, database, obstacles, 0, time, false);

        test_ignore_obstacle(plan, database.latest_version());

        WHEN("Obstacle 28->3, 16-29, 24->26 added")
        {
          rmf_traffic::Trajectory obstacle_3;
          obstacle_3.insert(
            time+40s,
            Eigen::Vector3d{9, 4, 0},
            Eigen::Vector3d{0, 0, 0});
          obstacle_3.insert(
            time+60s,
            Eigen::Vector3d{18, 4, 0},
            Eigen::Vector3d{0, 0, 0});

          const auto view =
            database.query(rmf_traffic::schedule::query_all());
          for (const auto& _t : view)
          {
            REQUIRE_FALSE(DetectConflict::between(
                profile, obstacle_3, profile, _t.route.trajectory()));
          }

          REQUIRE_FALSE(DetectConflict::between(
              profile, obstacle_3, profile, t));
          obstacles.push_back(obstacle_3);

          test_with_obstacle(
            "Partial 28->3, 16-29, 24->26", plan, database,
            obstacles, 0, time, false);

          test_ignore_obstacle(plan, database.latest_version());

          WHEN("Obstacle 28->3, 16-29, 24->26, 21->22, 13->14, 5->6 added")
          {
            rmf_traffic::Trajectory obstacle_4;
            obstacle_4.insert(
              time + 10s,
              Eigen::Vector3d{-2, -4, 0},
              Eigen::Vector3d{0, 0, 0});
            obstacle_4.insert(
              time + 20s,
              Eigen::Vector3d{3, 4, 0},
              Eigen::Vector3d{0, 0, 0});

            const auto view =
              database.query(rmf_traffic::schedule::query_all());
            for (const auto& _t : view)
              REQUIRE_FALSE(DetectConflict::between(
                  profile, obstacle_4, profile, _t.route.trajectory()));

            REQUIRE_FALSE(DetectConflict::between(
                profile, obstacle_4, profile, t));
            obstacles.push_back(obstacle_4);

            rmf_traffic::Trajectory obstacle_5;
            obstacle_5.insert(
              time + 15s,
              Eigen::Vector3d{-15, 0, 0},
              Eigen::Vector3d{0, 0, 0});

            obstacle_5.insert(
              time + 45s,
              Eigen::Vector3d{-10, 0, 0},
              Eigen::Vector3d{0, 0, 0});

            for (const auto& _t : view)
            {
              REQUIRE_FALSE(DetectConflict::between(
                  profile, obstacle_5, profile, _t.route.trajectory()));
            }

            REQUIRE_FALSE(DetectConflict::between(
                profile, obstacle_5, profile, t));
            obstacles.push_back(obstacle_5);

            rmf_traffic::Trajectory obstacle_6;
            obstacle_6.insert(
              time + 60s,
              Eigen::Vector3d{-12, -8, 0},
              Eigen::Vector3d{0, 0, 0});
            obstacle_6.insert(
              time + 75s,
              Eigen::Vector3d{-18, -8, 0},
              Eigen::Vector3d{0, 0, 0});

            for (const auto& _t : view)
            {
              REQUIRE_FALSE(DetectConflict::between(
                  profile, obstacle_6, profile, _t.route.trajectory()));
            }

            REQUIRE_FALSE(DetectConflict::between(
                profile, obstacle_6, profile, t));
            obstacles.push_back(obstacle_6);

            test_with_obstacle(
              "Partial 28->3, 16-29, 24->26, 21->22, 13->14, 5->6",
              plan, database, obstacles, 0, time, false);

            test_ignore_obstacle(plan, database.latest_version());
          }
        }
      }
    }
  }

  WHEN(
    "Robot moves from 1->30 given multiple non-conflicting obstacles that fully overlap in time")
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

    CHECK(plan->get_itinerary().size() == 1);
    auto t = plan->get_itinerary().front().trajectory();

    const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_initial_g =
      graph.get_waypoint(start_index).get_location();
    CHECK( (p_initial - p_initial_g).norm() == Approx(0.0) );

    const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_final_g =
      graph.get_waypoint(goal_index).get_location();
    CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

    WHEN("Obstacle 28->3 that partially overlaps in time")
    {
      rmf_traffic::Trajectory obstacle_1;
      obstacle_1.insert(
        time,
        Eigen::Vector3d{-10, 8, -M_PI_2},
        Eigen::Vector3d{0, 0, 0});
      obstacle_1.insert(
        time + 30s,
        Eigen::Vector3d{-10, -8, -M_PI_2},
        Eigen::Vector3d{0, 0, 0});
      obstacle_1.insert(
        time + 50s,
        Eigen::Vector3d{-10, -8, 0},
        Eigen::Vector3d{0, 0, 0});
      obstacle_1.insert(
        time + 76s,
        Eigen::Vector3d{-2, -8, 0},
        Eigen::Vector3d{0, 0, 0});

      REQUIRE_FALSE(DetectConflict::between(profile, obstacle_1, profile, t));
      obstacles.push_back(obstacle_1);
      test_with_obstacle(
        "Full 28->3", plan, database, obstacles, 0, time, false);

      test_ignore_obstacle(plan, database.latest_version());

      WHEN("Obstacle 28->3, 16-29 added")
      {
        rmf_traffic::Trajectory obstacle_2;
        obstacle_2.insert(
          time,
          Eigen::Vector3d{3, 0, M_PI_2},
          Eigen::Vector3d{0, 0, 0});
        obstacle_2.insert(
          time+76s,
          Eigen::Vector3d{3, 8, M_PI_2},
          Eigen::Vector3d{0, 0, 0});

        const auto view = database.query(rmf_traffic::schedule::query_all());
        for (const auto& _t : view)
        {
          REQUIRE_FALSE(DetectConflict::between(
              profile, obstacle_2, profile, _t.route.trajectory()));
        }

        REQUIRE_FALSE(DetectConflict::between(profile, obstacle_2, profile, t));
        obstacles.push_back(obstacle_2);
        test_with_obstacle(
          "Full 28->3, 16-29", plan, database, obstacles, 0, time, false);

        test_ignore_obstacle(plan, database.latest_version());

        WHEN("Obstacle 28->3, 16-29, 24->26 added")
        {
          rmf_traffic::Trajectory obstacle_3;
          obstacle_3.insert(
            time,
            Eigen::Vector3d{9, 4, 0},
            Eigen::Vector3d{0, 0, 0});
          obstacle_3.insert(
            time + 76s,
            Eigen::Vector3d{18, 4, 0},
            Eigen::Vector3d{0, 0, 0});

          const auto view = database.query(rmf_traffic::schedule::query_all());
          for (const auto& _t : view)
          {
            REQUIRE_FALSE(DetectConflict::between(
                profile, obstacle_3, profile, _t.route.trajectory()));
          }

          REQUIRE_FALSE(DetectConflict::between(
              profile, obstacle_3, profile, t));
          obstacles.push_back(obstacle_3);

          test_with_obstacle(
            "Full 28->3, 16-29, 24->26",
            plan, database, obstacles, 0, time, false);

          test_ignore_obstacle(plan, database.latest_version());

          WHEN("Obstacle 28->3, 16-29, 24->26, 21->22, 13->14, 5->6 added")
          {
            rmf_traffic::Trajectory obstacle_4;
            obstacle_4.insert(
              time,
              Eigen::Vector3d{-2, 4, 0},
              Eigen::Vector3d{0, 0, 0});
            obstacle_4.insert(
              time + 76s,
              Eigen::Vector3d{3, 4, 0},
              Eigen::Vector3d{0, 0, 0});

            const auto view =
              database.query(rmf_traffic::schedule::query_all());
            for (const auto& _t : view)
            {
              REQUIRE_FALSE(DetectConflict::between(
                  profile, obstacle_4, profile, _t.route.trajectory()));
            }

            REQUIRE_FALSE(DetectConflict::between(
                profile, obstacle_4, profile, t));
            obstacles.push_back(obstacle_4);

            rmf_traffic::Trajectory obstacle_5;
            obstacle_5.insert(
              time,
              Eigen::Vector3d{-15, 0, 0},
              Eigen::Vector3d{0, 0, 0});
            obstacle_5.insert(
              time + 76s,
              Eigen::Vector3d{-10, 0, 0},
              Eigen::Vector3d{0, 0, 0});

            for (const auto& _t : view)
            {
              REQUIRE_FALSE(DetectConflict::between(
                  profile, obstacle_5, profile, _t.route.trajectory()));
            }

            REQUIRE_FALSE(DetectConflict::between(
                profile, obstacle_5, profile, t));
            obstacles.push_back(obstacle_5);

            rmf_traffic::Trajectory obstacle_6;
            obstacle_6.insert(
              time,
              Eigen::Vector3d{-12, -8, 0},
              Eigen::Vector3d{0, 0, 0});
            obstacle_6.insert(
              time + 76s,
              Eigen::Vector3d{-18, -8, 0},
              Eigen::Vector3d{0, 0, 0});

            for (const auto& _t : view)
            {
              REQUIRE_FALSE(DetectConflict::between(
                  profile, obstacle_6, profile, _t.route.trajectory()));
            }

            REQUIRE_FALSE(DetectConflict::between(
                profile, obstacle_6, profile, t));
            obstacles.push_back(obstacle_6);

            test_with_obstacle(
              "Full 28->3, 16-29, 24->26, 2->3, 13->14, 5->6",
              plan, database, obstacles, 0, time, false);

            test_ignore_obstacle(plan, database.latest_version());
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

    CHECK(plan->get_itinerary().size() == 1);
    const auto t = plan->get_itinerary().front().trajectory();

    const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_initial_g =
      graph.get_waypoint(start_index).get_location();
    CHECK( (p_initial - p_initial_g).norm() == Approx(0.0) );

    const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_final_g =
      graph.get_waypoint(goal_index).get_location();
    CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

    rmf_traffic::Trajectory obstacle;
    obstacle.insert(
      time + 6s,
      Eigen::Vector3d{6, 4, 0},
      Eigen::Vector3d{0, 0, 0});
    obstacle.insert(
      time + 16s,
      Eigen::Vector3d{18, 4, 0},
      Eigen::Vector3d{0, 0, 0});
    obstacle.insert(
      time + 26s,
      Eigen::Vector3d{18, 0, 0},
      Eigen::Vector3d{0, 0, 0});

    WHEN("First obstacle is introduced")
    {
      REQUIRE_FALSE(!rmf_traffic::DetectConflict::between(
          profile, obstacle, profile, t));
      obstacles.push_back(obstacle);
      test_with_obstacle("Unconstrained", plan, database, obstacles, 32, time);

      test_ignore_obstacle(plan, database.latest_version());
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

    CHECK(plan->get_itinerary().size() == 1);
    auto t = plan->get_itinerary().front().trajectory();

    const Eigen::Vector2d p_initial = t.front().position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_initial_g =
      graph.get_waypoint(start_index).get_location();
    CHECK( (p_initial - p_initial_g).norm() == Approx(0.0) );

    const Eigen::Vector2d p_final = t.back().position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_final_g =
      graph.get_waypoint(goal_index).get_location();
    CHECK( (p_final - p_final_g).norm() == Approx(0.0) );

    rmf_traffic::Trajectory obstacle_1;
    obstacle_1.insert(
      time,
      Eigen::Vector3d{-10, 8, -M_PI/2.0},
      Eigen::Vector3d{0, 0, 0});
    obstacle_1.insert(
      time + 25s,
      Eigen::Vector3d{-10, 0, -M_PI/2.0},
      Eigen::Vector3d{0, 0, 0});
    obstacle_1.insert(
      time + 50s,
      Eigen::Vector3d{-10, -8, -M_PI/2.0},
      Eigen::Vector3d{0, 0, 0});

    REQUIRE(obstacle_1.size() == 3);
    REQUIRE(DetectConflict::between(profile, t, profile, obstacle_1));

    WHEN("Planning is interrupted")
    {
      const std::size_t start_index = 27;
      const auto start = rmf_traffic::agv::Plan::Start{time, start_index, 0.0};

      const std::size_t goal_index = 32;
      const auto goal = rmf_traffic::agv::Plan::Goal{goal_index};

      rmf_utils::optional<rmf_traffic::agv::Plan::Result> result;
      auto plan_thread = std::thread(
        [&]()
        {
          result = planner.plan(start, goal);
        });
      *interrupt_flag = true;
      plan_thread.join();
      CHECK_FALSE(*result);
      CHECK(result->interrupted());

      THEN("Plan can resume and find a solution")
      {
        const auto new_interrupt_flag = std::make_shared<bool>(false);
        result->resume(new_interrupt_flag);
        CHECK(*result);
      }
    }

    WHEN("Obstacle 28->2")
    {
      obstacles.push_back(obstacle_1);

      const auto t_obs1 = test_with_obstacle(
        "Obstacle 28->2", plan, database, obstacles, 27, time);

      test_ignore_obstacle(plan, database.latest_version());

      WHEN("Obstacle 28->2 , 29->4")
      {
        //robot waits 10s at 27 and then rotates on the spot at 13 for another 5s
        rmf_traffic::Trajectory obstacle_2;
        obstacle_2.insert(
          time,
          Eigen::Vector3d{3, 8, -M_PI_2},
          Eigen::Vector3d{0, 0, 0});
        obstacle_2.insert(
          time + 53s,
          Eigen::Vector3d{3, 0, -M_PI_2},
          Eigen::Vector3d{0, 0, 0});
        obstacle_2.insert(
          time + 60s,
          Eigen::Vector3d{3, -4, -M_PI_2},
          Eigen::Vector3d{0, 0, 0});

        CHECK(obstacle_2.size() == 3);
        CHECK(DetectConflict::between(profile, t_obs1, profile, obstacle_2));

        obstacles.push_back(obstacle_2);

        const auto t_obs2 = test_with_obstacle(
          "Obstacle 28->2 , 29->4", plan, database, obstacles, 27, time);

        test_ignore_obstacle(plan, database.latest_version());

        WHEN("Obstacle 28->2 , 29->4, 23->26")
        {
          //robot waits 10s at 27 and then rotates on the spot at 13, 16

          rmf_traffic::Trajectory obstacle_3;
          obstacle_3.insert(
            time + 50s,
            Eigen::Vector3d{6, 4, 0},
            Eigen::Vector3d{0, 0, 0});
          obstacle_3.insert(
            time + 85s,
            Eigen::Vector3d{9, 4, 0},
            Eigen::Vector3d{0, 0, 0});
          obstacle_3.insert(
            time + 95s,
            Eigen::Vector3d{18, 4, 0},
            Eigen::Vector3d{0, 0, 0});
          CHECK(obstacle_3.size() == 3);
          CHECK(DetectConflict::between(profile, t_obs2, profile, obstacle_3));

          obstacles.push_back(obstacle_3);
          //std::cout<<"Obstacle Size: "<<obstacles.size()<<std::endl;

          test_with_obstacle(
            "Obstacle 28->2 , 29->4, 23->26",
            plan, database, obstacles, 27, time);

          test_ignore_obstacle(plan, database.latest_version());
        }
      }
    }
  }
}

std::size_t count_events(const rmf_traffic::agv::Plan& plan)
{
  std::size_t count = 0;
  for (const auto& wp : plan.get_waypoints())
  {
    if (wp.event())
      ++count;
  }

  return count;
}

class ExpectEvent : public rmf_traffic::agv::Graph::Lane::Executor
{
public:

  enum Expectation
  {
    DoorOpen,
    DoorClose,
    LiftDoorOpen,
    LiftDoorClose,
    LiftMove,
    Dock,
    Wait
  };

  using Lane = rmf_traffic::agv::Graph::Lane;

  ExpectEvent(Expectation e)
  : _expectation(e),
    _result(false)
  {
    // Do nothing
  }

  void execute(const Lane::DoorOpen&) final
  {
    _result = _expectation == DoorOpen;
  }

  void execute(const Lane::DoorClose&) final
  {
    _result = _expectation == DoorClose;
  }

  void execute(const Lane::LiftDoorOpen&) final
  {
    _result = _expectation == LiftDoorOpen;
  }

  void execute(const Lane::LiftDoorClose&) final
  {
    _result = _expectation == LiftDoorClose;
  }

  void execute(const Lane::LiftMove&) final
  {
    _result = _expectation == LiftMove;
  }

  void execute(const Lane::Dock&) final
  {
    _result = _expectation == Dock;
  }

  void execute(const Lane::Wait& wait) final
  {
    _result = _expectation == Wait;
  }

  bool result() const
  {
    return _result;
  }

private:

  Expectation _expectation;
  bool _result;

};

bool has_event(
  ExpectEvent::Expectation expectation,
  const rmf_traffic::agv::Plan& plan)
{
  ExpectEvent executor(expectation);
  for (const auto& wp : plan.get_waypoints())
  {
    if (wp.event() && wp.event()->execute(executor).result())
      return true;
  }

  return false;
}

SCENARIO("Graph with door", "[door]")
{
  using namespace std::chrono_literals;
  using rmf_traffic::agv::Graph;
  using Event = Graph::Lane::Event;
  using DoorOpen = Graph::Lane::DoorOpen;
  using DoorClose = Graph::Lane::DoorClose;

  const std::string test_map_name = "test_map";
  Graph graph;
  graph.add_waypoint(test_map_name, {  0, 0}); // 0
  graph.add_waypoint(test_map_name, {  0, 0}); // 1
  graph.add_waypoint(test_map_name, {  0, 0}); // 2
  graph.add_waypoint(test_map_name, {  5, 0}); // 3
  graph.add_waypoint(test_map_name, { 10, 0}); // 4
  CHECK(graph.num_waypoints() == 5);

  graph.add_lane(0, 3);
  graph.add_lane({1, Event::make(DoorOpen("door", 5s))}, 3);
  graph.add_lane(
    {2, Event::make(DoorOpen("door", 4s))},
    {3, Event::make(DoorClose("door", 4s))});
  graph.add_lane(3, 4);

  const rmf_traffic::agv::VehicleTraits traits(
    {0.7, 0.3}, {1.0, 0.45}, create_test_profile(UnitCircle));

  rmf_traffic::schedule::Database database;

  const auto default_options = rmf_traffic::agv::Planner::Options{
    make_test_schedule_validator(database, traits.profile())};

  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options
  };

  const rmf_traffic::Time start_time = std::chrono::steady_clock::now();

  const auto plan_no_door = planner.plan(
    rmf_traffic::agv::Planner::Start(start_time, 0, 0.0),
    rmf_traffic::agv::Planner::Goal(4));
  REQUIRE(plan_no_door);
  REQUIRE(plan_no_door->get_itinerary().size() == 1);
  CHECK(count_events(*plan_no_door) == 0);

  const auto plan_with_door_open = planner.plan(
    rmf_traffic::agv::Planner::Start(start_time, 1, 0.0),
    rmf_traffic::agv::Planner::Goal(4));
  REQUIRE(plan_with_door_open);
  REQUIRE(plan_with_door_open->get_itinerary().size() == 1);
  CHECK(count_events(*plan_with_door_open) == 1);
  CHECK(has_event(ExpectEvent::DoorOpen, *plan_with_door_open));

  const auto t_with_door_open =
    plan_with_door_open->get_itinerary().front().trajectory().duration();
  const auto t_no_door =
    plan_no_door->get_itinerary().front().trajectory().duration();
  CHECK(rmf_traffic::time::to_seconds(t_with_door_open - t_no_door)
    == Approx(5.0).margin(1e-12));

  const auto plan_with_door_open_close = planner.plan(
    rmf_traffic::agv::Planner::Start(start_time, 2, 0.0),
    rmf_traffic::agv::Planner::Goal(4));
  REQUIRE(plan_with_door_open_close);
  REQUIRE(plan_with_door_open_close->get_itinerary().size() == 1);
  CHECK(count_events(*plan_with_door_open_close) == 2);

  const auto t_with_door_open_close =
    plan_with_door_open_close->get_itinerary().front().trajectory().duration();
  CHECK(rmf_traffic::time::to_seconds(t_with_door_open_close - t_no_door)
    > rmf_traffic::time::to_seconds(8s));
  CHECK(has_event(ExpectEvent::DoorOpen, *plan_with_door_open_close));
  CHECK(has_event(ExpectEvent::DoorClose, *plan_with_door_open_close));
}

SCENARIO("Test planner with various start conditions")
{
  using namespace std::chrono_literals;
  using rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using Planner = rmf_traffic::agv::Planner;
  using Duration = std::chrono::nanoseconds;
  using DetectConflict = rmf_traffic::DetectConflict;

  const std::string test_map_name = "test_map";
  Graph graph;
  graph.add_waypoint(test_map_name, {0, -5}); // 0
  graph.add_waypoint(test_map_name, {-5, 0}); // 1
  graph.add_waypoint(test_map_name, {0, 0}); // 2
  graph.add_waypoint(test_map_name, {5, 0}); // 3
  graph.add_waypoint(test_map_name, {0, 5}); // 4
  REQUIRE(graph.num_waypoints() == 5);

  graph.add_lane(0, 2); // 0
  graph.add_lane(2, 0); // 1
  // added within tests for testing with orientation constraints
  // graph.add_lane(1, 2);
  // graph.add_lane(2, 1);
  graph.add_lane(3, 2); // 2
  graph.add_lane(2, 3); // 3
  graph.add_lane(4, 2); // 4
  graph.add_lane(2, 4); // 5

  const auto profile = create_test_profile(UnitCircle);
  const VehicleTraits traits{
    {1.0, 0.4},
    {1.0, 0.5},
    profile};

  rmf_traffic::schedule::Database database;
  const rmf_traffic::schedule::ParticipantId p_obs =
    database.register_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "obstacles",
      "test_Planner",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      profile
    });
  rmf_traffic::schedule::ItineraryVersion iv_o = 0;
  rmf_traffic::RouteId ri_o = 0;

  const auto interrupt_flag = std::make_shared<bool>(false);
  Duration hold_time = std::chrono::seconds(6);
  const rmf_traffic::agv::Planner::Options default_options{
    make_test_schedule_validator(database, profile),
    hold_time,
    interrupt_flag};

  Planner planner{
    Planner::Configuration{graph, traits},
    default_options};

  const rmf_traffic::Time initial_time = std::chrono::steady_clock::now();

  WHEN("Start initial_location coincident with initial_waypoint")
  {
    graph.add_lane(1, 2); // 6
    graph.add_lane(2, 1); // 7
    planner = Planner{Planner::Configuration{graph, traits}, default_options};

    rmf_utils::optional<Eigen::Vector2d> initial_location =
      Eigen::Vector2d{-5.0, 0};

    Planner::Start start1 = Planner::Start{
      initial_time,
      1,
      0.0};
    CHECK_FALSE(start1.location());

    Planner::Start start2 = Planner::Start{
      initial_time,
      1,
      0.0,
      std::move(initial_location)};
    CHECK(start2.location());

    Planner::Goal goal{3};

    const auto plan1 = planner.plan(start1, goal);
    REQUIRE(plan1);
    CHECK_PLAN(plan1, {-5.0, 0}, 0.0, {5.0, 0}, {1, 3});
    const auto duration1 =
      plan1->get_itinerary().front().trajectory().duration();
    const auto plan2 = plan1.replan(start2);
    REQUIRE(plan2);
    CHECK_PLAN(plan2, {-5.0, 0}, 0.0, {5.0, 0}, {1, 3});
    const auto duration2 =
      plan2->get_itinerary().front().trajectory().duration();
    CHECK((duration1 - duration2).count() == Approx(0.0));
    CHECK(plan1->get_itinerary().size() == plan2->get_itinerary().size());

    // Test with startset
    std::vector<Planner::Start> starts{start1, start2};
    const auto plan = plan1.replan(starts);
    REQUIRE(plan);
    CHECK_PLAN(plan, {-5.0, 0}, 0.0, {5.0, 0}, {1, 3});
    CHECK(
      (plan->get_itinerary().front().trajectory().duration() - duration1).count() == Approx(
        0.));
    CHECK(plan->get_itinerary().size() == plan1->get_itinerary().size());
  }

  WHEN("Start initial_location is not on an initial_waypoint")
  {
    graph.add_lane(1, 2); // 6
    graph.add_lane(2, 1); // 7
    planner = Planner{Planner::Configuration{graph, traits}, default_options};

    rmf_utils::optional<Eigen::Vector2d> initial_location =
      Eigen::Vector2d{-2.5, 0};

    Planner::Start start = Planner::Start{
      initial_time,
      1,
      0.0,
      std::move(initial_location)};

    CHECK(start.location());
    CHECK_FALSE(start.lane());

    Planner::Goal goal = Planner::Goal{3};

    const auto plan = planner.plan(start, goal);

    // Note: Waypoint 2 will be skipped because the robot does not need to
    // stop or turn there. It moves directly from waypoint 1 to waypoint 3.
    CHECK_PLAN(plan, {-2.5, 0}, 0.0, {5.0, 0}, {1, 3});

    WHEN("Testing replan")
    {
      auto plan2 = plan.replan(start);
      CHECK_PLAN(plan2, {-2.5, 0}, 0.0, {5.0, 0}, {1, 3});
    }

    WHEN("Obstace 4->0 overlaps")
    {
      std::vector<rmf_traffic::Trajectory> obstacles;
      rmf_traffic::Trajectory obstacle;
      obstacle.insert(
        initial_time,
        Eigen::Vector3d{0, 0, 0},
        Eigen::Vector3d{0, 0, 0});

      obstacle.insert(
        initial_time + 60s,
        Eigen::Vector3d{0, 0, 0},
        Eigen::Vector3d{0, 0, 0});

      CHECK(DetectConflict::between(
          profile, obstacle, profile,
          plan->get_itinerary().front().trajectory()));
      obstacles.push_back(obstacle);

      test_with_obstacle(
        "Obstace 4->0 overlaps",
        plan, database, obstacles, 1, initial_time, true);

      // Note: Waypoint 2 will be skipped because the robot does not need to
      // stop or turn there. It moves directly from waypoint 1 to waypoint 3.
      CHECK_PLAN(plan, {-2.5, 0}, 0.0, {5.0, 0}, {1, 3});
    }
  }

  WHEN("Start contains initial_location and initial_lane")
  {

    Planner::Goal goal{3};
    rmf_utils::optional<Eigen::Vector2d> initial_location =
      Eigen::Vector2d{-2.5, 0};

    WHEN("initial_lane is not constrained")
    {
      graph.add_lane(1, 2); // 6
      graph.add_lane(2, 1); // 7
      planner = Planner{Planner::Configuration{graph, traits}, default_options};

      rmf_utils::optional<std::size_t> initial_lane = std::size_t{7};

      Planner::Start start{
        initial_time,
        1,
        0.0,
        std::move(initial_location),
        std::move(initial_lane)};

      CHECK(start.location());
      CHECK(start.lane());

      const auto plan = planner.plan(start, goal);

      // Note: Waypoint 2 will be skipped because the robot does not need to
      // stop or turn there. It moves directly from waypoint 1 to waypoint 3.
      CHECK_PLAN(plan, {-2.5, 0}, 0.0, {5.0, 0}, {1, 3});
    }

  }

  WHEN("Start contains initial location and lane constraint")
  {
    graph.add_lane(1, {2, Graph::OrientationConstraint::make({M_PI})}); // 6
    graph.add_lane(2, 1); // 7
    planner = Planner{Planner::Configuration{graph, traits}, default_options};

    rmf_utils::optional<Eigen::Vector2d> initial_location =
      Eigen::Vector2d{-4.99, 0};
    rmf_utils::optional<std::size_t> initial_lane = 6;

    Planner::Start start = Planner::Start{
      initial_time,
      2,
      0.0,
      std::move(initial_location),
      std::move(initial_lane)};

    CHECK(start.location());
    CHECK(start.lane());

    double goal_orientation = M_PI_2;
    Planner::Goal goal{3, goal_orientation};

    const auto plan = planner.plan(start, goal);
    CHECK_PLAN(plan, {-4.99, 0}, 0.0, {5.0, 0}, {2, 3}, &goal_orientation);
    // Check if lane exit constraint was satisfied
    auto waypoints = plan->get_waypoints();
    REQUIRE(waypoints.size() != 0);
    auto second_wp = ++waypoints.begin();
    CHECK((second_wp->position()[2] - M_PI) == Approx(0.0));
  }

  WHEN("Planning with startset with varying orientations")
  {
    graph.add_lane(1, 2); // 6
    graph.add_lane(2, 1); // 7
    planner = Planner{Planner::Configuration{graph, traits}, default_options};

    std::vector<Planner::Start> starts;
    Planner::Start start1{initial_time, 1, 0.0};
    Planner::Start start2{initial_time, 1, M_PI_2};
    starts.push_back(start1);
    starts.push_back(start2);

    Planner::Goal goal{3, 0.0};

    auto plan = planner.plan(starts, goal);
    //we expect the starting condition with orientation = 0 to be shortest
    CHECK_PLAN(plan, {-5, 0}, 0.0, {5.0, 0}, {1, 3});

    WHEN("Testing replan with startsets")
    {
      auto plan2 = plan.replan(starts);
      CHECK_PLAN(plan, {-5, 0}, 0.0, {5.0, 0}, {1, 3});
    }
    WHEN("Obstace 4->0 overlaps")
    {
      std::vector<rmf_traffic::Trajectory> obstacles;
      rmf_traffic::Trajectory obstacle;
      obstacle.insert(
        initial_time,
        Eigen::Vector3d{0, 0, 0},
        Eigen::Vector3d{0, 0, 0});

      obstacle.insert(
        initial_time + 60s,
        Eigen::Vector3d{0, 0, 0},
        Eigen::Vector3d{0, 0, 0});
      auto t = plan->get_itinerary().front().trajectory();
      REQUIRE(DetectConflict::between(profile, obstacle, profile, t));
      obstacles.push_back(obstacle);

      for (auto& obstacle : obstacles)
      {
        const auto r = std::make_shared<rmf_traffic::Route>(
          "test_map", obstacle);
        database.extend(p_obs, {{ri_o++, r}}, iv_o++);
      }

      const auto result1 = plan.replan(start1);
      const auto duration1 =
        result1->get_itinerary().front().trajectory().duration();
      const auto result2 = plan.replan(start2);
      const auto duration2 =
        result2->get_itinerary().front().trajectory().duration();

      const auto best_start = duration1 < duration2 ?
        result1->get_start() : result2->get_start();

      plan = planner.plan(starts, goal);
      CHECK_PLAN(plan, {-5, 0}, best_start.orientation(), {5.0, 0}, {1, 3});
    }
  }

  WHEN("Startset with same initial_location but different initial waypoints")
  {
    graph.add_lane(1, 2); // 6
    graph.add_lane(2, 1); // 7
    planner = Planner{Planner::Configuration{graph, traits}, default_options};

    rmf_utils::optional<Eigen::Vector2d> location = Eigen::Vector2d{-2.5, 0};
    std::vector<Planner::Start> starts;
    Planner::Start start1{initial_time, 1, 0.0, location};
    Planner::Start start2{initial_time, 2, 0.0, location};
    starts.push_back(start1);
    starts.push_back(start2);

    Planner::Goal goal{4, M_PI_2};

    auto plan = planner.plan(starts, goal);
    //we expect the starting condition with initial_waypoint = 2 to be shortest
    CHECK_PLAN(plan, {-2.5, 0}, 0.0, {0, 5}, {2, 4});
    const auto duration = plan->get_itinerary().front().trajectory().duration();

    const auto result1 = plan.replan(start1);
    REQUIRE(result1);
    const auto duration1 =
      result1->get_itinerary().front().trajectory().duration();
    const auto result2 = plan.replan(start2);
    REQUIRE(result2);
    const auto duration2 =
      result2->get_itinerary().front().trajectory().duration();
    CHECK(duration2 < duration1);
    CHECK(duration < duration1);
    CHECK((duration - duration2).count() == Approx(0.0).margin(1e-9));
  }

  WHEN(
    "Startset with same initial_location and initial_waypoints but different initial_orientations")
  {
    // lane with exit orientation constraint
    graph.add_lane(1, {2, Graph::OrientationConstraint::make({0})}); // 6
    graph.add_lane(2, 1); // 7
    planner = Planner{Planner::Configuration{graph, traits}, default_options};

    rmf_utils::optional<Eigen::Vector2d> location = Eigen::Vector2d{-2.5, 0};
    rmf_utils::optional<std::size_t> initial_lane = 6;

    std::vector<Planner::Start> starts;
    Planner::Start start1{initial_time, 2, 0.0, location, initial_lane};
    Planner::Start start2{initial_time, 2, M_PI, location, initial_lane};
    starts.push_back(start1);
    starts.push_back(start2);

    Planner::Goal goal{4, M_PI_2};

    auto plan = planner.plan(starts, goal);
    CHECK_PLAN(plan, {-2.5, 0}, 0.0, {0, 5}, {2, 4});
    const auto duration = plan->get_itinerary().front().trajectory().duration();
    const auto result1 = plan.replan(start1);
    REQUIRE(result1);
    const auto duration1 =
      result1->get_itinerary().front().trajectory().duration();
    const auto result2 = plan.replan(start2);
    REQUIRE(result2);
    const auto duration2 =
      result2->get_itinerary().front().trajectory().duration();

//    print_trajectory_info(*result1, initial_time);
//    print_trajectory_info(*result2, initial_time);
    CHECK(duration1 < duration2);
    CHECK(duration < duration2);
    CHECK((duration - duration1).count() == Approx(0.0).margin(1e-9));
  }
}

SCENARIO("Test starts using graph with non-colinear waypoints")
{
  using namespace std::chrono_literals;
  using rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using Planner = rmf_traffic::agv::Planner;
  using Duration = std::chrono::nanoseconds;

  const std::string test_map_name = "test_map";
  Graph graph;
  graph.add_waypoint(test_map_name, {0, 0}); // 0
  graph.add_waypoint(test_map_name, {-4, 3}); // 1
  graph.add_waypoint(test_map_name, {4, 3}); // 2
  graph.add_waypoint(test_map_name, {-4, 15}); // 3
  graph.add_waypoint(test_map_name, {4, 15}); // 4
  REQUIRE(graph.num_waypoints() == 5);

  graph.add_lane(0, 1); // 0
  graph.add_lane(1, 0); // 1
  graph.add_lane(0, 2); // 2
  graph.add_lane(2, 0); // 3
  graph.add_lane(1, 3); // 4
  graph.add_lane(3, 1); // 5
  graph.add_lane(2, 4); // 6
  REQUIRE(graph.num_lanes() == 7);

  const VehicleTraits traits{
    {1.0, 0.4},
    {1.0, 0.5},
    create_test_profile(UnitCircle)};

  rmf_traffic::schedule::Database database;
  const auto interrupt_flag = std::make_shared<bool>(false);
  Duration hold_time = std::chrono::seconds(1);
  const rmf_traffic::agv::Planner::Options default_options{
    make_test_schedule_validator(database, traits.profile()),
    hold_time,
    interrupt_flag};

  Planner planner{
    Planner::Configuration{graph, traits},
    default_options};

  const rmf_traffic::Time initial_time = std::chrono::steady_clock::now();

  // GIVEN("Robot moves from waypoint 3->4")
  // {
  //   Planner::Start start{initial_time, 3, -M_PI_2};
  //   const double goal_orientation = 0.0;
  //   Planner::Goal goal{4, goal_orientation};
  //   const auto plan = planner.plan(start, goal);
  //   CHECK_PLAN(plan, {-4, 15}, -M_PI_2, {4, 15}, {3, 1, 0 , 2, 4});
  // }

  GIVEN("Multiple starts on and off waypoints")
  {
    const double goal_orientaion = M_PI_2;
    Planner::Goal goal{4, goal_orientaion};
    // starts where robot is at initial_waypoint 1 but with different orientations
    Planner::Start start1{initial_time, 1, M_PI_2};
    Planner::Start start2{initial_time, 1, M_PI};

    // starts where robot has initial_locations displace from waypoint 1
    // Displaced 0.5m along lane 5
    rmf_utils::optional<Eigen::Vector2d> start_location1 =
      Eigen::Vector2d{-4, 3.5};

    Planner::Start start3{initial_time, 1, -M_PI_2, start_location1};
    // Displaced 0.5m along lane 1
    rmf_utils::optional<Eigen::Vector2d> start_location2 =
      Eigen::Vector2d{-3.6, 2.7};
    Planner::Start start4{initial_time, 1, -0.64, start_location1};

    std::vector<Planner::Start> starts{start1, start2, start3, start4};
    REQUIRE(starts.size() == 4);

    auto result1 = planner.plan(start1, goal);
    REQUIRE(result1);
    const auto duration1 =
      result1->get_itinerary().front().trajectory().duration();
    auto result2 = result1.replan(start2);
    REQUIRE(result2);
    const auto duration2 =
      result2->get_itinerary().front().trajectory().duration();
    auto result3 = result2.replan(start3);
    REQUIRE(result3);
    const auto duration3 =
      result3->get_itinerary().front().trajectory().duration();
    auto result4 = result3.replan(start4);
    REQUIRE(result4);
    const auto duration4 =
      result3->get_itinerary().front().trajectory().duration();

    std::vector<rmf_traffic::Duration> durations{
      duration1, duration2, duration3, duration4};

    auto best_duration_it = durations.begin();

    for (auto it = durations.begin(); it != durations.end(); it++)
    {
      if (*it < *best_duration_it)
      {
        best_duration_it = it;
      }
    }

    auto best_index = std::distance(durations.begin(), best_duration_it);
    Planner::Start best_start = starts[best_index];

    const auto plan = planner.plan(starts, goal);
    const auto plan_duration =
      plan->get_itinerary().front().trajectory().duration();
    const auto start_position =
      best_start.location() ? best_start.location().value() :
      graph.get_waypoint(best_start.waypoint()).get_location();
    CHECK_PLAN(
      plan,
      start_position,
      best_start.orientation(),
      {4, 15},
      {1, 0, 2, 4},
      &goal_orientaion);
    for (const auto duration : durations)
    {
      CHECK(plan_duration <= duration);
    }
    // start2 has the shortest duration
  }
}

SCENARIO("Multilevel Planning")
{
  using namespace std::chrono_literals;
  using rmf_traffic::agv::Graph;
  using VehicleTraits = rmf_traffic::agv::VehicleTraits;
  using Planner = rmf_traffic::agv::Planner;
  using Duration = std::chrono::nanoseconds;

  const VehicleTraits traits{
    {1.0, 0.4},
    {1.0, 0.5},
    create_test_profile(UnitCircle)};

  rmf_traffic::schedule::Database database;
  const auto interrupt_flag = std::make_shared<bool>(false);
  Duration hold_time = std::chrono::seconds(1);
  const rmf_traffic::agv::Planner::Options default_options{
    make_test_schedule_validator(database, traits.profile()),
    hold_time,
    interrupt_flag};

  GIVEN("Goal waypoint is the first waypoint on the second map")
  {
    Graph graph;
    graph.add_waypoint("L1", {-5, 0}); // 0
    graph.add_waypoint("L1", {0, 0}); // 1
    graph.add_waypoint("L2", {0, -5}); // 2
    REQUIRE(graph.num_waypoints() == 3);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(1, 2); // 2
    graph.add_lane(2, 1); // 3

    REQUIRE(graph.num_lanes() == 4);
    Planner planner{
      Planner::Configuration{graph, traits},
      default_options};

    // Plan from 0 -> 1
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const auto start = rmf_traffic::agv::Planner::Start(time, 0, 0.0);
    const auto goal = rmf_traffic::agv::Planner::Goal(2);
    const auto plan = planner.plan(start, goal);
    REQUIRE(plan.success());
    CHECK_PLAN(plan, {-5, 0}, 0.0, {0, -5}, {0, 1, 2});
  }

  GIVEN("Goal waypoint is the second waypoint on the second map")
  {
    Graph graph;
    graph.add_waypoint("L1", {-5, 0}); // 0
    graph.add_waypoint("L1", {0, 0}); // 1
    graph.add_waypoint("L2", {0, -5}); // 2
    graph.add_waypoint("L2", {5, -5}); // 3
    REQUIRE(graph.num_waypoints() == 4);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(1, 2); // 2
    graph.add_lane(2, 1); // 3
    graph.add_lane(2, 3); // 4
    graph.add_lane(3, 2); // 5

    REQUIRE(graph.num_lanes() == 6);
    Planner planner{
      Planner::Configuration{graph, traits},
      default_options};

    // Plan from 0 -> 3
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const auto start = rmf_traffic::agv::Planner::Start(time, 0, 0.0);
    const auto goal = rmf_traffic::agv::Planner::Goal(3);
    const auto plan = planner.plan(start, goal);
    REQUIRE(plan.success());
    CHECK_PLAN(plan, {-5, 0}, 0.0, {5, -5}, {0, 1, 2, 3});
  }

  GIVEN("Goal waypoint is the second waypoint on the third map")
  {
    // L1 -> L2 -> L3
    Graph graph;
    graph.add_waypoint("L1", {-5, 0}); // 0
    graph.add_waypoint("L1", {0, 0}); // 1
    graph.add_waypoint("L2", {0, -5}); // 2
    graph.add_waypoint("L2", {5, -5}); // 3
    graph.add_waypoint("L3", {5, -10}); // 4
    graph.add_waypoint("L3", {10, -10}); // 5
    REQUIRE(graph.num_waypoints() == 6);

    graph.add_lane(0, 1); // 0
    graph.add_lane(1, 0); // 1
    graph.add_lane(1, 2); // 2
    graph.add_lane(2, 1); // 3
    graph.add_lane(2, 3); // 4
    graph.add_lane(3, 2); // 5
    graph.add_lane(3, 4); // 6
    graph.add_lane(4, 3); // 7
    graph.add_lane(4, 5); // 8
    graph.add_lane(5, 4); // 9

    REQUIRE(graph.num_lanes() == 10);
    Planner planner{
      Planner::Configuration{graph, traits},
      default_options};

    // Plan from 0 -> 5
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const auto start = rmf_traffic::agv::Planner::Start(time, 0, 0.0);
    const auto goal = rmf_traffic::agv::Planner::Goal(5);
    const auto plan = planner.plan(start, goal);
    REQUIRE(plan.success());
    CHECK_PLAN(plan, {-5, 0}, 0.0, {10, -10}, {0, 1, 2, 3, 4, 5});
  }

  GIVEN("Graph with Lift")
  {
    using Event = Graph::Lane::Event;
    using LiftDoorOpen = Graph::Lane::LiftDoorOpen;
    Graph graph;
    graph.add_waypoint("L1", {-5, 0}); // 0
    graph.add_waypoint("L1", {0, 0}); // 1
    graph.add_waypoint("L2", {0, -5}); // 2
    graph.add_waypoint("L2", {5, -5}); // 3
    REQUIRE(graph.num_waypoints() == 4);

    graph.add_lane(
      {0, Event::make(LiftDoorOpen("Lift1", "L1", 4s))}, 1);
    graph.add_lane(
      {1, Event::make(LiftDoorOpen("Lift1", "L1", 4s))}, 0);
    graph.add_lane(
      {1, Event::make(LiftDoorOpen("Lift1", "L2", 4s))}, 2);
    graph.add_lane(
      {2, Event::make(LiftDoorOpen("Lift1", "L2", 4s))}, 1);
    graph.add_lane(2, 3);
    graph.add_lane(3, 2);
    REQUIRE(graph.num_lanes() == 6);

    Planner planner{
      Planner::Configuration{graph, traits},
      default_options};

    // Plan from 0 -> 3
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const auto start = rmf_traffic::agv::Planner::Start(time, 0, 0.0);
    const auto goal = rmf_traffic::agv::Planner::Goal(3);
    const auto plan = planner.plan(start, goal);
    REQUIRE(plan.success());
    CHECK_PLAN(plan, {-5, 0}, 0.0, {5, -5}, {0, 1, 2, 3});
    CHECK(count_events(*plan) == 2);
    CHECK(has_event(ExpectEvent::LiftDoorOpen, *plan));
  }
}

SCENARIO("Close start", "[close_start]")
{
  using namespace std::chrono_literals;

  auto database = std::make_shared<rmf_traffic::schedule::Database>();

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(0.1),
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  auto p1 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 1",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  auto p2 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 2",
      "test_Negotiator",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, { 0.0, -5.0}); // 0
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 1
  graph.add_waypoint(test_map_name, { 0.0, 0.0}); // 2
  graph.add_waypoint(test_map_name, { 5.0,  0.0}); // 3
  graph.add_waypoint(test_map_name, { 0.0, 5.0}); // 4
  graph.add_waypoint(test_map_name, { 5.0, 5.0}); // 5

  /*
   *         4-----5
   *         |     |
   *         |     |
   *   1-----2-----3
   *         |
   *         |
   *         0
   */

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 2);
  add_bidir_lane(1, 2);
  add_bidir_lane(3, 2);
  add_bidir_lane(3, 5);
  add_bidir_lane(4, 2);
  add_bidir_lane(4, 5);

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  const auto time = std::chrono::steady_clock::now();

  rmf_traffic::agv::Planner::Configuration configuration{graph, traits};

  rmf_traffic::Trajectory t_obs;
  t_obs.insert(time, {-0.5, 0.0, 0.0}, {0.0, 0.0, 0.0});
  t_obs.insert(time + 10min, {-0.5, 0.0, 0.0}, {0.0, 0.0, 0.0});
  p2.set({{test_map_name, t_obs}});

  rmf_traffic::agv::Planner planner{
    configuration,
    rmf_traffic::agv::Plan::Options{
      rmf_traffic::agv::ScheduleRouteValidator::make(
            database, p1.id(), p1.description().profile())
    }
  };

  const auto result =
      planner.plan(
        {
          rmf_traffic::agv::Plan::Start(time, 2, 0, Eigen::Vector2d{0.5, 0.0}),
          rmf_traffic::agv::Plan::Start(time, 3, 0, Eigen::Vector2d{0.5, 0.0})
        }, 4);

  REQUIRE(result);

  CHECK(result->get_itinerary().back().trajectory().back().time()
        < time + 10min);

  std::unordered_set<std::size_t> visited_wps;
  for (const auto& wp : result->get_waypoints())
  {
    if (wp.graph_index())
      visited_wps.insert(*wp.graph_index());
  }

  CHECK(visited_wps.size() == 3);
  CHECK(visited_wps.count(3));
  CHECK(visited_wps.count(5));
  CHECK(visited_wps.count(4));
}
