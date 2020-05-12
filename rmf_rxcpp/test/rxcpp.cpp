/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "rx-wrappers.hpp"

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/Rollout.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rxcpp/rx.hpp>

#include <future>
#include <iostream>
#include <queue>

//==============================================================================
inline rmf_traffic::Time print_start(const rmf_traffic::Route& route)
{
  std::cout << "(start) --> ";
  std::cout << "(" << 0.0 << "; "
            << route.trajectory().front().position().transpose()
            << ") --> ";

  return *route.trajectory().start_time();
}

//==============================================================================
inline void print_route(
    const rmf_traffic::Route& route,
    const rmf_traffic::Time start_time)
{
  for (auto it = ++route.trajectory().begin(); it != route.trajectory().end(); ++it)
  {
    const auto& wp = *it;
    if (wp.velocity().norm() > 1e-3)
      continue;

    const auto rel_time = wp.time() - start_time;
    std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
              << wp.position().transpose() << ") --> ";
  }
}

//==============================================================================
inline void print_itinerary(
    const rmf_traffic::schedule::Itinerary& itinerary)
{
  auto start_time = print_start(*itinerary.front());
  for (const auto& r : itinerary)
    print_route(*r, start_time);

  std::cout << "(end)\n" << std::endl;
}

//==============================================================================
inline void print_itinerary(const std::vector<rmf_traffic::Route>& itinerary)
{
  auto start_time = print_start(itinerary.front());
  for (const auto& r : itinerary)
    print_route(r, start_time);

  std::cout << "(end)\n" << std::endl;
}

//==============================================================================
rmf_traffic::agv::Graph make_graph()
{
  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0}, true);  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}, true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}, true); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}, true); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}, true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                  10
   *                   |
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(1, 5);
  add_bidir_lane(2, 6);
  add_bidir_lane(3, 4);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(6, 7);
  add_bidir_lane(5, 8);
  add_bidir_lane(6, 9);
  add_bidir_lane(8, 9);
  add_bidir_lane(8, 10);

  return graph;
}

//==============================================================================
struct PlannerJob
{
  using Result = rmf_traffic::agv::Planner::Result;

  PlannerJob(
      rmf_traffic::schedule::Database schedule,
      rmf_traffic::agv::Planner::Configuration config,
      rmf_traffic::agv::Planner::Start start,
      rmf_traffic::agv::Planner::Goal goal)
    : _database(std::move(schedule)),
      _current_result(
        initiate(
          _database,
          std::move(config),
          std::move(start),
          std::move(goal)))
  {
    // Do nothing
  }

  const rmf_traffic::agv::Planner::Result& process()
  {
    _current_result.resume();
    if (_current_result)
      _completed = true;
    return _current_result;
  }

  double cost_estimate() const
  {
    return _current_result.cost_estimate();
  }

  struct Compare
  {
    bool operator()(
        const std::shared_ptr<PlannerJob>& a,
        const std::shared_ptr<PlannerJob>& b)
    {
      return b->cost_estimate() < a->cost_estimate();
    }
  };

  inline bool completed() const
  {
    return _completed;
  }

private:
  rmf_traffic::schedule::Database _database;
  rmf_traffic::agv::Planner::Result _current_result;
  bool _completed = false;

  static rmf_traffic::agv::Planner::Result initiate(
      const rmf_traffic::schedule::Viewer& viewer,
      rmf_traffic::agv::Planner::Configuration config,
      rmf_traffic::agv::Planner::Start start,
      rmf_traffic::agv::Planner::Goal goal)
  {
    // Using this do_not_start flag with a value of true will make sure that the
    // planning problem gets started up but is immediately interrupted so that
    // it can be continued later.
    //
    // TODO(MXG): Should we create a Planner::setup() function that does this as
    // an alternative to Planner::plan()?
    bool do_not_start = true;
    rmf_traffic::agv::Planner::Options options(
          rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
            viewer, 10000, config.vehicle_traits().profile()),
          std::chrono::seconds(5),
          &do_not_start);

    return rmf_traffic::agv::Planner(std::move(config), std::move(options))
        .plan(std::move(start), std::move(goal));
  }
};

//==============================================================================
int main()
{
  //============================================================================
  // vvvvvvvvvvvvvvvvvvvvvvvv Setting up the problem vvvvvvvvvvvvvvvvvvvvvvvvvvv
  const auto graph = make_graph();

  const rmf_traffic::Profile profile(
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(1.0));

  const rmf_traffic::schedule::ParticipantDescription description{
    "participant_0",
    "rxcpp",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    profile
  };

  const rmf_traffic::agv::VehicleTraits traits(
      {0.7, 0.3}, {1.0, 0.45}, profile);

  rmf_traffic::schedule::Database master_schedule;
  auto p_A = rmf_traffic::schedule::make_participant(
        description, master_schedule);

  auto p_B = rmf_traffic::schedule::make_participant(
        description, master_schedule);

  rmf_traffic::agv::Planner::Configuration config(graph, traits);
  auto start_time = std::chrono::steady_clock::now();

  const auto start_B = rmf_traffic::agv::Plan::Start(start_time, 5, 0.0);
  const auto goal_B = rmf_traffic::agv::Plan::Goal(3);
  rmf_traffic::agv::Planner planner_B(
        config,
        {
          rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
            master_schedule, p_B.id(), profile)
        });

  const auto initial_plan = planner_B.plan(start_B, goal_B);
  assert(initial_plan);

  p_B.set(initial_plan->get_itinerary());

  rmf_traffic::agv::Planner planner_A(
        config,
        {
          rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
            master_schedule, p_A.id(), profile)
        });

  // This will fail because the other participant is blocking it.
  const auto start_A = rmf_traffic::agv::Plan::Start(start_time, 3, 0.0);
  const auto goal_A = rmf_traffic::agv::Plan::Goal(7);
  auto result_A = planner_A.plan(start_A, goal_A);
  assert(!result_A);

  rmf_traffic::agv::Rollout rollout_A(result_A);
  auto alternatives = rollout_A.expand(
        p_B.id(), std::chrono::seconds(30), {nullptr});

  using JobQueue =
    std::priority_queue<
      PlannerJob,
      std::vector<std::shared_ptr<PlannerJob>>,
      PlannerJob::Compare
    >;

  JobQueue planning_job_queue;
  std::vector<std::shared_ptr<PlannerJob>> planner_jobs;

  std::cout << "Alternatives:" << std::endl;
  for (const auto& alternative : alternatives)
  {
    print_itinerary(alternative);
    rmf_traffic::schedule::Database database;
    const auto p = database.register_participant(description);
    rmf_traffic::schedule::ItineraryVersion v = 0;
    rmf_traffic::schedule::Version r = 0;
    for (const auto& route : alternative)
      database.extend(p, {{r, route}}, v++);

//    planning_job_queue.push(std::make_shared<PlannerJob>(
//                     std::move(database), config, start_B, goal_B));
    planner_jobs.emplace_back(std::make_shared<PlannerJob>(
                         std::move(database), config, start_B, goal_B));
  }

  //^^^^^^^^^^^^^^^^^^^^ Done setting up the problem ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //============================================================================


  //============================================================================
  //vvvvvvvvvvvvvvvvvvvvv TODO: Make this parallel vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  std::cout << "Jobs: " << planning_job_queue.size() << std::endl;

  struct JobResult
  {
    double cost;
    rmf_traffic::agv::Plan plan;
  };

  // Task 1: Find the best_result for the planning_job_queue
  rmf_utils::optional<JobResult> best_result;

  std::size_t finished_count = 0;

  run_jobs(planner_jobs, [&finished_count, &start_time, &best_result](const auto& progress)
  {
    std::cout << "thread: " << std::this_thread::get_id() << std::endl;
    const auto& result = progress.result;

    if (!result.cost_estimate())
    {
      // The plan is impossible, so we should just move on
      std::cout << "(Fail) Finished: " << ++finished_count << std::endl;
      progress.cancel();
      return;
    }

    if (result) // This evaluates to true if a plan is ready
    {
      std::cout << "(Success) Finished: " << ++finished_count << std::endl;
      const auto finish_time =
          *result->get_itinerary().back().trajectory().finish_time();

      const double cost =
          rmf_traffic::time::to_seconds(finish_time - start_time);

      if (!best_result || cost < best_result->cost)
      {
        std::cout << "New best" << std::endl;
        // If no result exists yet, use this as the best result.
        best_result = JobResult{cost, *result};
      }
    }
    else if (!best_result || *result.cost_estimate() < best_result->cost)
    {
      // This job could still produce a better plan than the current best, so
      // put it back in the job queue.
      std::cout << "(Resume) " << "current cost: " << *result.cost_estimate() << " best: ";
      if (best_result)
        std::cout << best_result->cost;
      else
        std::cout << "N/A";
      std::cout << std::endl;
//      planning_job_queue.push(progress.job);
    }
    else
    {
      std::cout << "(Discarded) Finished: " << ++finished_count << std::endl;
      progress.cancel();
    }
    std::cout << "//////////" << std::endl;
  });

//  while (!planning_job_queue.empty())
//  {
//    const auto top = planning_job_queue.top();
//    planning_job_queue.pop();
//
//    bool interrupt_flag = false;
//    auto future =
//        std::async(
//          std::launch::async,
//          [&]() -> const rmf_traffic::agv::Planner::Result& {
//            return top->process(&interrupt_flag);
//          });
//
//    future.wait_for(std::chrono::milliseconds(10));
//    interrupt_flag = true;
//    const auto& result = future.get();
//
//    if (!result.cost_estimate())
//    {
//      // The plan is impossible, so we should just move on
//      std::cout << "(Fail) Finished: " << ++finished_count << std::endl;
//      continue;
//    }
//
//    if (result) // This evaluates to true if a plan is ready
//    {
//      std::cout << "(Success) Finished: " << ++finished_count << std::endl;
//      const auto finish_time =
//          *result->get_itinerary().back().trajectory().finish_time();
//
//      const double cost =
//          rmf_traffic::time::to_seconds(finish_time - start_time);
//
//      if (!best_result || cost < best_result->cost)
//      {
//        std::cout << "New best" << std::endl;
//        // If no result exists yet, use this as the best result.
//        best_result = JobResult{cost, *result};
//      }
//    }
//    else if (!best_result || *result.cost_estimate() < best_result->cost)
//    {
//      // This job could still produce a better plan than the current best, so
//      // put it back in the job queue.
//      planning_job_queue.push(top);
//    }
//    else
//    {
//      std::cout << "(Discarded) Finished: " << ++finished_count << std::endl;
//    }
//  }
//
  std::cout <<"\nBest plan for B:" << std::endl;
  print_itinerary(best_result->plan.get_itinerary());

  // Task 2: Given the best_result from the planning_job_queue, add it to the
  // schedule and find an itinerary for the other participant
  p_B.set(best_result->plan.get_itinerary());
  auto final_result_A = planner_A.plan(start_A, goal_A);
  assert(final_result_A);
  std::cout << "\nBest plan for A:" << std::endl;
  print_itinerary(final_result_A->get_itinerary());
  p_A.set(final_result_A->get_itinerary());
}
