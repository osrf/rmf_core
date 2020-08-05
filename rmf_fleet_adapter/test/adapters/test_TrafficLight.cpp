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

#include "../mock/MockTrafficLightCommand.hpp"

#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>

#include <rmf_traffic_ros2/schedule/Node.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rclcpp/context.hpp>
#include <rclcpp/executors.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <rmf_utils/catch.hpp>

#include "../thread_cooldown.hpp"

static std::size_t node_count = 0;

//==============================================================================
std::vector<rmf_fleet_adapter::agv::Waypoint> make_path(
    const rmf_traffic::agv::Graph& graph,
    const std::vector<std::size_t>& wp_indices,
    const double orientation)
{
  std::vector<rmf_fleet_adapter::agv::Waypoint> path;
  for (auto index : wp_indices)
  {
    const auto& wp = graph.get_waypoint(index);
    Eigen::Vector3d p;
    p[2] = orientation;
    p.block<2,1>(0,0) = wp.get_location();
    path.emplace_back(wp.get_map_name(), p);
  }

  return path;
}

//==============================================================================
rmf_traffic::agv::Graph make_test_graph()
{
  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                   10
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

  add_bidir_lane(0, 1);  // 0   1
  add_bidir_lane(1, 2);  // 2   3
  add_bidir_lane(1, 5);  // 4   5
  add_bidir_lane(2, 6);  // 6   7
  add_bidir_lane(3, 4);  // 8   9
  add_bidir_lane(4, 5);  // 10 11
  add_bidir_lane(5, 6);  // 12 13
  add_bidir_lane(6, 7);  // 14 15
  add_bidir_lane(5, 8);  // 16 17
  add_bidir_lane(6, 9);  // 18 19
  add_bidir_lane(8, 9);  // 20 21
  add_bidir_lane(8, 10); // 22 23

  return graph;
}

//==============================================================================
rclcpp::NodeOptions make_test_node_options()
{
  rclcpp::NodeOptions node_options;

  const rclcpp::Parameter use_sim_time("use_sim_time", true);
  node_options.parameter_overrides().push_back(use_sim_time);

  auto rcl_context = std::make_shared<rclcpp::Context>();
  rcl_context->init(0, nullptr);
  node_options.context(rcl_context);

  return node_options;
}

//==============================================================================
SCENARIO("Test new path timing")
{
  rmf_fleet_adapter_test::thread_cooldown = true;
  using namespace std::chrono_literals;

  const auto graph = make_test_graph();

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  auto rcl_context = std::make_shared<rclcpp::Context>();
  rcl_context->init(0, nullptr);
  rmf_fleet_adapter::agv::test::MockAdapter adapter(
        "test_TrafficLight_" + std::to_string(++node_count),
        rclcpp::NodeOptions().context(rcl_context));

  auto command_0 =
      std::make_shared<rmf_fleet_adapter_test::MockTrafficLightCommand>();
  auto update_0 = adapter.add_traffic_light(
        command_0, "fleet_0", "robot_0", traits, profile);

  auto command_1 =
      std::make_shared<rmf_fleet_adapter_test::MockTrafficLightCommand>();
  auto update_1 = adapter.add_traffic_light(
        command_1, "fleet_1", "robot_1", traits, profile);

  adapter.start();

  WHEN("Crossing paths")
  {
    const auto now = adapter.node()->now();

    const auto path_0 = make_path(graph, {0, 1, 5, 8, 10}, M_PI/2.0);
    update_0->update_path(path_0);
    std::unique_lock<std::mutex> lock_0(command_0->mutex);
    command_0->cv.wait_for(
          lock_0, 100ms,
          [command_0](){ return command_0->current_version.has_value(); });
    REQUIRE(command_0->current_version.has_value());
    CHECK(path_0.size() == command_0->current_timing.size());

    const auto path_1 = make_path(graph, {3, 4, 5, 6, 7}, 0.0);
    update_1->update_path(path_1);
    std::unique_lock<std::mutex> lock_1(command_1->mutex);
    command_1->cv.wait_for(
          lock_1, 100ms,
          [command_1](){ return command_1->current_version.has_value(); });
    REQUIRE(command_1->current_version.has_value());
    CHECK(path_1.size() == command_1->current_timing.size());

    REQUIRE(command_0->current_timing.size()
            == command_1->current_timing.size());

    // They would normally collide at index==2, so from index==2 onwards,
    // the path of command_1 must be lagging behind the path of command_0 by
    // at least the planner's default minimum holding time, or else the
    // planner might not have actually avoided the collision as intended.
    for (std::size_t i=2; i < command_0->current_timing.size(); ++i)
    {
      CHECK(command_1->current_timing[i] - command_0->current_timing[i]
            >= rmf_traffic::agv::Planner::Options::DefaultMinHoldingTime);
    }
  }

  WHEN("Sharing a lane")
  {
    const auto now = adapter.node()->now();

    const auto path_0 = make_path(graph, {2, 6, 5, 4, 3}, M_PI/2.0);
    update_0->update_path(path_0);
    std::unique_lock<std::mutex> lock_0(command_0->mutex);
    command_0->cv.wait_for(
          lock_0, 100ms,
          [command_0](){ return command_0->current_version.has_value(); });
    REQUIRE(command_0->current_version.has_value());
    CHECK(path_0.size() == command_0->current_timing.size());

    const auto path_1 = make_path(graph, {7, 6, 5, 8, 10}, 0.0);
    update_1->update_path(path_1);
    std::unique_lock<std::mutex> lock_1(command_1->mutex);
    command_1->cv.wait_for(
          lock_1, 100ms,
          [command_1](){ return command_1->current_version.has_value(); });
    REQUIRE(command_1->current_version.has_value());
    CHECK(path_1.size() == command_1->current_timing.size());

    for (std::size_t index : {0, 1, 2})
    {
      CHECK(command_1->current_timing[index] - command_0->current_timing[index]
            >= rmf_traffic::agv::Planner::Options::DefaultMinHoldingTime);
    }
  }
}

//==============================================================================
SCENARIO("Test negotiated timing")
{
  rmf_fleet_adapter_test::thread_cooldown = true;
  using namespace std::chrono_literals;

  bool finished = false;
  std::thread schedule_node_thread(
        [&finished]()
  {
    const auto schedule_node =
        rmf_traffic_ros2::schedule::make_node(make_test_node_options());

    rclcpp::executor::ExecutorArgs args;
    args.context = schedule_node->get_node_options().context();
    rclcpp::executors::SingleThreadedExecutor executor(args);
    executor.add_node(schedule_node);

    while (!finished && rclcpp::ok(args.context))
      executor.spin_some();
  });

  const auto adapter = rmf_fleet_adapter::agv::Adapter::make(
        "test_TrafficLight_" + std::to_string(++node_count),
        make_test_node_options());
  REQUIRE(adapter);

  const auto clock =
      adapter->node()->create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::SystemDefaultsQoS());

  adapter->start();

  const auto graph = make_test_graph();

  const rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  const auto command_0 =
      std::make_shared<rmf_fleet_adapter_test::MockTrafficLightCommand>();
  std::promise<rmf_fleet_adapter::agv::TrafficLight::UpdateHandlePtr> promise_0;
  auto future_0 = promise_0.get_future();
  adapter->add_traffic_light(
        command_0, "fleet_0", "robot_0", traits, profile,
        [&promise_0](auto update_handle)
  {
    promise_0.set_value(update_handle);
  });

  const auto command_1 =
      std::make_shared<rmf_fleet_adapter_test::MockTrafficLightCommand>();
  std::promise<rmf_fleet_adapter::agv::TrafficLight::UpdateHandlePtr> promise_1;
  auto future_1 = promise_1.get_future();
  adapter->add_traffic_light(
        command_1, "fleet_1", "robot_1", traits, profile,
        [&promise_1](auto update_handle)
  {
    promise_1.set_value(update_handle);
  });

  REQUIRE(future_0.wait_for(std::chrono::seconds(5))
          == std::future_status::ready);
  const auto update_0 = future_0.get();

  REQUIRE(future_1.wait_for(std::chrono::seconds(5))
          == std::future_status::ready);
  const auto update_1 = future_1.get();

  WHEN("Sharing a lane")
  {
    const auto now = adapter->node()->now();

    const auto path_0 = make_path(graph, {2, 6, 5, 4, 3}, M_PI/2.0);
    auto old_count_0 = command_0->command_counter;
    update_0->update_path(path_0);
    {
      std::unique_lock<std::mutex> lock(command_0->mutex);
      command_0->cv.wait_for(
            lock, 100ms,
            [command_0, old_count_0]()
      {
        return old_count_0 < command_0->command_counter;
      });
    }
    REQUIRE(command_0->current_version.has_value());
    CHECK(path_0.size() == command_0->current_timing.size());

    // TODO(MXG): This wait time is somewhat arbitrary and may result in race
    // conditions for this test. The hope is that this provides enough time for
    // the itinerary robot_0 to make it to the schedule and then get updated in
    // the mirror. Maybe we should provide debug hooks for the mirror so that we
    // only wait exactly as long as necessary.
    std::this_thread::sleep_for(100ms);

    const auto path_1 = make_path(graph, {7, 6, 5, 8, 10}, 0.0);
    auto old_count_1 = command_1->command_counter;
    update_1->update_path(path_1);
    {
      std::unique_lock<std::mutex> lock(command_1->mutex);
      command_1->cv.wait_for(
            lock, 100ms,
            [command_1, old_count_1]()
      {
        return old_count_1 < command_1->command_counter;
      });
    }
    REQUIRE(command_1->current_version.has_value());
    CHECK(path_1.size() == command_1->current_timing.size());

    REQUIRE(command_0->current_timing.size()
            == command_1->current_timing.size());

    for (std::size_t i : {0, 1, 2})
    {
      CHECK(command_1->current_timing[i] - command_0->current_timing[i]
             >= rmf_traffic::agv::Planner::Options::DefaultMinHoldingTime);
    }

    rosgraph_msgs::msg::Clock time;
    time.clock.sec = (command_1->current_timing[0].seconds()
        + command_1->current_timing[1].seconds())/2.0;

    clock->publish(time);

    const auto begin_wait = std::chrono::steady_clock::now();
    // Wait for the ROS2 clock to be updated.
    while (adapter->node()->now() < time.clock)
    {
      std::this_thread::sleep_for(10ms);
      REQUIRE(std::chrono::steady_clock::now() - begin_wait < 2s);
    }

    REQUIRE(command_0->current_version);
    CHECK(command_0->current_version.value() == 1);
    CHECK(command_0->command_counter == 1);
    REQUIRE(command_0->current_progress_updater);

    REQUIRE(command_1->current_version);
    CHECK(command_1->current_version.value() == 1);
    CHECK(command_1->command_counter == 1);
    REQUIRE(command_1->current_progress_updater);

    Eigen::Vector3d p1;
    p1.block<2,1>(0,0) = Eigen::Vector2d(8.0, 0.0);
    p1[2] = 0.0;
    old_count_1 = command_1->command_counter;
    command_1->current_progress_updater(1, p1);

    std::this_thread::sleep_for(10ms);

    Eigen::Vector3d p0;
    p0.block<2,1>(0,0) = graph.get_waypoint(2).get_location();
    p0[2] = M_PI/2.0;
    old_count_0 = command_0->command_counter;
    command_0->current_progress_updater(1, p0);
    {
      std::unique_lock<std::mutex> lock(command_0->mutex);
      command_0->cv.wait_for(
            lock, 30s,
            [command_0, old_count_0]()
      {
        return old_count_0 < command_0->command_counter;
      });
      CHECK(command_0->command_counter == 2);
      CHECK(command_0->current_version.value() == 1);
    }

    {
      std::unique_lock<std::mutex> lock(command_1->mutex);
      if (command_1->current_version.value() < 2)
      command_1->cv.wait_for(
            lock, 30s,
            [command_1, old_count_1]()
      {
        return old_count_1 < command_1->command_counter;
      });
      CHECK(command_1->command_counter == 2);
      CHECK(command_1->current_version.value() == 1);
    }

    for (std::size_t i : {0, 1, 2})
    {
      // Race conditions in the current implementation of the negotiation system
      // may cause the winner to change between runs. There's only one choice
      // that is truly optimal, but the current implementation of the
      // negotiation system simply chooses whichever proposal happens to finish
      // first, which may change between runs.
      //
      // What matters most for this test is that when the vehicles are visiting
      // the shared waypoints, there is a sufficient separation in their timing.
      const bool sufficient_separation =
          (command_1->current_timing[i] - command_0->current_timing[i]
           >= rmf_traffic::agv::Planner::Options::DefaultMinHoldingTime)
       || (command_0->current_timing[i] - command_1->current_timing[i]
           >= rmf_traffic::agv::Planner::Options::DefaultMinHoldingTime);

      CHECK(sufficient_separation);
    }
  }

  adapter->stop().wait_for(std::chrono::seconds(2));
  finished = true;
  if (schedule_node_thread.joinable())
    schedule_node_thread.join();
}
