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

#include "../mock/MockRobotCommand.hpp"

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_fleet_adapter/agv/test/MockAdapter.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>

#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <rmf_utils/catch.hpp>

#include "../thread_cooldown.hpp"

#include <rmf_task_msgs/msg/task_summary.hpp>


//==============================================================================
SCENARIO("Test loop requests")
{
  rmf_fleet_adapter_test::thread_cooldown = true;
  using namespace std::chrono_literals;

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

  // TODO(MXG): Add tests with mock doors and mock lifts

  /*
   *                   10[north]
   *                   |
   *                  (B)
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6--(A)--7[east]
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0[south]
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  auto add_dock_lane = [&](
    const std::size_t w0,
    const std::size_t w1,
    std::string dock_name)
    {
      using Lane = rmf_traffic::agv::Graph::Lane;
      graph.add_lane({w0, Lane::Event::make(Lane::Dock(dock_name, 10s))}, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);  // 0   1
  add_bidir_lane(1, 2);  // 2   3
  add_bidir_lane(1, 5);  // 4   5
  add_bidir_lane(2, 6);  // 6   7
  add_bidir_lane(3, 4);  // 8   9
  add_bidir_lane(4, 5);  // 10 11
  add_bidir_lane(5, 6);  // 12 13
  add_dock_lane(6, 7, "A");  // 14 15
  add_bidir_lane(5, 8);  // 16 17
  add_bidir_lane(6, 9);  // 18 19
  add_bidir_lane(8, 9);  // 20 21
  add_dock_lane(8, 10, "B"); // 22 23

  const std::string north = "north";
  REQUIRE(graph.add_key(north, 10));

  const std::string east = "east";
  REQUIRE(graph.add_key(east, 7));

  const std::string south = "south";
  REQUIRE(graph.add_key(south, 0));


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
    "test_Loop", rclcpp::NodeOptions().context(rcl_context));

  const std::string loop_0 = "loop_0";
  std::promise<bool> task_0_completed_promise;
  auto task_0_completed_future = task_0_completed_promise.get_future();
  std::size_t completed_0_count = 0;
  bool at_least_one_incomplete_task_0 = false;
  rmf_task_msgs::msg::TaskSummary last_task_0_msg;
  std::size_t finding_a_plan_0_count = 0;
  std::vector<std::string> finding_a_plan_0_statuses;

  const std::string loop_1 = "loop_1";
  std::promise<bool> task_1_completed_promise;
  auto task_1_completed_future = task_1_completed_promise.get_future();
  std::size_t completed_1_count = 0;
  bool at_least_one_incomplete_task_1 = false;
  rmf_task_msgs::msg::TaskSummary last_task_1_msg;
  std::size_t finding_a_plan_1_count = 0;
  std::vector<std::string> finding_a_plan_1_statuses;

  const auto task_sub = adapter.node()->create_subscription<
    rmf_task_msgs::msg::TaskSummary>(
    rmf_fleet_adapter::TaskSummaryTopicName, rclcpp::SystemDefaultsQoS(),
    [&task_0_completed_promise, &loop_0, &at_least_one_incomplete_task_0,
    &completed_0_count, &last_task_0_msg, &finding_a_plan_0_count,
    &task_1_completed_promise, &loop_1, &at_least_one_incomplete_task_1,
    &completed_1_count, &last_task_1_msg, &finding_a_plan_1_count,
    &finding_a_plan_0_statuses, &finding_a_plan_1_statuses](
      const rmf_task_msgs::msg::TaskSummary::SharedPtr msg)
    {
      if (msg->STATE_COMPLETED == msg->state)
      {
        if (msg->task_id == loop_0)
        {
          if (completed_0_count == 0)
            task_0_completed_promise.set_value(true);

          ++completed_0_count;
        }
        else if (msg->task_id == loop_1)
        {
          if (completed_1_count == 0)
            task_1_completed_promise.set_value(true);

          ++completed_1_count;
        }
        else
          CHECK(false);
      }
      else
      {
        if (msg->task_id == loop_0)
          at_least_one_incomplete_task_0 = true;
        else if (msg->task_id == loop_1)
          at_least_one_incomplete_task_1 = true;
        else
          CHECK(false);
      }

      if (msg->task_id == loop_0)
      {
        last_task_0_msg = *msg;
        if (msg->status.find("Finding a plan for") != std::string::npos)
        {
          ++finding_a_plan_0_count;
          finding_a_plan_0_statuses.push_back(msg->status);
        }
      }
      else if (msg->task_id == loop_1)
      {
        last_task_1_msg = *msg;
        if (msg->status.find("Finding a plan for") != std::string::npos)
        {
          ++finding_a_plan_1_count;
          finding_a_plan_1_statuses.push_back(msg->status);
        }
      }
    });

  const std::size_t n_loops = 5;

  const std::string fleet_type = "test_fleet";
  const auto fleet = adapter.add_fleet(fleet_type, traits, graph);

  // Configure default battery param
  using BatterySystem = rmf_battery::agv::BatterySystem;
  using PowerSystem = rmf_battery::agv::PowerSystem;
  using MechanicalSystem = rmf_battery::agv::MechanicalSystem;
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;

  auto battery_system = std::make_shared<BatterySystem>(
    *BatterySystem::make(24.0, 40.0, 8.8));

  auto mechanical_system = MechanicalSystem::make(70.0, 40.0, 0.22);
  auto motion_sink = std::make_shared<SimpleMotionPowerSink>(
    *battery_system, *mechanical_system);

  auto ambient_power_system = PowerSystem::make(20.0);
  auto ambient_sink = std::make_shared<SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  auto tool_power_system = PowerSystem::make(10.0);
  auto tool_sink = std::make_shared<SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  fleet->account_for_battery_drain(false);
  fleet->set_recharge_threshold(0.2);
  fleet->set_task_planner_params(
    battery_system, motion_sink, ambient_sink, tool_sink);

  fleet->accept_task_requests(
    [](const rmf_task_msgs::msg::TaskProfile& task)
    {
      // Accept all loop task requests
      CHECK(task.description.task_type.type ==
      rmf_task_msgs::msg::TaskType::TYPE_LOOP);
      return true;
    });

  // Add Robot T0
  const auto now = rmf_traffic_ros2::convert(adapter.node()->now());
  const rmf_traffic::agv::Plan::StartSet starts_0 = {{now, 0, 0.0}};
  auto robot_cmd_0 = std::make_shared<
    rmf_fleet_adapter_test::MockRobotCommand>(adapter.node(), graph);
  fleet->add_robot(
    robot_cmd_0, "T0", profile, starts_0,
    [&robot_cmd_0](rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
    {
      // assume battery soc is full
      updater->update_battery_soc(1.0);
      robot_cmd_0->updater = std::move(updater);
    });

  // Add Robot T1
  const rmf_traffic::agv::Plan::StartSet starts_1 = {{now, 7, 0.0}};
  auto robot_cmd_1 = std::make_shared<
    rmf_fleet_adapter_test::MockRobotCommand>(adapter.node(), graph);
  fleet->add_robot(
    robot_cmd_1, "T1", profile, starts_1,
    [&robot_cmd_1](rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
    {
      // assume battery soc is full
      updater->update_battery_soc(1.0);
      robot_cmd_1->updater = std::move(updater);
    });

  adapter.start();

  // Note: wait for task_manager to start, else TM will be suspicously "empty"
  std::this_thread::sleep_for(1s);

  rmf_task_msgs::msg::TaskProfile task_profile;
  task_profile.description.start_time = adapter.node()->now();
  task_profile.description.task_type.type =
    rmf_task_msgs::msg::TaskType::TYPE_LOOP;

  // Dsipatch Loop 0 Task
  task_profile.task_id = loop_0;
  task_profile.description.loop.num_loops = n_loops;
  task_profile.description.loop.robot_type = fleet_type;
  task_profile.description.loop.start_name = south;
  task_profile.description.loop.finish_name = east;
  adapter.dispatch_task(task_profile);

  // Dispatch Loop 1 Task
  task_profile.task_id = loop_1;
  task_profile.description.loop.start_name = north;
  task_profile.description.loop.finish_name = east;
  adapter.dispatch_task(task_profile);

  const auto task_0_completed_status = task_0_completed_future.wait_for(20s);
  CHECK(task_0_completed_status == std::future_status::ready);
  CHECK(at_least_one_incomplete_task_0);
  if (task_0_completed_status != std::future_status::ready)
  {
    std::cout << "Last " << loop_0 << " status (" << last_task_0_msg.task_id
              << "|" << last_task_0_msg.state << "): " << last_task_0_msg.status
              << std::endl;
  }

  const auto task_1_completed_status = task_1_completed_future.wait_for(20s);
  CHECK(task_1_completed_status == std::future_status::ready);
  CHECK(at_least_one_incomplete_task_1);
  if (task_1_completed_status != std::future_status::ready)
  {
    std::cout << "Last " << loop_1 << " status (" << last_task_1_msg.task_id
              << "|" << last_task_1_msg.state << "): " << last_task_1_msg.status
              << std::endl;
  }

  using VisitMap = std::unordered_map<std::size_t, std::size_t>;
  const auto visited_wp = [](std::size_t wp, const VisitMap& v, std::size_t num)
    {
      const auto it = v.find(wp);
      if (it == v.end())
        return false;

      return num <= it->second;
    };

  const auto visited_north = [&visited_wp](const VisitMap& v, std::size_t num)
    {
      return visited_wp(10, v, num);
    };

  const auto visited_east = [&visited_wp](const VisitMap& v, std::size_t num)
    {
      return visited_wp(7, v, num);
    };

  const auto visited_south = [&visited_wp](const VisitMap& v, std::size_t num)
    {
      return visited_wp(0, v, num);
    };

  // Note: I don't assume which robot will be selected for each loop request, so
  // I expect that either robot will be selected for either request, but that
  // each robot does get selected.
  const auto& v0 = robot_cmd_0->visited_wps();
  CHECK(robot_cmd_0->visited_wps().size() > 2);
  CHECK((visited_north(v0, n_loops) | visited_south(v0, n_loops)));
  CHECK(visited_east(v0, n_loops));
  CHECK(completed_0_count == 1);
  CHECK(finding_a_plan_0_count >= 2*n_loops - 1);
  if (finding_a_plan_0_count < 2*n_loops - 1)
  {
    // Note(MXG): This is a flaky test, so we're printing out some extra output
    // to hopefully offer more insight into what is happening. For some reason,
    // it seems that not all of the status messages are getting through to the
    // ROS subscriber, but I don't know why.
    std::cout << "The following plan finding statuses were received for "
              << loop_0 << ":\n";
    for (const auto& s : finding_a_plan_0_statuses)
      std::cout << " -- " << s << std::endl;
  }

  const auto& v1 = robot_cmd_1->visited_wps();
  CHECK(robot_cmd_1->visited_wps().size() > 2);
  CHECK((visited_north(v1, n_loops) | visited_south(v1, n_loops)));
  CHECK(visited_east(v1, n_loops));
  CHECK(completed_1_count == 1);
  CHECK(finding_a_plan_1_count >= 2*n_loops - 1);
  if (finding_a_plan_1_count < 2*n_loops - 1)
  {
    // Note(MXG): See previous note.
    std::cout << "The following plan finding statuses were received for "
              << loop_1 << ":";
    for (const auto& s : finding_a_plan_1_statuses)
      std::cout << " -- " << s << std::endl;
  }
}
