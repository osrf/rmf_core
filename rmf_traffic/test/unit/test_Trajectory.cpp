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

#include "utils_Trajectory.hpp"
#include <src/rmf_traffic/debug_Trajectory.hpp>
#include <rmf_utils/catch.hpp>
#include <iostream>

using namespace std::chrono_literals;

SCENARIO("Profile unit tests")
{
  // TODO(MXG): Test the new rmf_traffic::Profile API.
}

SCENARIO("Waypoint Unit Tests")
{
  // Waypoint Construction and Getters
  GIVEN("Construction values for Waypoints")
  {
    const auto time = std::chrono::steady_clock::now();
    const Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
    const Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);

    WHEN("Attemping to construct Waypoint using rmf_traffic::Trajectory::insert()")
    {
      rmf_traffic::Trajectory trajectory;
      auto result = trajectory.insert(time, pos, vel);

      const rmf_traffic::Trajectory::Waypoint &waypoint = *(result.it);

      THEN("Waypoint is constructed according to specifications.")
      {
        // From IteratorResult
        CHECK(result.inserted);
        CHECK(waypoint.time() == time);
        CHECK(waypoint.position() == pos);
        CHECK(waypoint.velocity() == vel);
      }
    }
  }

  // Waypoint Functions
  GIVEN("Sample Waypoint")
  {
    std::vector<TrajectoryInsertInput> inputs;
    rmf_traffic::Time time = std::chrono::steady_clock::now();
    inputs.push_back({time, UnitBox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)});
    inputs.push_back({time + 10s, UnitBox, Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1)});
    inputs.push_back({time + 20s, UnitBox, Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(0, 0, 0)});
    rmf_traffic::Trajectory trajectory = create_test_trajectory(inputs);
    rmf_traffic::Trajectory::iterator trajectory_it = trajectory.begin();
    rmf_traffic::Trajectory::Waypoint& waypoint = *trajectory_it;
    rmf_traffic::Trajectory::Waypoint& segment_10s = *(++trajectory_it);

    WHEN("Setting a new finish position using position() function")
    {
      const Eigen::Vector3d new_position = Eigen::Vector3d(1, 1, 1);
      waypoint.position(new_position);

      THEN("Finish position is updated successfully.")
      {
        CHECK(waypoint.position() == new_position);
      }
    }

    WHEN("Setting a new finish velocity using set_finish_velocity function")
    {
      const Eigen::Vector3d new_velocity = Eigen::Vector3d(1, 1, 1);
      waypoint.velocity(new_velocity);

      THEN("Finish velocity is updated successfully.")
      {
        CHECK(waypoint.velocity() == new_velocity);
      }
    }

    WHEN("Setting a new finish time using set_finish_time function")
    {
      const rmf_traffic::Time new_time = time + 5s;
      waypoint.change_time(new_time);

      THEN("Finish time is updated successfully.")
      {
        CHECK(waypoint.time() == new_time);
      }
    }

    WHEN("Setting a new finish time that conflicts with another waypoint")
    {
      const rmf_traffic::Time new_time = time + 10s;

      THEN("Error is thrown.")
      {
        CHECK_THROWS(waypoint.change_time(new_time));
      }
    }

    WHEN("Setting a new finish time that causes a rearrangement of adjacent segments")
    {
      const rmf_traffic::Time new_time = time + 12s;
      waypoint.change_time(new_time);

      THEN("The appropriate segments are rearranged")
      {
        int new_order[3] = {1, 0, 2};
        int i = 0;
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
          CHECK(it->position() == Eigen::Vector3d::Constant(new_order[i]));
      }
    }

    WHEN("Setting a new finish time that causes a rearrangement of non-adjacent segments")
    {
      const rmf_traffic::Time new_time = time + 22s;
      waypoint.change_time(new_time);

      THEN("The appropriate segments are rearranged")
      {
        int new_order[3] = {1, 2, 0};
        int i = 0;
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->position() == Eigen::Vector3d::Constant(new_order[i]));
        }
      }
    }

    WHEN("Positively adjusting all finish times using adjust_finish_times function, using first waypoint")
    {
      const std::chrono::seconds delta_t = std::chrono::seconds(5);
      waypoint.adjust_times(delta_t);
      int i = 0;
      const rmf_traffic::Time new_order[3] = {time + 5s, time + 15s, time + 25s};

      THEN("All finish times are adjusted correctly.")
      {
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->time() == new_order[i]);
        }
      }
    }

    WHEN("Negatively adjusting all finish times using adjust_finish_times function, using first waypoint")
    {
      const std::chrono::seconds delta_t = std::chrono::seconds(-5);
      waypoint.adjust_times(delta_t);
      int i = 0;
      const rmf_traffic::Time new_order[3] = {time - 5s, time + 5s, time + 15s};

      THEN("All finish times are adjusted correctly.")
      {
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->time() == new_order[i]);
        }
      }
    }

    WHEN("Large negative adjustment all finish times using adjust_finish_times function, using first waypoint")
    {
      const std::chrono::seconds delta_t = std::chrono::seconds(-50);
      waypoint.adjust_times(delta_t);
      int i = 0;
      const rmf_traffic::Time new_order[3] = {time - 50s, time - 40s, time - 30s};

      THEN("All finish times are adjusted correctly, as there is no waypoint preceding first waypoint")
      {
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->time() == new_order[i]);
        }
      }
    }

    WHEN("Positively adjusting all finish times using adjust_finish_times function, using second waypoint")
    {
      const std::chrono::seconds delta_t = std::chrono::seconds(5);
      segment_10s.adjust_times(delta_t);
      int i = 0;

      THEN("Finish times from the second waypoint on are adjusted correctly.")
      {
        const rmf_traffic::Time new_order[3] = {time, time + 15s, time + 25s};
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->time() == new_order[i]);
        }
      }
    }

    WHEN("Negatively adjusting all finish times using adjust_finish_times function, using second waypoint")
    {
      const std::chrono::seconds delta_t = std::chrono::seconds(-5);
      segment_10s.adjust_times(delta_t);
      int i = 0;

      THEN("All finish times are adjusted correctly.")
      {
        const rmf_traffic::Time new_order[3] = {time, time + 5s, time + 15s};
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->time() == new_order[i]);
        }
      }
    }

    WHEN("Large negative adjustment all finish times using adjust_finish_times function, using second waypoint")
    {
      const std::chrono::seconds delta_t = std::chrono::seconds(-50);

      THEN("std::invalid_argument exception thrown due to violation of previous waypoint time boundary")
      {
        CHECK_THROWS(segment_10s.adjust_times(delta_t));
      }
    }
  }
}

SCENARIO("Trajectory and base_iterator unit tests")
{
  // Trajectory construction
  GIVEN("Parameters for insert function")
  {
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    const Eigen::Vector3d pos_0 = Eigen::Vector3d(0, 0, 0);
    const Eigen::Vector3d vel_0 = Eigen::Vector3d(1, 1, 1);
    const Eigen::Vector3d pos_1 = Eigen::Vector3d(2, 2, 2);
    const Eigen::Vector3d vel_1 = Eigen::Vector3d(3, 3, 3);
    const Eigen::Vector3d pos_2 = Eigen::Vector3d(4, 4, 4);
    const Eigen::Vector3d vel_2 = Eigen::Vector3d(5, 5, 5);
    std::vector<TrajectoryInsertInput> param_inputs;
    param_inputs.push_back({time, UnitBox, pos_0, vel_0});
    param_inputs.push_back({time + 10s, UnitBox, pos_1, vel_1});
    param_inputs.push_back({time + 20s, UnitBox, pos_2, vel_2});

    WHEN("Construct empty trajectory")
    {
      rmf_traffic::Trajectory trajectory;

      THEN("Empty trajectory is created.")
      {
        CHECK(trajectory.begin() == trajectory.end());
        CHECK(trajectory.end() == trajectory.end());
      }
    }

    WHEN("Construct a length 1 trajectory")
    {
      rmf_traffic::Trajectory trajectory;
      auto result = trajectory.insert(time, pos_0, vel_0);
      const rmf_traffic::Trajectory::iterator zeroth_it = result.it;

      THEN("Length 1 trajectory is created.")
      {
        //base_iterator tests
        CHECK(result.inserted);
        CHECK(zeroth_it == trajectory.begin());
        CHECK(trajectory.begin() != trajectory.end());
        CHECK(zeroth_it != trajectory.end());
        CHECK(zeroth_it < trajectory.end());
        CHECK(zeroth_it <= trajectory.end());
        CHECK(trajectory.end() > zeroth_it);
        CHECK(trajectory.end() >= trajectory.end());

        CHECK(pos_0 == zeroth_it->position());
        CHECK(vel_0 == zeroth_it->velocity());
        CHECK(time == zeroth_it->time());
      }
    }

    WHEN("Construct a length 2 trajectory")
    {
      rmf_traffic::Trajectory trajectory;
      auto result = trajectory.insert(time, pos_0, vel_0);
      const rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time + 10s, pos_1, vel_1);
      const rmf_traffic::Trajectory::iterator first_it = result_1.it;

      THEN("Length 2 trajectory is created.")
      {
        //base_iterator tests
        CHECK(first_it == ++trajectory.begin());
        CHECK(first_it != trajectory.begin());
        CHECK(first_it > trajectory.begin());
        CHECK(first_it >= trajectory.begin());
        CHECK(trajectory.begin() < first_it);
        CHECK(trajectory.begin() <= first_it);

        CHECK(first_it != zeroth_it);
        CHECK(first_it > zeroth_it);
        CHECK(first_it >= zeroth_it);
        CHECK(zeroth_it < first_it);
        CHECK(zeroth_it <= first_it);

        CHECK(first_it != trajectory.end());
        CHECK(first_it < trajectory.end());
        CHECK(first_it <= trajectory.end());
        CHECK(trajectory.end() > first_it);
        CHECK(trajectory.end() >= first_it);

        CHECK(first_it->position() == pos_1);
        CHECK(first_it->velocity() == vel_1);
        CHECK(first_it->time() == time + 10s);
      }
    }

    WHEN("Inserting a waypoint with a unique finish_time violation")
    {
      rmf_traffic::Trajectory trajectory;
      auto result = trajectory.insert(time, pos_0, vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time, pos_1, vel_1);

      THEN("Returned result has inserted field set to false.")
      {
        CHECK(result_1.inserted == false);
        CHECK(result.it == result_1.it);
      }
    }

    WHEN("Copy Construction from another base_iterator")
    {
      rmf_traffic::Trajectory trajectory;
      auto result = trajectory.insert(time, pos_0, vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time + 10s, pos_1, vel_1);
      rmf_traffic::Trajectory::iterator first_it = result_1.it;

      THEN("New iterator is created")
      {
        const rmf_traffic::Trajectory::iterator copied_zeroth_it(zeroth_it);
        CHECK(&zeroth_it != &copied_zeroth_it);
        CHECK(zeroth_it == copied_zeroth_it);
      }
    }

    WHEN("Copy Construction from rvalue base_iterator")
    {
      rmf_traffic::Trajectory trajectory;
      auto result = trajectory.insert(time, pos_0, vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time + 10s, pos_1, vel_1);
      rmf_traffic::Trajectory::iterator first_it = result_1.it;

      THEN("New iterator is created")
      {
        const rmf_traffic::Trajectory::iterator &&rvalue_it = std::move(zeroth_it);
        const rmf_traffic::Trajectory::iterator copied_zeroth_it(rvalue_it);
        CHECK(&zeroth_it != &copied_zeroth_it);
      }
    }

    WHEN("Move Construction from another base_iterator")
    {
      rmf_traffic::Trajectory trajectory;
      auto result = trajectory.insert(time, pos_0, vel_0);
      const rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time + 10s, pos_1, vel_1);
      const rmf_traffic::Trajectory::iterator first_it = result_1.it;

      THEN("New iterator is created")
      {
        const rmf_traffic::Trajectory::iterator copied_zeroth_it(zeroth_it);
        const rmf_traffic::Trajectory::iterator moved_zeroth_it(std::move(copied_zeroth_it));
        CHECK(&zeroth_it != &moved_zeroth_it);
        CHECK(zeroth_it == moved_zeroth_it);
      }
    }

    WHEN("Copy Construction of Trajectory from another trajectory")
    {
      const rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      const rmf_traffic::Trajectory trajectory_copy = trajectory;

      THEN("Elements of trajectories are consistent")
      {
        rmf_traffic::Trajectory::const_iterator ot = trajectory.begin();
        rmf_traffic::Trajectory::const_iterator ct = trajectory_copy.begin();
        for (; ot != trajectory.end() && ct != trajectory.end(); ++ot, ++ct)
        {
          CHECK(ot->position() == ct->position());
          CHECK(ot->velocity() == ct->velocity());
          CHECK(ot->time() == ct->time());
        }
        CHECK(ot == trajectory.end());
        CHECK(ct == trajectory_copy.end());
      }
    }

    WHEN("Copy Construction of Trajectory followed by move of source trajectory")
    {
      const rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory trajectory_copy = trajectory;
      const rmf_traffic::Trajectory trajectory_moved = std::move(trajectory);

      THEN("Elements of trajectories are consistent")
      {
        rmf_traffic::Trajectory::const_iterator ct = trajectory_copy.begin();
        rmf_traffic::Trajectory::const_iterator mt = trajectory_moved.begin();
        for (; ct != trajectory_copy.end() && mt != trajectory_moved.end(); ++ct, ++mt)
        {
          CHECK(ct->position() == mt->position());
          CHECK(ct->velocity() == mt->velocity());
          CHECK(ct->time() == mt->time());
        }
        CHECK(ct == trajectory_copy.end());
        CHECK(mt == trajectory_moved.end());
      }
    }

    WHEN("Appending waypoint to trajectory")
    {
      rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory::iterator first_it = trajectory.begin();
      rmf_traffic::Trajectory::iterator second_it = trajectory.find(time + 10s);
      rmf_traffic::Trajectory::iterator third_it = trajectory.find(time + 20s);
      const rmf_traffic::Time time_3 = time + 30s;
      const Eigen::Vector3d pos_3 = Eigen::Vector3d(6, 6, 6);
      const Eigen::Vector3d vel_3 = Eigen::Vector3d(7, 7, 7);
      auto fourth_it = trajectory.insert(time_3, pos_3, vel_3).it;

      THEN("base_iterators assigned prior are still valid")
      {
        CHECK(first_it->time() == time);
        CHECK(second_it->time() == time + 10s);
        CHECK(third_it->time() == time + 20s);
        CHECK(fourth_it->time() == time + 30s);

        CHECK(first_it == trajectory.begin());
        CHECK(++first_it == second_it);
        CHECK(++second_it == third_it);
        CHECK(++third_it == fourth_it);
        CHECK(++fourth_it == trajectory.end());
      }
    }

    WHEN("Prepending waypoint to trajectory")
    {
      rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory::iterator first_it = trajectory.begin();
      rmf_traffic::Trajectory::iterator second_it = trajectory.find(time + 10s);
      rmf_traffic::Trajectory::iterator third_it = trajectory.find(time + 20s);
      const rmf_traffic::Time time_3 = time - 30s;
      const Eigen::Vector3d pos_3 = Eigen::Vector3d(6, 6, 6);
      const Eigen::Vector3d vel_3 = Eigen::Vector3d(7, 7, 7);
      auto fourth_it = trajectory.insert(time_3, pos_3, vel_3).it;

      THEN("base_iterators assigned prior are still valid")
      {
        CHECK(first_it->time() == time);
        CHECK(second_it->time() == time + 10s);
        CHECK(third_it->time() == time + 20s);
        CHECK(fourth_it->time() == time - 30s);

        CHECK(fourth_it == trajectory.begin());
        CHECK(++fourth_it == first_it);
        CHECK(++first_it == second_it);
        CHECK(++second_it == third_it);
        CHECK(++third_it == trajectory.end());
      }
    }

    WHEN("Interpolating waypoint to trajectory")
    {
      rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory::iterator first_it = trajectory.begin();
      rmf_traffic::Trajectory::iterator second_it = trajectory.find(time + 10s);
      rmf_traffic::Trajectory::iterator third_it = trajectory.find(time + 20s);
      const rmf_traffic::Time time_3 = time + 15s;
      const Eigen::Vector3d pos_3 = Eigen::Vector3d(6, 6, 6);
      const Eigen::Vector3d vel_3 = Eigen::Vector3d(7, 7, 7);
      auto fourth_it = trajectory.insert(time_3, pos_3, vel_3).it;

      THEN("base_iterators assigned prior are still valid")
      {
        CHECK(first_it->time() == time);
        CHECK(second_it->time() == time + 10s);
        CHECK(fourth_it->time() == time + 15s);
        CHECK(third_it->time() == time + 20s);

        CHECK(first_it == trajectory.begin());
        CHECK(++first_it == second_it);
        CHECK(++second_it == fourth_it);
        CHECK(++fourth_it == third_it);
        CHECK(++third_it == trajectory.end());
      }
    }
  }
  // Trajectory functions
  GIVEN("Sample Trajectories")
  {
    std::vector<TrajectoryInsertInput> param_inputs;
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    param_inputs.push_back({time, UnitBox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)});
    param_inputs.push_back({time + 10s, UnitBox, Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(3, 3, 3)});
    param_inputs.push_back({time + 20s, UnitBox, Eigen::Vector3d(4, 4, 4), Eigen::Vector3d(5, 5, 5)});
    rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
    const rmf_traffic::Trajectory empty_trajectory = create_test_trajectory();

    WHEN("Finding a waypoint at the precise time specified")
    {
      THEN("Waypoint is retreieved successfully")
      {
        CHECK(trajectory.find(time)->position() == Eigen::Vector3d(0, 0, 0));
        CHECK(trajectory.find(time + 10s)->position() == Eigen::Vector3d(2, 2, 2));
        CHECK(trajectory.find(time + 20s)->position() == Eigen::Vector3d(4, 4, 4));
      }
    }

    WHEN("Finding a waypoint at an offset time")
    {
      THEN("Waypoints currently active are retrieved successfully")
      {
        CHECK(trajectory.find(time)->position() == Eigen::Vector3d(0, 0, 0));
        CHECK(trajectory.find(time + 2s)->position() == Eigen::Vector3d(2, 2, 2));
        CHECK(trajectory.find(time + 8s)->position() == Eigen::Vector3d(2, 2, 2));
        CHECK(trajectory.find(time + 12s)->position() == Eigen::Vector3d(4, 4, 4));
        CHECK(trajectory.find(time + 20s)->position() == Eigen::Vector3d(4, 4, 4));
      }
    }

    WHEN("Finding a waypoint at an out of bounds time")
    {
      THEN("rmf_traffic::Trajectory::end() is returned")
      {
        CHECK(trajectory.find(time - 50s) == trajectory.end());
        CHECK(trajectory.find(time + 50s) == trajectory.end());
      }
    }

    WHEN("Erasing a first waypoint")
    {
      THEN("Waypoint is erased and trajectory is rearranged")
      {
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_target = trajectory.begin();
        rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_target);
        CHECK(next_it->time() == time + 10s);
        CHECK(trajectory.size() == 2);
      }
    }

    WHEN("Erasing a first waypoint from a copy")
    {
      THEN("Waypoint is erased and only copy is updated, source is unaffected")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_target = trajectory_copy.begin();
        const rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_target);
        CHECK(next_it->time() == time + 10s);
        CHECK(trajectory_copy.size() == 2);
        CHECK(trajectory.size() == 3);
      }
    }

    WHEN("Erasing a second waypoint")
    {
      THEN("Waypoint is erased and trajectory is rearranged")
      {
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_target = ++(trajectory.begin());
        const rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_target);
        CHECK(next_it->time() == time + 20s);
        CHECK(trajectory.size() == 2);
      }
    }

    WHEN("Erasing a second waypoint from a copy")
    {
      THEN("Waypoint is erased and only copy is updated, source is unaffected")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_target = ++(trajectory_copy.begin());
        const rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_target);
        CHECK(next_it->time() == time + 20s);
        CHECK(trajectory_copy.size() == 2);
        CHECK(trajectory.size() == 3);
      }
    }

    WHEN("Erasing a empty range of segments using range notation")
    {
      THEN("Nothing is erased and current iterator is returned")
      {
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        const rmf_traffic::Trajectory::iterator erase_last = erase_first;
        const rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 3);
        CHECK(next_it->time() == time);
      }
    }

    WHEN("Erasing a empty range of segments from a copy using range notation")
    {
      THEN("Nothing is erased")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        const rmf_traffic::Trajectory::iterator erase_last = erase_first;
        const rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        CHECK(next_it->time() == time);
      }
    }

    WHEN("Erasing the first waypoint using range notation")
    {
      THEN("1 Waypoint is erased and trajectory is rearranged")
      {
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        const rmf_traffic::Trajectory::iterator erase_last = trajectory.find(time + 10s);
        const rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 2);
        CHECK(next_it->time() == time + 10s);
      }
    }

    WHEN("Erasing the first waypoint of a copy using range notation")
    {
      THEN("1 Waypoint is erased and trajectory is rearranged")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        const rmf_traffic::Trajectory::iterator erase_last = trajectory.find(time + 10s);
        const rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 2);
        CHECK(next_it->time() == time + 10s);
      }
    }

    WHEN("Erasing the first and second segments using range notation")
    {
      THEN("2 Waypoints are erased and trajectory is rearranged")
      {
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        const rmf_traffic::Trajectory::iterator erase_last = trajectory.find(time + 20s);
        const rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 1);
        CHECK(next_it->time() == time + 20s);
      }
    }

    WHEN("Erasing the first and second segments of a copy using range notation")
    {
      THEN("2 Waypoints are erased and trajectory is rearranged")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        const rmf_traffic::Trajectory::iterator erase_last = trajectory.find(time + 20s);
        const rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 1);
        CHECK(next_it->time() == time + 20s);
      }
    }

    WHEN("Erasing all segments using range notation")
    {
      THEN("All Waypoints are erased and trajectory is empty")
      {
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        const rmf_traffic::Trajectory::iterator erase_last = trajectory.end();
        const rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 0);
        CHECK(next_it == trajectory.end());
      }
    }

    WHEN("Erasing all but last waypoint using range notation")
    {
      THEN("All but one Waypoint is erased")
      {
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        const rmf_traffic::Trajectory::iterator erase_last = --(trajectory.end());
        const rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 1);
        CHECK(next_it == trajectory.begin());
        CHECK(next_it == --trajectory.end());
      }
    }

    WHEN("Erasing all segments of a copy using range notation")
    {
      THEN("All Waypoints are erased and trajectory is empty")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory_copy.begin();
        const rmf_traffic::Trajectory::iterator erase_last = trajectory_copy.end();
        const rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 0);
        CHECK(next_it == trajectory_copy.end());
      }
    }

    WHEN("Erasing all but last waypoint of a copy using range notation")
    {
      THEN("All but one Waypoint is erased")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        const rmf_traffic::Trajectory::iterator erase_first = trajectory_copy.begin();
        const rmf_traffic::Trajectory::iterator erase_last = --(trajectory_copy.end());
        const rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 1);
        CHECK(next_it == trajectory_copy.begin());
        CHECK(next_it == --trajectory_copy.end());
      }
    }

    WHEN("Getting the first iterator of empty trajectory")
    {
      THEN("trajectory.end() is returned")
      {
        CHECK(empty_trajectory.begin() == empty_trajectory.end());
      }
    }

    WHEN("Getting start_time of empty trajectory using start_time function")
    {
      THEN("nullptr is returned")
      {
        CHECK(empty_trajectory.start_time() == nullptr);
      }
    }

    WHEN("Getting start_time of trajectory using start_time function")
    {
      THEN("Start time is returned")
      {
        REQUIRE(trajectory.start_time() != nullptr);
        CHECK(*(trajectory.start_time()) == time);
      }
    }

    WHEN("Getting finish_time of empty trajectory using finish_time function")
    {
      THEN("nullptr is returned")
      {
        CHECK(empty_trajectory.finish_time() == nullptr);
      }
    }

    WHEN("Getting finish_time of trajectory using finish_time function")
    {
      THEN("finish time is returned")
      {
        REQUIRE(trajectory.start_time() != nullptr);
        CHECK(*(trajectory.finish_time()) == time + 20s);
      }
    }

    WHEN("Getting duration of empty trajectory using duration function")
    {
      THEN("0 is returned")
      {
        CHECK(empty_trajectory.duration() == std::chrono::seconds(0));
      }
    }

    WHEN("Getting duration of trajectory using duration function")
    {
      THEN("duration is returned")
      {
        CHECK(trajectory.duration() == std::chrono::seconds(20));
      }
    }
  }
}
