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
  // Profile Construction and Getters
  GIVEN("Construction values for Profile")
  {
    std::shared_ptr<rmf_traffic::geometry::Box> unitBox_shape = std::make_shared<rmf_traffic::geometry::Box>(1.0, 1.0);
    std::shared_ptr<rmf_traffic::geometry::Circle> unitCircle_shape = std::make_shared<rmf_traffic::geometry::Circle>(1.0);
    std::string queue_number = "5";

    WHEN("Constructing a Profile given shape and agency")
    {
      rmf_traffic::Trajectory::ProfilePtr strict_profile = rmf_traffic::Trajectory::Profile::make_strict(unitBox_shape);
      rmf_traffic::Trajectory::ProfilePtr queue_profile = rmf_traffic::Trajectory::Profile::make_queued(unitCircle_shape, queue_number);

      THEN("Profile is constructed according to specifications.")
      {
        CHECK(strict_profile->get_shape() == unitBox_shape);
        CHECK(strict_profile->get_agency() == rmf_traffic::Trajectory::Profile::Agency::Strict);
        CHECK(strict_profile->get_queue_info() == nullptr);

        CHECK(queue_profile->get_shape() == unitCircle_shape);
        CHECK(queue_profile->get_agency() == rmf_traffic::Trajectory::Profile::Agency::Queued);
        CHECK(queue_profile->get_queue_info()->get_queue_id() == queue_number);
      }
    }

    WHEN("Shape object used for profile construction is changed")
    {
      rmf_traffic::Trajectory::ProfilePtr strict_profile = rmf_traffic::Trajectory::Profile::make_strict(unitBox_shape);
      *unitBox_shape = rmf_traffic::geometry::Box(2.0, 2.0);

      THEN("Profile shape is updated")
      {
        CHECK(strict_profile->get_shape() == unitBox_shape);
        const auto &box = static_cast<const rmf_traffic::geometry::Box &>(*strict_profile->get_shape());
        CHECK(box.get_x_length() == 2.0);
        CHECK(box.get_y_length() == 2.0);
      }
    }

    WHEN("Pointer for shape used for profile construction is changed")
    {
      rmf_traffic::Trajectory::ProfilePtr strict_profile = rmf_traffic::Trajectory::Profile::make_strict(unitBox_shape);
      rmf_traffic::geometry::Box *ptr_address = unitBox_shape.get();
      unitBox_shape = std::make_shared<rmf_traffic::geometry::Box>(2.0, 2.0);

      THEN("Profile shape is unaffected")
      {
        CHECK(strict_profile->get_shape() != unitBox_shape);
        CHECK(strict_profile->get_shape().get() == ptr_address);
      }
    }

    WHEN("Shape object used for profile construction is moved")
    {
      // Move constructor
      rmf_traffic::Trajectory::ProfilePtr strict_profile = rmf_traffic::Trajectory::Profile::make_strict(unitBox_shape);
      std::shared_ptr<rmf_traffic::geometry::Box> new_unitBox_shape = std::move(unitBox_shape);

      THEN("Profile shape is unaffected")
      {
        CHECK(strict_profile->get_shape() == new_unitBox_shape);
      }
    }

    WHEN("Queue number used for profile construction is changed")
    {
      std::string queue_number = "5";
      rmf_traffic::Trajectory::ProfilePtr strict_profile = rmf_traffic::Trajectory::Profile::make_queued(unitBox_shape, queue_number);
      queue_number = "6";

      THEN("Queue number is unaffected")
      {
        CHECK(strict_profile->get_queue_info()->get_queue_id() == "5");
      }
    }
  }

  // Profile Function Tests
  GIVEN("Sample Profiles and Shapes")
  {
    rmf_traffic::Trajectory::ProfilePtr strict_unitbox_profile = create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict);
    rmf_traffic::Trajectory::ProfilePtr queued_unitCircle_profile = create_test_profile(UnitCircle, rmf_traffic::Trajectory::Profile::Agency::Queued, "3");
    std::shared_ptr<rmf_traffic::geometry::Box> new_Box_shape = std::make_shared<rmf_traffic::geometry::Box>(2.0, 2.0);

    WHEN("Profile agency is changed using API set_to_* function")
    {
      THEN("Profile agency is successfully changed")
      {
        CHECK(strict_unitbox_profile->get_agency() == rmf_traffic::Trajectory::Profile::Agency::Strict);
        CHECK(strict_unitbox_profile->get_queue_info() == nullptr);

        strict_unitbox_profile->set_to_autonomous();
        CHECK(strict_unitbox_profile->get_agency() == rmf_traffic::Trajectory::Profile::Agency::Autonomous);
        CHECK(strict_unitbox_profile->get_queue_info() == nullptr);

        strict_unitbox_profile->set_to_queued("2");
        CHECK(strict_unitbox_profile->get_agency() == rmf_traffic::Trajectory::Profile::Agency::Queued);
        REQUIRE(strict_unitbox_profile->get_queue_info() != nullptr);
        CHECK(strict_unitbox_profile->get_queue_info()->get_queue_id() == "2");

        strict_unitbox_profile->set_to_strict();
        CHECK(strict_unitbox_profile->get_agency() == rmf_traffic::Trajectory::Profile::Agency::Strict);
        CHECK(strict_unitbox_profile->get_queue_info() == nullptr);
      }
    }

    WHEN("Changing profile shapes using API set_shape function")
    {
      CHECK(strict_unitbox_profile->get_shape() != new_Box_shape);
      strict_unitbox_profile->set_shape(new_Box_shape);

      THEN("ProfilePtr is updated accordingly.")
      {
        CHECK(strict_unitbox_profile->get_shape() == new_Box_shape);
      }
    }
  }
}

SCENARIO("Segment Unit Tests")
{
  // Segment Construction and Getters
  GIVEN("Construction values for Segments")
  {
    rmf_traffic::Trajectory::ProfilePtr strict_unitbox_profile = create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict);
    rmf_traffic::Trajectory::ProfilePtr queued_unitCircle_profile = create_test_profile(UnitCircle, rmf_traffic::Trajectory::Profile::Agency::Queued, "3");
    const auto time = std::chrono::steady_clock::now();
    const Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
    const Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);

    WHEN("Attemping to construct Segment using rmf_traffic::Trajectory::insert()")
    {
      rmf_traffic::Trajectory trajectory{"test_map"};
      auto result = trajectory.insert(time, strict_unitbox_profile, pos, vel);

      const rmf_traffic::Trajectory::Segment &segment = *(result.it);

      THEN("Segment is constructed according to specifications.")
      {
        // From IteratorResult
        CHECK(result.inserted);
        CHECK(segment.get_finish_time() == time);
        CHECK(segment.get_finish_position() == pos);
        CHECK(segment.get_finish_velocity() == vel);
        CHECK(segment.get_profile() == strict_unitbox_profile);
      }
    }

    WHEN("Profile used for construction is changed")
    {
      rmf_traffic::Trajectory trajectory{"test_map"};
      auto result = trajectory.insert(time, strict_unitbox_profile, pos, vel);
      const rmf_traffic::Trajectory::Segment &segment = *(result.it);

      *strict_unitbox_profile = *queued_unitCircle_profile;

      THEN("Segment profile is updated.")
      {
        CHECK(segment.get_profile() == strict_unitbox_profile);
        const auto &circle = static_cast<const rmf_traffic::geometry::Circle &>(*strict_unitbox_profile->get_shape());
        CHECK(circle.get_radius() == 1.0);
      }
    }

    WHEN("Pointer for profile used for construction is changed")
    {
      rmf_traffic::Trajectory trajectory{"test_map"};
      auto result = trajectory.insert(time, strict_unitbox_profile, pos, vel);
      const rmf_traffic::Trajectory::Segment &segment = *(result.it);

      rmf_traffic::Trajectory::ProfilePtr new_profile = std::move(strict_unitbox_profile);

      THEN("Segment profile is updated")
      {
        CHECK(segment.get_profile() != strict_unitbox_profile);
        CHECK(segment.get_profile() == new_profile);
      }
    }

    WHEN("Profile used for construction is moved")
    {
      rmf_traffic::Trajectory trajectory{"test_map"};
      auto result = trajectory.insert(time, strict_unitbox_profile, pos, vel);
      const rmf_traffic::Trajectory::Segment &segment = *(result.it);

      rmf_traffic::Trajectory::ProfilePtr new_profile = std::move(strict_unitbox_profile);

      THEN("Segment profile is updated")
      {
        CHECK(segment.get_profile() != strict_unitbox_profile);
        CHECK(segment.get_profile() == new_profile);
      }
    }
  }

  // Segment Functions
  GIVEN("Sample Segment")
  {
    std::vector<TrajectoryInsertInput> inputs;
    rmf_traffic::Time time = std::chrono::steady_clock::now();
    inputs.push_back({time, UnitBox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)});
    inputs.push_back({time + 10s, UnitBox, Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1)});
    inputs.push_back({time + 20s, UnitBox, Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(0, 0, 0)});
    rmf_traffic::Trajectory trajectory = create_test_trajectory(inputs);
    rmf_traffic::Trajectory::iterator trajectory_it = trajectory.begin();
    rmf_traffic::Trajectory::Segment &segment = *trajectory_it;
    rmf_traffic::Trajectory::Segment &segment_10s = *(++trajectory_it);

    WHEN("Setting a new profile using set_profile function")
    {
      rmf_traffic::Trajectory::ProfilePtr new_profile = create_test_profile(UnitCircle, rmf_traffic::Trajectory::Profile::Agency::Autonomous);
      segment.set_profile(new_profile);

      THEN("Profile is updated successfully.")
      {
        CHECK(segment.get_profile() == new_profile);
      }
    }

    WHEN("Setting a new finish position using set_finish_position function")
    {
      Eigen::Vector3d new_position = Eigen::Vector3d(1, 1, 1);
      segment.set_finish_position(new_position);

      THEN("Finish position is updated successfully.")
      {
        CHECK(segment.get_finish_position() == new_position);
      }
    }

    WHEN("Setting a new finish velocity using set_finish_velocity function")
    {
      Eigen::Vector3d new_velocity = Eigen::Vector3d(1, 1, 1);
      segment.set_finish_velocity(new_velocity);

      THEN("Finish velocity is updated successfully.")
      {
        CHECK(segment.get_finish_velocity() == new_velocity);
      }
    }

    WHEN("Setting a new finish time using set_finish_time function")
    {
      rmf_traffic::Time new_time = time + 5s;
      segment.set_finish_time(new_time);

      THEN("Finish time is updated successfully.")
      {
        CHECK(segment.get_finish_time() == new_time);
      }
    }

    WHEN("Setting a new finish time that conflicts with another segment")
    {
      rmf_traffic::Time new_time = time + 10s;

      THEN("Error is thrown.")
      {
        CHECK_THROWS(segment.set_finish_time(new_time));
      }
    }

    WHEN("Setting a new finish time that causes a rearrangement of adjacent segments")
    {
      rmf_traffic::Time new_time = time + 12s;
      segment.set_finish_time(new_time);

      THEN("The appropriate segments are rearranged")
      {
        int new_order[3] = {1, 0, 2};
        int i = 0;
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
          CHECK(it->get_finish_position() == Eigen::Vector3d(new_order[i],
                                                             new_order[i],
                                                             new_order[i]));
      }
    }

    WHEN("Setting a new finish time that causes a rearrangement of non-adjacent segments")
    {
      rmf_traffic::Time new_time = time + 22s;
      segment.set_finish_time(new_time);

      THEN("The appropriate segments are rearranged")
      {
        int new_order[3] = {1, 2, 0};
        int i = 0;
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->get_finish_position() == Eigen::Vector3d(new_order[i],
                                                             new_order[i],
                                                             new_order[i]));
        }
      }
    }

    WHEN("Positively adjusting all finish times using adjust_finish_times function, using first segment")
    {
      std::chrono::seconds delta_t = std::chrono::seconds(5);
      segment.adjust_finish_times(delta_t);
      int i = 0;
      rmf_traffic::Time new_order[3] = {time + 5s, time + 15s, time + 25s};

      THEN("All finish times are adjusted correctly.")
      {
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->get_finish_time() == new_order[i]);
        }
      }
    }

    WHEN("Negatively adjusting all finish times using adjust_finish_times function, using first segment")
    {
      std::chrono::seconds delta_t = std::chrono::seconds(-5);
      segment.adjust_finish_times(delta_t);
      int i = 0;
      rmf_traffic::Time new_order[3] = {time - 5s, time + 5s, time + 15s};

      THEN("All finish times are adjusted correctly.")
      {
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->get_finish_time() == new_order[i]);
        }
      }
    }

    WHEN("Large negative adjustment all finish times using adjust_finish_times function, using first segment")
    {
      std::chrono::seconds delta_t = std::chrono::seconds(-50);
      segment.adjust_finish_times(delta_t);
      int i = 0;
      rmf_traffic::Time new_order[3] = {time - 50s, time - 40s, time - 30s};

      THEN("All finish times are adjusted correctly, as there is no segment preceding first segment")
      {
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->get_finish_time() == new_order[i]);
        }
      }
    }

    WHEN("Positively adjusting all finish times using adjust_finish_times function, using second segment")
    {
      std::chrono::seconds delta_t = std::chrono::seconds(5);
      segment_10s.adjust_finish_times(delta_t);
      int i = 0;

      THEN("Finish times from the second segment on are adjusted correctly.")
      {
        rmf_traffic::Time new_order[3] = {time, time + 15s, time + 25s};
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->get_finish_time() == new_order[i]);
        }
      }
    }

    WHEN("Negatively adjusting all finish times using adjust_finish_times function, using second segment")
    {
      std::chrono::seconds delta_t = std::chrono::seconds(-5);
      segment_10s.adjust_finish_times(delta_t);
      int i = 0;

      THEN("All finish times are adjusted correctly.")
      {
        rmf_traffic::Time new_order[3] = {time, time + 5s, time + 15s};
        for (rmf_traffic::Trajectory::iterator it = trajectory.begin(); it != trajectory.end(); it++, i++)
        {
          CHECK(it->get_finish_time() == new_order[i]);
        }
      }
    }

    WHEN("Large negative adjustment all finish times using adjust_finish_times function, using second segment")
    {
      std::chrono::seconds delta_t = std::chrono::seconds(-50);

      THEN("std::invalid_argument exception thrown due to violation of previous segment time boundary")
      {
        CHECK_THROWS(segment_10s.adjust_finish_times(delta_t));
      }
    }
  }
}

SCENARIO("Trajectory and base_iterator unit tests")
{
  // Trajectory construction
  GIVEN("Parameters for insert function")
  {
    rmf_traffic::Time time = std::chrono::steady_clock::now();
    Eigen::Vector3d pos_0 = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel_0 = Eigen::Vector3d(1, 1, 1);
    Eigen::Vector3d pos_1 = Eigen::Vector3d(2, 2, 2);
    Eigen::Vector3d vel_1 = Eigen::Vector3d(3, 3, 3);
    Eigen::Vector3d pos_2 = Eigen::Vector3d(4, 4, 4);
    Eigen::Vector3d vel_2 = Eigen::Vector3d(5, 5, 5);
    std::vector<TrajectoryInsertInput> param_inputs;
    param_inputs.push_back({time, UnitBox, pos_0, vel_0});
    param_inputs.push_back({time + 10s, UnitBox, pos_1, vel_1});
    param_inputs.push_back({time + 20s, UnitBox, pos_2, vel_2});

    WHEN("Construct empty trajectory")
    {
      rmf_traffic::Trajectory trajectory("test_map");

      THEN("Empty trajectory is created.")
      {
        CHECK(trajectory.begin() == trajectory.end());
        CHECK(trajectory.end() == trajectory.end());
      }
    }

    WHEN("Construct a length 1 trajectory")
    {
      rmf_traffic::Trajectory trajectory("test_map");
      auto result = trajectory.insert(
          time, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
          pos_0, vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;

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

        CHECK(pos_0 == zeroth_it->get_finish_position());
        CHECK(vel_0 == zeroth_it->get_finish_velocity());
        CHECK(time == zeroth_it->get_finish_time());
      }
    }

    WHEN("Construct a length 2 trajectory")
    {
      rmf_traffic::Trajectory trajectory("test_map");
      auto result = trajectory.insert(time, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                      pos_0,
                                      vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time + 10s, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                        pos_1,
                                        vel_1);
      rmf_traffic::Trajectory::iterator first_it = result_1.it;

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

        CHECK(first_it->get_finish_position() == pos_1);
        CHECK(first_it->get_finish_velocity() == vel_1);
        CHECK(first_it->get_finish_time() == time + 10s);
      }
    }

    WHEN("Inserting a segment with a unique finish_time violation")
    {
      rmf_traffic::Trajectory trajectory("test_map");
      auto result = trajectory.insert(time, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                      pos_0,
                                      vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                        pos_1,
                                        vel_1);

      THEN("Returned result has inserted field set to false.")
      {
        CHECK(result_1.inserted == false);
        CHECK(result.it == result_1.it);
      }
    }

    WHEN("Copy Construction from another base_iterator")
    {
      rmf_traffic::Trajectory trajectory("test_map");
      auto result = trajectory.insert(time, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                      pos_0,
                                      vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time + 10s, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                        pos_1,
                                        vel_1);
      rmf_traffic::Trajectory::iterator first_it = result_1.it;

      THEN("New iterator is created")
      {
        rmf_traffic::Trajectory::iterator copied_zeroth_it(zeroth_it);
        CHECK(&zeroth_it != &copied_zeroth_it);
        CHECK(copied_zeroth_it->get_profile() == zeroth_it->get_profile());
        CHECK(zeroth_it == copied_zeroth_it);
      }
    }

    WHEN("Copy Construction from rvalue base_iterator")
    {
      rmf_traffic::Trajectory trajectory("test_map");
      auto result = trajectory.insert(time, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                      pos_0,
                                      vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time + 10s, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                        pos_1,
                                        vel_1);
      rmf_traffic::Trajectory::iterator first_it = result_1.it;

      THEN("New iterator is created")
      {
        rmf_traffic::Trajectory::iterator &&rvalue_it = std::move(zeroth_it);
        rmf_traffic::Trajectory::iterator copied_zeroth_it(rvalue_it);
        CHECK(&zeroth_it != &copied_zeroth_it);
        CHECK(copied_zeroth_it->get_profile() == zeroth_it->get_profile());
      }
    }

    WHEN("Move Construction from another base_iterator")
    {
      rmf_traffic::Trajectory trajectory("test_map");
      auto result = trajectory.insert(time, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                      pos_0,
                                      vel_0);
      rmf_traffic::Trajectory::iterator zeroth_it = result.it;
      auto result_1 = trajectory.insert(time + 10s, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                        pos_1,
                                        vel_1);
      rmf_traffic::Trajectory::iterator first_it = result_1.it;

      THEN("New iterator is created")
      {
        rmf_traffic::Trajectory::iterator copied_zeroth_it(zeroth_it);
        rmf_traffic::Trajectory::iterator moved_zeroth_it(std::move(copied_zeroth_it));
        CHECK(&zeroth_it != &moved_zeroth_it);
        CHECK(zeroth_it == moved_zeroth_it);
        CHECK(moved_zeroth_it->get_profile() == zeroth_it->get_profile());
      }
    }

    WHEN("Copy Construction of Trajectory from another trajectory")
    {
      rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory trajectory_copy = trajectory;

      THEN("Elements of trajectories are consistent")
      {
        rmf_traffic::Trajectory::const_iterator ot = trajectory.begin();
        rmf_traffic::Trajectory::const_iterator ct = trajectory_copy.begin();
        for (; ot != trajectory.end() && ct != trajectory.end(); ++ot, ++ct)
        {
          CHECK(ot->get_profile() == ct->get_profile());
          CHECK(ot->get_finish_position() == ct->get_finish_position());
          CHECK(ot->get_finish_velocity() == ct->get_finish_velocity());
          CHECK(ot->get_finish_time() == ct->get_finish_time());
        }
        CHECK(ot == trajectory.end());
        CHECK(ct == trajectory_copy.end());
      }
    }

    WHEN("Copy Construction of Trajectory followed by move of source trajectory")
    {
      rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory trajectory_copy = trajectory;
      rmf_traffic::Trajectory trajectory_moved = std::move(trajectory);

      THEN("Elements of trajectories are consistent")
      {
        rmf_traffic::Trajectory::const_iterator ct = trajectory_copy.begin();
        rmf_traffic::Trajectory::const_iterator mt = trajectory_moved.begin();
        for (; ct != trajectory_copy.end() && mt != trajectory_moved.end(); ++ct, ++mt)
        {
          CHECK(ct->get_profile() == mt->get_profile());
          CHECK(ct->get_finish_position() == mt->get_finish_position());
          CHECK(ct->get_finish_velocity() == mt->get_finish_velocity());
          CHECK(ct->get_finish_time() == mt->get_finish_time());
        }
        CHECK(ct == trajectory_copy.end());
        CHECK(mt == trajectory_moved.end());
      }
    }

    WHEN("Appending segment to trajectory")
    {
      rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory::iterator first_it = trajectory.begin();
      rmf_traffic::Trajectory::iterator second_it = trajectory.find(time + 10s);
      rmf_traffic::Trajectory::iterator third_it = trajectory.find(time + 20s);
      rmf_traffic::Time time_3 = time + 30s;
      Eigen::Vector3d pos_3 = Eigen::Vector3d(6, 6, 6);
      Eigen::Vector3d vel_3 = Eigen::Vector3d(7, 7, 7);
      rmf_traffic::Trajectory::iterator fourth_it = trajectory.insert(
                                                                  time_3, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                                                  pos_3, vel_3)
                                                        .it;

      THEN("base_iterators assigned prior are still valid")
      {
        CHECK(first_it->get_finish_time() == time);
        CHECK(second_it->get_finish_time() == time + 10s);
        CHECK(third_it->get_finish_time() == time + 20s);
        CHECK(fourth_it->get_finish_time() == time + 30s);

        CHECK(first_it == trajectory.begin());
        CHECK(++first_it == second_it);
        CHECK(++second_it == third_it);
        CHECK(++third_it == fourth_it);
        CHECK(++fourth_it == trajectory.end());
      }
    }

    WHEN("Prepending segment to trajectory")
    {
      rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory::iterator first_it = trajectory.begin();
      rmf_traffic::Trajectory::iterator second_it = trajectory.find(time + 10s);
      rmf_traffic::Trajectory::iterator third_it = trajectory.find(time + 20s);
      rmf_traffic::Time time_3 = time - 30s;
      Eigen::Vector3d pos_3 = Eigen::Vector3d(6, 6, 6);
      Eigen::Vector3d vel_3 = Eigen::Vector3d(7, 7, 7);
      rmf_traffic::Trajectory::iterator fourth_it = trajectory.insert(
                                                                  time_3, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                                                  pos_3, vel_3)
                                                        .it;

      THEN("base_iterators assigned prior are still valid")
      {
        CHECK(first_it->get_finish_time() == time);
        CHECK(second_it->get_finish_time() == time + 10s);
        CHECK(third_it->get_finish_time() == time + 20s);
        CHECK(fourth_it->get_finish_time() == time - 30s);

        CHECK(fourth_it == trajectory.begin());
        CHECK(++fourth_it == first_it);
        CHECK(++first_it == second_it);
        CHECK(++second_it == third_it);
        CHECK(++third_it == trajectory.end());
      }
    }

    WHEN("Interpolating segment to trajectory")
    {
      rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
      rmf_traffic::Trajectory::iterator first_it = trajectory.begin();
      rmf_traffic::Trajectory::iterator second_it = trajectory.find(time + 10s);
      rmf_traffic::Trajectory::iterator third_it = trajectory.find(time + 20s);
      rmf_traffic::Time time_3 = time + 15s;
      Eigen::Vector3d pos_3 = Eigen::Vector3d(6, 6, 6);
      Eigen::Vector3d vel_3 = Eigen::Vector3d(7, 7, 7);
      rmf_traffic::Trajectory::iterator fourth_it = trajectory.insert(
                                                                  time_3, create_test_profile(UnitBox, rmf_traffic::Trajectory::Profile::Agency::Strict),
                                                                  pos_3, vel_3)
                                                        .it;

      THEN("base_iterators assigned prior are still valid")
      {
        CHECK(first_it->get_finish_time() == time);
        CHECK(second_it->get_finish_time() == time + 10s);
        CHECK(fourth_it->get_finish_time() == time + 15s);
        CHECK(third_it->get_finish_time() == time + 20s);

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
    rmf_traffic::Time time = std::chrono::steady_clock::now();
    param_inputs.push_back({time, UnitBox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)});
    param_inputs.push_back({time + 10s, UnitBox, Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(3, 3, 3)});
    param_inputs.push_back({time + 20s, UnitBox, Eigen::Vector3d(4, 4, 4), Eigen::Vector3d(5, 5, 5)});
    rmf_traffic::Trajectory trajectory = create_test_trajectory(param_inputs);
    rmf_traffic::Trajectory empty_trajectory = create_test_trajectory();

    WHEN("Setting a new map name using set_map_name function")
    {
      THEN("Name is changed successfully.")
      {
        CHECK(trajectory.get_map_name() == "test_map");
        trajectory.set_map_name(std::string("new_name"));
        CHECK(trajectory.get_map_name() == "new_name");
      }
    }

    WHEN("Finding a segment at the precise time specified")
    {
      THEN("Segment is retreieved successfully")
      {
        CHECK(trajectory.find(time)->get_finish_position() == Eigen::Vector3d(0, 0, 0));
        CHECK(trajectory.find(time + 10s)->get_finish_position() == Eigen::Vector3d(2, 2, 2));
        CHECK(trajectory.find(time + 20s)->get_finish_position() == Eigen::Vector3d(4, 4, 4));
      }
    }

    WHEN("Finding a segment at an offset time")
    {
      THEN("Segments currently active are retrieved successfully")
      {
        CHECK(trajectory.find(time)->get_finish_position() == Eigen::Vector3d(0, 0, 0));
        CHECK(trajectory.find(time + 2s)->get_finish_position() == Eigen::Vector3d(2, 2, 2));
        CHECK(trajectory.find(time + 8s)->get_finish_position() == Eigen::Vector3d(2, 2, 2));
        CHECK(trajectory.find(time + 12s)->get_finish_position() == Eigen::Vector3d(4, 4, 4));
        CHECK(trajectory.find(time + 20s)->get_finish_position() == Eigen::Vector3d(4, 4, 4));
      }
    }

    WHEN("Finding a segment at an out of bounds time")
    {
      THEN("rmf_traffic::Trajectory::end() is returned")
      {
        CHECK(trajectory.find(time - 50s) == trajectory.end());
        CHECK(trajectory.find(time + 50s) == trajectory.end());
      }
    }

    WHEN("Erasing a first segment")
    {
      THEN("Segment is erased and trajectory is rearranged")
      {
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_target = trajectory.begin();
        rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_target);
        CHECK(next_it->get_finish_time() == time + 10s);
        CHECK(trajectory.size() == 2);
      }
    }

    WHEN("Erasing a first segment from a copy")
    {
      THEN("Segment is erased and only copy is updated, source is unaffected")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_target = trajectory_copy.begin();
        rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_target);
        CHECK(next_it->get_finish_time() == time + 10s);
        CHECK(trajectory_copy.size() == 2);
        CHECK(trajectory.size() == 3);
      }
    }

    WHEN("Erasing a second segment")
    {
      THEN("Segment is erased and trajectory is rearranged")
      {
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_target = ++(trajectory.begin());
        rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_target);
        CHECK(next_it->get_finish_time() == time + 20s);
        CHECK(trajectory.size() == 2);
      }
    }

    WHEN("Erasing a second segment from a copy")
    {
      THEN("Segment is erased and only copy is updated, source is unaffected")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_target = ++(trajectory_copy.begin());
        rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_target);
        CHECK(next_it->get_finish_time() == time + 20s);
        CHECK(trajectory_copy.size() == 2);
        CHECK(trajectory.size() == 3);
      }
    }

    WHEN("Erasing a empty range of segments using range notation")
    {
      THEN("Nothing is erased and current iterator is returned")
      {
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        rmf_traffic::Trajectory::iterator erase_last = erase_first;
        rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 3);
        CHECK(next_it->get_finish_time() == time);
      }
    }

    WHEN("Erasing a empty range of segments from a copy using range notation")
    {
      THEN("Nothing is erased")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        rmf_traffic::Trajectory::iterator erase_last = erase_first;
        rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        CHECK(next_it->get_finish_time() == time);
      }
    }

    WHEN("Erasing the first segment using range notation")
    {
      THEN("1 Segment is erased and trajectory is rearranged")
      {
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        rmf_traffic::Trajectory::iterator erase_last = trajectory.find(time + 10s);
        rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 2);
        CHECK(next_it->get_finish_time() == time + 10s);
      }
    }

    WHEN("Erasing the first segment of a copy using range notation")
    {
      THEN("1 Segment is erased and trajectory is rearranged")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        rmf_traffic::Trajectory::iterator erase_last = trajectory.find(time + 10s);
        rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 2);
        CHECK(next_it->get_finish_time() == time + 10s);
      }
    }

    WHEN("Erasing the first and second segments using range notation")
    {
      THEN("2 Segments are erased and trajectory is rearranged")
      {
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        rmf_traffic::Trajectory::iterator erase_last = trajectory.find(time + 20s);
        rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 1);
        CHECK(next_it->get_finish_time() == time + 20s);
      }
    }

    WHEN("Erasing the first and second segments of a copy using range notation")
    {
      THEN("2 Segments are erased and trajectory is rearranged")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        rmf_traffic::Trajectory::iterator erase_last = trajectory.find(time + 20s);
        rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 1);
        CHECK(next_it->get_finish_time() == time + 20s);
      }
    }

    WHEN("Erasing all segments using range notation")
    {
      THEN("All Segments are erased and trajectory is empty")
      {
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        rmf_traffic::Trajectory::iterator erase_last = trajectory.end();
        rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 0);
        CHECK(next_it == trajectory.end());
      }
    }

    WHEN("Erasing all but last segment using range notation")
    {
      THEN("All but one Segment is erased")
      {
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory.begin();
        rmf_traffic::Trajectory::iterator erase_last = --(trajectory.end());
        rmf_traffic::Trajectory::iterator next_it = trajectory.erase(erase_first, erase_last);
        CHECK(trajectory.size() == 1);
        CHECK(next_it == trajectory.begin());
        CHECK(next_it == --trajectory.end());
      }
    }

    WHEN("Erasing all segments of a copy using range notation")
    {
      THEN("All Segments are erased and trajectory is empty")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory_copy.begin();
        rmf_traffic::Trajectory::iterator erase_last = trajectory_copy.end();
        rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
        CHECK(trajectory_copy.size() == 0);
        CHECK(next_it == trajectory_copy.end());
      }
    }

    WHEN("Erasing all but last segment of a copy using range notation")
    {
      THEN("All but one Segment is erased")
      {
        rmf_traffic::Trajectory trajectory_copy = trajectory;
        CHECK(trajectory_copy.size() == 3);
        CHECK(trajectory.size() == 3);
        rmf_traffic::Trajectory::iterator erase_first = trajectory_copy.begin();
        rmf_traffic::Trajectory::iterator erase_last = --(trajectory_copy.end());
        rmf_traffic::Trajectory::iterator next_it = trajectory_copy.erase(erase_first, erase_last);
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
