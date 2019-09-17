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

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/geometry/Box.hpp>

#include <rmf_utils/catch.hpp>

rmf_traffic::Trajectory::ProfilePtr make_test_profile()
{
  return rmf_traffic::Trajectory::Profile::make_strict(
        std::make_shared<rmf_traffic::geometry::Box>(1.0, 1.0));
}

TEST_CASE("Construct a Trajectory")
{
  using namespace std::chrono_literals;

  rmf_traffic::Trajectory trajectory{"test_map"};
  CHECK(trajectory.begin() == trajectory.end());
  CHECK(trajectory.end() == trajectory.end());

  const auto profile = make_test_profile();

  const auto begin_time = std::chrono::steady_clock::now();
  const Eigen::Vector3d begin_p = Eigen::Vector3d(0, 0, 0);
  const Eigen::Vector3d begin_v = Eigen::Vector3d(0, 0, 0);

  auto result = trajectory.insert(
        begin_time, profile, begin_p, begin_v);

  const rmf_traffic::Trajectory::iterator begin_it = result.it;
  CHECK(result.inserted);

  CHECK(begin_it == trajectory.begin());
  CHECK(trajectory.begin() != trajectory.end());
  CHECK(begin_it != trajectory.end());
  CHECK(begin_it < trajectory.end());
  CHECK(begin_it <= trajectory.end());
  CHECK(trajectory.end() > begin_it);
  CHECK(trajectory.end() >= trajectory.end());

  CHECK(begin_p == begin_it->get_position());
  CHECK(begin_v == begin_it->get_velocity());
  CHECK(begin_time == begin_it->get_finish_time());

  const auto second_time = begin_time + 10s;
  const Eigen::Vector3d second_p = Eigen::Vector3d(1, 2, 3);
  const Eigen::Vector3d second_v = Eigen::Vector3d(3, 2, 1);

  result = trajectory.insert(
        second_time, profile, second_p, second_v);

  const rmf_traffic::Trajectory::iterator second_it = result.it;
  CHECK(result.inserted);

  CHECK(second_it == ++trajectory.begin());
  CHECK(second_it != trajectory.begin());
  CHECK(second_it > trajectory.begin());
  CHECK(second_it >= trajectory.begin());
  CHECK(trajectory.begin() < second_it);
  CHECK(trajectory.begin() <= second_it);

  CHECK(second_it != begin_it);
  CHECK(second_it > begin_it);
  CHECK(second_it >= begin_it);
  CHECK(begin_it < second_it);
  CHECK(begin_it <= second_it);

  CHECK(second_it != trajectory.end());
  CHECK(second_it < trajectory.end());
  CHECK(second_it <= trajectory.end());
  CHECK(trajectory.end() > second_it);
  CHECK(trajectory.end() >= second_it);

  CHECK(second_it->get_position() == second_p);
  CHECK(second_it->get_velocity() == second_v);
  CHECK(second_it->get_finish_time() == second_time);
}

TEST_CASE("Copy and move a trajectory")
{
  using namespace std::chrono_literals;

  rmf_traffic::Trajectory trajectory{"test_map"};

  const auto begin_time = std::chrono::steady_clock::now();

  trajectory.insert(
        begin_time, make_test_profile(),
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitX());

  trajectory.insert(
        begin_time + 10s, make_test_profile(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY());

  trajectory.insert(
        begin_time + 15s, make_test_profile(),
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ());

  rmf_traffic::Trajectory copy = trajectory;

  rmf_traffic::Trajectory::const_iterator ot = trajectory.begin();
  rmf_traffic::Trajectory::const_iterator ct = copy.begin();
  for( ; ot != trajectory.end() && ct != trajectory.end(); ++ot, ++ct)
  {
    CHECK(ot->get_profile() == ct->get_profile());
    CHECK(ot->get_position() == ct->get_position());
    CHECK(ot->get_velocity() == ct->get_velocity());
    CHECK(ot->get_finish_time() == ct->get_finish_time());
  }
  CHECK(ot == trajectory.end());
  CHECK(ct == copy.end());

  for(auto it = copy.begin(); it != copy.end(); ++it)
  {
    it->set_profile(make_test_profile());
    it->set_position(it->get_position() + Eigen::Vector3d::UnitZ());
    it->set_velocity(it->get_velocity() + Eigen::Vector3d::UnitZ());
    it->set_finish_time(it->get_finish_time() + 2s);
  }

  ot = trajectory.begin();
  ct = copy.begin();
  for( ; ot != trajectory.end() && ct != trajectory.end(); ++ot, ++ct)
  {
    CHECK(ot->get_profile() != ct->get_profile());
    CHECK(ot->get_position() != ct->get_position());
    CHECK(ot->get_velocity() != ct->get_velocity());
    CHECK(ot->get_finish_time() != ct->get_finish_time());
  }
  CHECK(ot == trajectory.end());
  CHECK(ct == copy.end());

  // Copy again
  copy = trajectory;

  // Now move the original
  rmf_traffic::Trajectory moved = std::move(trajectory);

  ct = copy.begin();
  rmf_traffic::Trajectory::const_iterator mt = moved.begin();
  for( ; ct != copy.end() && mt != moved.end(); ++ct, ++mt)
  {
    CHECK(ct->get_profile() == mt->get_profile());
    CHECK(ct->get_position() == mt->get_position());
    CHECK(ct->get_velocity() == mt->get_velocity());
    CHECK(ct->get_finish_time() == mt->get_finish_time());
  }
  CHECK(ct == copy.end());
  CHECK(mt == moved.end());
}
