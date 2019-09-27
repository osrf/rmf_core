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

#include <rmf_utils/catch.hpp>

#include <rmf_traffic/agv/Interpolate.hpp>

#include <iostream>

struct PredictedTimes
{
  /// time (relative to the start of the trajectory) that the vehicle started at rest
  double t_start;

  /// time (relative to t_start) that the vehicle finished accelerating
  double t_a;

  /// time (relative to t_start) that the vehicle began to decelerate
  double t_d;

  /// time (relative to the start of the trajectory) that the vehicle came to rest
  double t_finish;
};

void test_interpolation(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& dir,
    const double a_n, // nominal acceleration
    const PredictedTimes& times,
    const rmf_traffic::Trajectory& trajectory)
{
  using namespace std::chrono_literals;

  const double t_start = times.t_start;
  const double t_a = times.t_a;
  const double t_d = times.t_d;
  const double t_finish = times.t_finish;

  std::cout << "times: ";
  for(const double t : {t_start, t_a, t_d, t_finish})
    std::cout << t << "  ";
  std::cout << std::endl;

  REQUIRE(trajectory.size() > 0);
  const double dt = t_finish - t_start;
  REQUIRE(dt > 0.0);
  const rmf_traffic::Time trajectory_start_time = *trajectory.start_time();
  const rmf_traffic::Time start_time =
      rmf_traffic::time::apply_offset(trajectory_start_time, t_start);

  const rmf_traffic::Time finish_time =
      rmf_traffic::time::apply_offset(trajectory_start_time, t_finish);

  const Eigen::Vector3d normalized_dir = dir.normalized();

  std::cout << t_start << " | " << t_a << " | " << t_d << " | " << t_finish << std::endl;

  std::size_t count = 0;
  for(auto it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    std::cout << "[" << count
              << "] t " << rmf_traffic::time::to_seconds(it->get_finish_time() - trajectory_start_time)
              << " | p " << it->get_finish_position().transpose()
              << " | v " << it->get_finish_velocity().transpose()
              << std::endl;
  }

  const std::size_t NumTests = 50;
  for(std::size_t i=1; i < NumTests; ++i)
  {
    const double u = static_cast<double>(i)/static_cast<double>(NumTests);
    const double t_rel = u*dt;
    const auto t = rmf_traffic::time::apply_offset(
          trajectory_start_time, t_rel + t_start);

    const auto it = trajectory.find(t);
    REQUIRE(it != trajectory.end());

    const auto motion = it->compute_motion();
    REQUIRE(motion != nullptr);
    CHECK(start_time - 10ns <= motion->start_time());
    std::cout  << "start_time: expected "
               << rmf_traffic::time::to_seconds(start_time - trajectory_start_time)
               << " | actual " << rmf_traffic::time::to_seconds(motion->start_time() - trajectory_start_time)
               << " | difference " << rmf_traffic::time::to_seconds(motion->start_time() - start_time) << std::endl;

    CHECK(motion->finish_time() <= finish_time + 10ns);
    std::cout << "finish_time: expected "
              << rmf_traffic::time::to_seconds(finish_time - trajectory_start_time)
              << " | actual " << rmf_traffic::time::to_seconds(motion->finish_time() - trajectory_start_time)
              << " | difference " << rmf_traffic::time::to_seconds(motion->finish_time() - finish_time) << std::endl;

    std::cout << "t: " << rmf_traffic::time::to_seconds(t - trajectory_start_time) << std::endl;

    if(t_rel <= t_a)
    {
      const double s = 0.5*a_n*pow(t_rel,2);
      const Eigen::Vector3d p_expected = p0 + s*normalized_dir;
      const Eigen::Vector3d p_actual = motion->compute_position(t);
      for(int k=0; k < 3; ++k)
        CHECK(p_actual[k] == Approx(p_expected[k]).margin(1e-8));

      const double v = a_n*t_rel;
      const Eigen::Vector3d v_expected = v*normalized_dir;
      const Eigen::Vector3d v_actual = motion->compute_velocity(t);
      for(int k=0; k < 3; ++k)
        CHECK(v_actual[k] == Approx(v_expected[k]).margin(1e-8));

      const Eigen::Vector3d a_expected = a_n*normalized_dir;
      const Eigen::Vector3d a_actual = motion->compute_acceleration(t);
      for(int k=0; k < 3; ++k)
        CHECK(a_actual[k] == Approx(a_expected[k]).margin(1e-8));
    }
    else if(t_rel <= t_d)
    {
      const double v = a_n * t_a;
      const double s = 0.5*a_n*pow(t_a, 2) + v * (t_rel - t_a);
      const Eigen::Vector3d p_expected = p0 + s*normalized_dir;
      const Eigen::Vector3d p_actual = motion->compute_position(t);
      for(int k=0; k < 3; ++k)
        CHECK(p_actual[k] == Approx(p_expected[k]).margin(1e-8));

      const Eigen::Vector3d v_expected = v*normalized_dir;
      const Eigen::Vector3d v_actual = motion->compute_velocity(t);
      for(int k=0; k < 3; ++k)
        CHECK(v_actual[k] == Approx(v_expected[k]).margin(1e-8));

      const Eigen::Vector3d a_expected = Eigen::Vector3d::Zero();
      const Eigen::Vector3d a_actual = motion->compute_acceleration(t);
      for(int k=0; k < 3; ++k)
        CHECK(a_actual[k] == Approx(a_expected[k]).margin(1e-8));
    }
    else if(t_rel <= dt)
    {
      const double v_peak = a_n*t_a;
      const double v = v_peak - a_n*(t_rel - t_d);
      const double s =
          0.5*a_n*pow(t_a, 2) + v_peak*(t_rel - t_a) - 0.5*a_n*pow(t_rel - t_d, 2);

      const Eigen::Vector3d p_expected = p0 + s*normalized_dir;
      const Eigen::Vector3d p_actual = motion->compute_position(t);
      for(int k=0; k < 3; ++k)
        CHECK(p_actual[k] == Approx(p_expected[k]).margin(1e-8));

      const Eigen::Vector3d v_expected = v*normalized_dir;
      const Eigen::Vector3d v_actual = motion->compute_velocity(t);
      for(int k=0; k < 3; ++k)
        CHECK(v_actual[k] == Approx(v_expected[k]).margin(1e-8));

      const Eigen::Vector3d a_expected = -a_n*normalized_dir;
      const Eigen::Vector3d a_actual = motion->compute_acceleration(t);
      for(int k=0; k < 3; ++k)
        CHECK(a_actual[k] == Approx(a_expected[k]).margin(1e-8));
    }
  }
}

PredictedTimes compute_predicted_times(
    const double t_start,
    const double s, // distance to travel
    const double v_n, // nominal velocity
    const double a_n // nominal acceleration
    )
{
  const double t_a = std::min(v_n/a_n, std::sqrt(s/a_n));
  const double v = a_n*t_a;
  const double t_d =
      s/v - a_n*pow(t_a, 2)/(2.0*v) - v/(2.0*a_n) + t_a;
  const double t_finish = t_d + v/a_n;

  return PredictedTimes{t_start, t_a, t_d, t_finish + t_start};
}

SCENARIO("Test Interpolations")
{
  const double v_n = 0.7;
  const double a_n = 0.5;
  const double w_n = 0.3;
  const double alpha_n = 0.25;

  rmf_traffic::agv::VehicleTraits traits;
  traits
      .linear().set_nominal_velocity(v_n)
      .linear().set_nominal_acceleration(a_n)
      .rotational().set_nominal_velocity(w_n)
      .rotational().set_nominal_acceleration(alpha_n);

  std::vector<Eigen::Vector3d> positions;
  GIVEN("Three distant points")
  {
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();
    const double xf = 10.0;
    const double angle1 = 90.0*M_PI/180.0;
    const double angle2 = 45.0*M_PI/180.0;
    positions = {
        Eigen::Vector3d{0.0, 0.0, 0.0},
        Eigen::Vector3d{xf, 0.0, angle1},
        Eigen::Vector3d{xf, -xf, angle2}
      };

    rmf_traffic::Trajectory trajectory =
        rmf_traffic::agv::Interpolate::positions(
          "test_map", traits, start_time, positions);

    THEN("The trajectory is correctly interpolated")
    {
      CHECK(trajectory.size() == 13);

      const PredictedTimes times1 = compute_predicted_times(0.0, xf, v_n, a_n);
      test_interpolation(
          positions[0], Eigen::Vector3d{1.0, 0.0, 0.0}, a_n, times1, trajectory);

      const PredictedTimes times2 =
          compute_predicted_times(times1.t_finish, angle1, w_n, alpha_n);
      test_interpolation(
          Eigen::Vector3d{xf, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 1.0},
          alpha_n, times2, trajectory);

      std::cout << " =========================== TEST 3 ========================" << std::endl;
      const PredictedTimes times3 =
          compute_predicted_times(times2.t_finish, xf, v_n, a_n);
      test_interpolation(
          positions[1], Eigen::Vector3d{0.0, -1.0, 0.0}, a_n, times3, trajectory);

      const PredictedTimes times4 =
          compute_predicted_times(
            times3.t_finish, std::abs(angle2 - angle1), w_n, alpha_n);
      test_interpolation(
          Eigen::Vector3d{xf, -xf, angle1}, Eigen::Vector3d{0.0, 0.0, -1.0},
          alpha_n, times4, trajectory);
    }
  }

}
