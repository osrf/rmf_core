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
#include <rmf_traffic/Motion.hpp>

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

  REQUIRE(trajectory.size() > 0);
  const double dt = t_finish - t_start;
  REQUIRE(dt > 0.0);
  const rmf_traffic::Time trajectory_start_time = *trajectory.start_time();
  const rmf_traffic::Time start_time =
    rmf_traffic::time::apply_offset(trajectory_start_time, t_start);

  const rmf_traffic::Time finish_time =
    rmf_traffic::time::apply_offset(trajectory_start_time, t_finish);

  const Eigen::Vector3d normalized_dir = dir.normalized();

  const std::size_t NumTests = 50;
  for (std::size_t i = 1; i < NumTests; ++i)
  {
    const double u = static_cast<double>(i)/static_cast<double>(NumTests);
    const double t_rel = u*dt;
    const auto t = rmf_traffic::time::apply_offset(
      trajectory_start_time, t_rel + t_start);

    const auto it = trajectory.find(t);
    REQUIRE(it != trajectory.end());

    const auto motion = rmf_traffic::Motion::compute_cubic_splines(
      --rmf_traffic::Trajectory::const_iterator(it),
      ++rmf_traffic::Trajectory::const_iterator(it));

    REQUIRE(motion != nullptr);
    CHECK(start_time - 10ns <= motion->start_time());
    CHECK(motion->finish_time() <= finish_time + 10ns);

    if (t_rel <= t_a)
    {
      const double s = 0.5*a_n*pow(t_rel, 2);
      const Eigen::Vector3d p_expected = p0 + s*normalized_dir;
      const Eigen::Vector3d p_actual = motion->compute_position(t);
      for (int k = 0; k < 3; ++k)
        CHECK(p_actual[k] == Approx(p_expected[k]).margin(1e-8));

      const double v = a_n*t_rel;
      const Eigen::Vector3d v_expected = v*normalized_dir;
      const Eigen::Vector3d v_actual = motion->compute_velocity(t);
      for (int k = 0; k < 3; ++k)
        CHECK(v_actual[k] == Approx(v_expected[k]).margin(1e-8));

      if (std::abs(t_rel - t_a) > 1e-8)
      {
        // Acceleration is discontinuous at the time t_a, so if we are very
        // very close to it, then we shouldn't make assumptions about the
        // acceleration value.
        const Eigen::Vector3d a_expected = a_n*normalized_dir;
        const Eigen::Vector3d a_actual = motion->compute_acceleration(t);
        for (int k = 0; k < 3; ++k)
          CHECK(a_actual[k] == Approx(a_expected[k]).margin(1e-8));
      }
    }
    else if (t_rel <= t_d)
    {
      const double v = a_n * t_a;
      const double s = 0.5*a_n*pow(t_a, 2) + v * (t_rel - t_a);
      const Eigen::Vector3d p_expected = p0 + s*normalized_dir;
      const Eigen::Vector3d p_actual = motion->compute_position(t);
      for (int k = 0; k < 3; ++k)
        CHECK(p_actual[k] == Approx(p_expected[k]).margin(1e-8));

      const Eigen::Vector3d v_expected = v*normalized_dir;
      const Eigen::Vector3d v_actual = motion->compute_velocity(t);
      for (int k = 0; k < 3; ++k)
        CHECK(v_actual[k] == Approx(v_expected[k]).margin(1e-8));

      if (std::abs(t_rel - t_d) > 1e-8)
      {
        // Acceleration is discontinuous at the time t_d, so if we are very
        // very close to it, then we shouldn't make assumptions about the
        // acceleration value.
        const Eigen::Vector3d a_expected = Eigen::Vector3d::Zero();
        const Eigen::Vector3d a_actual = motion->compute_acceleration(t);
        for (int k = 0; k < 3; ++k)
          CHECK(a_actual[k] == Approx(a_expected[k]).margin(1e-8));
      }
    }
    else if (t_rel <= dt)
    {
      const double v_peak = a_n*t_a;
      const double v = v_peak - a_n*(t_rel - t_d);
      const double s =
        0.5*a_n*
        pow(t_a, 2) + v_peak*(t_rel - t_a) - 0.5*a_n*pow(t_rel - t_d, 2);

      const Eigen::Vector3d p_expected = p0 + s*normalized_dir;
      const Eigen::Vector3d p_actual = motion->compute_position(t);
      for (int k = 0; k < 3; ++k)
        CHECK(p_actual[k] == Approx(p_expected[k]).margin(1e-8));

      const Eigen::Vector3d v_expected = v*normalized_dir;
      const Eigen::Vector3d v_actual = motion->compute_velocity(t);
      for (int k = 0; k < 3; ++k)
        CHECK(v_actual[k] == Approx(v_expected[k]).margin(1e-8));

      const Eigen::Vector3d a_expected = -a_n*normalized_dir;
      const Eigen::Vector3d a_actual = motion->compute_acceleration(t);
      for (int k = 0; k < 3; ++k)
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

  const rmf_traffic::agv::VehicleTraits traits(
    {v_n, a_n}, {w_n, alpha_n}, {nullptr, nullptr});

  GIVEN("Three distant points")
  {
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();
    const double xf = 10.0;
    const double angle1 = 90.0*M_PI/180.0;
    const double angle2 = 45.0*M_PI/180.0;
    const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{xf, 0.0, angle1},
      Eigen::Vector3d{xf, -xf, angle2}
    };

    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    THEN("The trajectory is correctly interpolated")
    {
      CHECK(trajectory.size() == 13);

      const PredictedTimes times1 = compute_predicted_times(0.0, xf, v_n, a_n);
      test_interpolation(
        positions[0], Eigen::Vector3d::UnitX(), a_n, times1, trajectory);

      const PredictedTimes times2 =
        compute_predicted_times(times1.t_finish, angle1, w_n, alpha_n);
      test_interpolation(
        Eigen::Vector3d{xf, 0.0, 0.0}, Eigen::Vector3d::UnitZ(),
        alpha_n, times2, trajectory);

      const PredictedTimes times3 =
        compute_predicted_times(times2.t_finish, xf, v_n, a_n);
      test_interpolation(
        positions[1], -Eigen::Vector3d::UnitY(), a_n, times3, trajectory);

      const PredictedTimes times4 =
        compute_predicted_times(
        times3.t_finish, std::abs(angle2 - angle1), w_n, alpha_n);
      test_interpolation(
        Eigen::Vector3d{xf, -xf, angle1}, -Eigen::Vector3d::UnitZ(),
        alpha_n, times4, trajectory);
    }
  }

  GIVEN("Three near points")
  {
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();
    const double x0 = 10.0;
    const double y0 = -15.0;
    const double theta0 = -60.0*M_PI/180.0;

    const double pf = 0.1;
    const double angle1 = 2.0*M_PI/180.0;
    const double angle2 = -1.0*M_PI/180.0;
    const std::vector<Eigen::Vector3d> positions = {
      {x0, y0, theta0},
      {x0 - pf, -15.0, theta0 + angle1},
      {x0 - pf, -15 + pf, theta0 + angle2}
    };

    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    THEN("The trajectory is correctly interpolated")
    {
      CHECK(trajectory.size() == 9);

      const PredictedTimes times1 = compute_predicted_times(0.0, pf, v_n, a_n);
      test_interpolation(
        positions[0], -Eigen::Vector3d::UnitX(), a_n, times1, trajectory);

      const PredictedTimes times2 = compute_predicted_times(
        times1.t_finish, angle1, w_n, alpha_n);
      test_interpolation(
        Eigen::Vector3d{x0 - pf, -15.0, theta0}, Eigen::Vector3d::UnitZ(),
        alpha_n, times2, trajectory);

      const PredictedTimes times3 = compute_predicted_times(
        times2.t_finish, pf, v_n, a_n);
      test_interpolation(
        positions[1], Eigen::Vector3d::UnitY(), a_n, times3, trajectory);

      const PredictedTimes times4 = compute_predicted_times(
        times3.t_finish, std::abs(angle2 - angle1), w_n, alpha_n);
      test_interpolation(
        Eigen::Vector3d{x0 - pf, -15 + pf, theta0 + angle1},
        -Eigen::Vector3d::UnitZ(), alpha_n, times4, trajectory);
    }
  }

  GIVEN("Six distant waypoints with no rotation")
  {
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();

    const std::vector<Eigen::Vector3d> positions = {
      {10, 12, 0}, {5, 8, 0}, {0, 8, 0}, {0, 0, 0}, {0, -5, 0}, {5, -5, 0}
    };

    rmf_traffic::Trajectory trajectory =
      rmf_traffic::agv::Interpolate::positions(traits, start_time, positions);

    // NOTE(MXG): This test is here because a bug was found when orientations
    // were not getting changed. It's a simple test, but please do not delete
    // it. If anything, it should be expanded upon.
    CHECK(rmf_traffic::time::to_seconds(trajectory.duration()) > 0.0);
  }
}

/// ================ TODO(MXG): Testing wishlist ================
/// * Test the different Interpolate::Options features:
///   - always stop
///   - translation threshold
///   - rotation threshold
///   - corner angle threshold
/// * Test a trajectory that moves diagonally through the x/y plane
/// * Compute some points along an expected trajectory by hand, and compare the
///   rmf_traffic::avg::Interpolate results against the hand-computed expectations
/// * A regression test for pull request #15: https://github.com/osrf/rmf_core/pull/15
///
