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

rmf_traffic::Time test_interpolation(
    const double v_n,
    const double a_n,
    const double t_a,
    const double t_d,
    const rmf_traffic::Trajectory& trajectory)
{

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
  GIVEN("Two far points")
  {
    const rmf_traffic::Time start_time = std::chrono::steady_clock::now();
    const double xf = 10.0;
    positions = {
        Eigen::Vector3d{0.0, 0.0, 0.0},
        Eigen::Vector3d{xf, 0.0, 90.0*M_PI/180.0},
        Eigen::Vector3d{xf, -xf, 45.0*M_PI/180.0}
      };

    rmf_traffic::Trajectory trajectory =
        rmf_traffic::agv::Interpolate::positions(
          "test_map", traits, start_time, positions);

    const double t_a1 = v_n/a_n;
    const double t_d1 =
        xf/v_n - a_n*pow(t_a1, 2)/(2.0*v_n) - v_n/(2.0*a_n) + t_a1;


  }

}
