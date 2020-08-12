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

#include "EstimateBatteryInternal.hpp"

#include <rmf_traffic/Motion.hpp>

#include <vector>
#include <cmath>

namespace rmf_battery {

double SampleEstimator::compute_soc(
  const rmf_traffic::Trajectory& trajectory,
  const rmf_battery::agv::SystemTraits& system_traits,
  const double initial_soc) const
{
  assert(system_traits.valid());
  std::vector<double> trajectory_soc;
  trajectory_soc.reserve(trajectory.size());
  trajectory_soc.push_back(initial_soc);

  double battery_soc = initial_soc;
  double nominal_capacity = system_traits.battery_system().nominal_capacity();
  double nominal_voltage = system_traits.battery_system().nominal_voltage();
  const double mass = system_traits.mechanical_system().mass();
  const double inertia = system_traits.mechanical_system().inertia();
  const double friction =
    system_traits.mechanical_system().friction_coefficient();
  const double g = 9.81; // ms-1
  const int sim_step = 100; // milliseconds

  for (auto it = trajectory.begin(); it != trajectory.end(); it++)
  {
    auto next_it = it; ++next_it;
    if (next_it == trajectory.end())
      break;
    const auto start_time = it->time() + std::chrono::milliseconds(sim_step);
    const auto end_time = next_it->time();
    const auto motion = rmf_traffic::Motion::compute_cubic_splines(it, next_it);
    
    double dE = 0.0;

    for (auto sim_time = start_time;
      sim_time <= end_time; sim_time += std::chrono::milliseconds(sim_step))
    {
      const Eigen::Vector3d velocity = motion->compute_velocity(sim_time);
      const double v = sqrt(pow(velocity[0], 2) + pow(velocity[1], 2));
      const double w = velocity[2];
      // Kinetic energy
      const double dKE = 0.5 * (mass*pow(v, 2) + inertia*pow(w, 2));
      // Friction energy
      const double dFE = friction * mass * g * 2 * v * (sim_step/1000);
      // TODO(YV) energy from power systems
      // 
      dE += dKE + dFE;
    }

    // Charge consumed
    const double dQ = dE / nominal_voltage;
    battery_soc -= dQ / nominal_capacity;
    trajectory_soc.push_back(battery_soc);
  }

  return trajectory_soc.back();
}

} // namespace rmf_battery