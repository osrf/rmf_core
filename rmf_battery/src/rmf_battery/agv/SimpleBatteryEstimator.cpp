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

#include<rmf_battery/agv/SimpleBatteryEstimator.hpp>
#include<rmf_battery/agv/SystemTraits.hpp>

#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/Time.hpp>

#include <vector>
#include <cmath>

#include <iostream>

namespace rmf_battery {
namespace agv {

class SimpleBatteryEstimator::Implementation
{
public:
  SystemTraits system_traits;
};

SimpleBatteryEstimator::SimpleBatteryEstimator(
  SystemTraits& system_traits)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation{std::move(system_traits)}))
{
  // Do nothing
}

auto SimpleBatteryEstimator::system_traits(const SystemTraits system_traits)
-> SimpleBatteryEstimator&
{
  _pimpl->system_traits = std::move(system_traits);
  return *this;
}

const SystemTraits SimpleBatteryEstimator::system_traits() const
{
  return _pimpl->system_traits;
}

namespace {
double compute_kinetic_energy(
  const double m,
  const double v,
  const double i,
  const double w)
{
  return  0.5 * (m*pow(v, 2) + i*pow(w, 2));
}

double compute_friction_energy(
  const double f,
  const double m,
  const double v,
  const double dt)
{
  const double g = 9.81; // ms-1
  return f * m * g * v * dt;
}

} // namespace anonymous
double SimpleBatteryEstimator::compute_state_of_charge(
  const rmf_traffic::Trajectory& trajectory,
  const double initial_soc,
  rmf_utils::optional<PowerMap> power_map) const
{
  assert(_pimpl->system_traits.valid());
  std::vector<double> trajectory_soc;
  trajectory_soc.reserve(trajectory.size());
  trajectory_soc.push_back(initial_soc);

  double battery_soc = initial_soc;
  double nominal_capacity = 
    _pimpl->system_traits.battery_system().nominal_capacity();
  double nominal_voltage = _pimpl->system_traits.battery_system().nominal_voltage();
  const double mass = _pimpl->system_traits.mechanical_system().mass();
  const double inertia = _pimpl->system_traits.mechanical_system().inertia();
  const double friction =
    _pimpl->system_traits.mechanical_system().friction_coefficient();
  const auto& power_systems = _pimpl->system_traits.power_systems();

  auto begin_it = trajectory.begin();
  auto end_it = --trajectory.end();

  auto start_time = begin_it->time();
  const auto end_time = end_it->time();
  const auto motion = rmf_traffic::Motion::compute_cubic_splines(
      begin_it, trajectory.end());
  
  // const Eigen::Vector3d velocity = motion->compute_velocity(start_time);
  // double v = sqrt(pow(velocity[0], 2) + pow(velocity[1], 2));
  // double w = velocity[2];

  const double sim_step = 0.1; // seconds
  // start_time = rmf_traffic::time::apply_offset(start_time, sim_step);

  double dE = 0.0;

  for (auto sim_time = start_time;
    sim_time <= end_time;
    sim_time = rmf_traffic::time::apply_offset(sim_time, sim_step))
  {
    const Eigen::Vector3d velocity = motion->compute_velocity(sim_time);
    const double v = sqrt(pow(velocity[0], 2) + pow(velocity[1], 2));
    const double w = velocity[2];
    
    const Eigen::Vector3d acceleration = motion->compute_acceleration(sim_time);
    const double a = sqrt(pow(acceleration[0], 2) + pow(acceleration[1], 2));
    const double alpha = acceleration[2];

    // Loss through acceleration
    const double EA = ((mass * a * v) + (inertia * alpha * w)) * sim_step;
    // std::cout << "EA: " << EA << " alpha: " << alpha << " w: " << w << " v: " << v << std::endl;
    // Loss through friction
    const double EF = compute_friction_energy(friction, mass, v, sim_step);

    double EP = 0.0;
    // Loss through power systems
    if (power_map.has_value())
    {
      for (const auto& item : power_map.value())
      {
        if (item.second.find(sim_time) != item.second.end())
        {
          const auto it = power_systems.find(item.first);
          assert(it != power_systems.end());
          EP += it->second.nominal_power() * sim_step;
        }
      }
    }    
    dE += EA + EF + EP;
  }

  // Computing the charge consumed
  const double dQ = dE / nominal_voltage;
  battery_soc -= dQ / (nominal_capacity * 3600);
  trajectory_soc.push_back(battery_soc);

  return trajectory_soc.back();
}

} // namespace agv
} // namespace rmf_battery
