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

#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>

#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/Time.hpp>

#include <vector>
#include <cmath>

#include <iostream>

namespace rmf_battery {
namespace agv {

class SimpleMotionPowerSink::Implementation
{
public:
  BatterySystem battery_system;
  MechanicalSystem mechanical_system;
};

//==============================================================================
SimpleMotionPowerSink::SimpleMotionPowerSink(
  BatterySystem& battery_system,
  MechanicalSystem& mechanical_system)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{battery_system, mechanical_system}))
{
  // Do nothing
}

//==============================================================================
const BatterySystem& SimpleMotionPowerSink::battery_system() const
{
  return _pimpl->battery_system;
}

//==============================================================================
const MechanicalSystem& SimpleMotionPowerSink::mechanical_system() const
{
  return _pimpl->mechanical_system;
}

//==============================================================================
namespace {

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

//==============================================================================
double SimpleMotionPowerSink::compute_change_in_charge(
  const rmf_traffic::Trajectory& trajectory) const
{
  assert(_pimpl->battery_system.valid());
  assert(_pimpl->mechanical_system.valid());

  const double nominal_capacity = _pimpl->battery_system.nominal_capacity();
  const double nominal_voltage = _pimpl->battery_system.nominal_voltage();
  const double mass = _pimpl->mechanical_system.mass();
  const double inertia = _pimpl->mechanical_system.inertia();
  const double friction = _pimpl->mechanical_system.friction_coefficient();

  auto begin_it = trajectory.begin();
  auto end_it = --trajectory.end();

  auto start_time = begin_it->time();
  const auto end_time = end_it->time();
  const auto motion = rmf_traffic::Motion::compute_cubic_splines(
    begin_it, trajectory.end());

  const double sim_step = 0.1; // seconds

  // Change in energy
  double dE = 0.0;

  // TODO explore analytical solutions as opposed to numerical integration
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

    dE += EA + EF;
  }

  // The charge consumed
  const double dQ = dE / nominal_voltage;
  // The depleted state of charge
  const double dSOC = dQ / (nominal_capacity * 3600.0);
  return dSOC;
}

} // namespace agv
} // namespace rmf_battery
