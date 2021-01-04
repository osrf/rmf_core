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

#ifndef SRC__RMF_TRAFFIC__AGV__INTERNAL_VEHICLETRAITS_HPP
#define SRC__RMF_TRAFFIC__AGV__INTERNAL_VEHICLETRAITS_HPP

#include <rmf_traffic/agv/VehicleTraits.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class VehicleTraits::Limits::Implementation
{
public:

  double velocity;
  double acceleration;

  static const Implementation& get(const Limits& limits);
};

//==============================================================================
struct KinematicLimits
{
  VehicleTraits::Limits::Implementation linear;
  VehicleTraits::Limits::Implementation angular;
};

//==============================================================================
class VehicleTraits::Implementation
{
public:

  Limits _linear;
  Limits _rotation;
  Profile _profile;

  Steering _steering_mode;
  Differential _differential;
  Holonomic _holonomic;

  Implementation(
    Limits linear,
    Limits rotation,
    Profile profile,
    Differential differential);

  static KinematicLimits get_limits(const VehicleTraits& traits);
};

} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__INTERNAL_VEHICLETRAITS_HPP
