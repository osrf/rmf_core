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

#include <rmf_traffic/agv/VehicleTraits.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class VehicleTraits::Limits::Implementation
{
public:

  double velocity;

  double acceleration;

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
    Differential differential)
  : _linear(std::move(linear)),
    _rotation(std::move(rotation)),
    _profile(std::move(profile)),
    _steering_mode(Steering::Differential),
    _differential(differential)
  {
    // Do nothing
  }
};

//==============================================================================
VehicleTraits::Limits::Limits(const double velocity, const double acceleration)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{velocity, acceleration}))
{
  // Do nothing
}

//==============================================================================
auto VehicleTraits::Limits::set_nominal_velocity(double nom_vel) -> Limits&
{
  _pimpl->velocity = nom_vel;
  return *this;
}

//==============================================================================
double VehicleTraits::Limits::get_nominal_velocity() const
{
  return _pimpl->velocity;
}

//==============================================================================
auto VehicleTraits::Limits::set_nominal_acceleration(double nom_accel)
-> Limits&
{
  _pimpl->acceleration = nom_accel;
  return *this;
}

//==============================================================================
double VehicleTraits::Limits::get_nominal_acceleration() const
{
  return _pimpl->acceleration;
}

//==============================================================================
bool VehicleTraits::Limits::valid() const
{
  return _pimpl->velocity > 0.0 && _pimpl->acceleration > 0.0;
}

//==============================================================================
class VehicleTraits::Differential::Implementation
{
public:

  Eigen::Vector2d forward;
  bool reversible;

};

//==============================================================================
VehicleTraits::Differential::Differential(
  Eigen::Vector2d forward,
  const bool reversible)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{std::move(forward), reversible}))
{
  // Do nothing
}

//==============================================================================
auto VehicleTraits::Differential::set_forward(Eigen::Vector2d forward)
-> Differential&
{
  _pimpl->forward = std::move(forward);
  return *this;
}

//==============================================================================
const Eigen::Vector2d& VehicleTraits::Differential::get_forward() const
{
  return _pimpl->forward;
}

//==============================================================================
auto VehicleTraits::Differential::set_reversible(bool reversible)
-> Differential&
{
  _pimpl->reversible = reversible;
  return *this;
}

//==============================================================================
bool VehicleTraits::Differential::is_reversible() const
{
  return _pimpl->reversible;
}

//==============================================================================
bool VehicleTraits::Differential::valid() const
{
  return _pimpl->forward.norm() > 1e-6;
}

//==============================================================================
VehicleTraits::Holonomic::Holonomic()
{
  // Do nothing. No need to instantiate _pimpl because it's not being used (yet)
}

//==============================================================================
VehicleTraits::VehicleTraits(
  Limits linear,
  Limits rotational,
  Profile profile,
  Differential steering)
: _pimpl(rmf_utils::make_impl<Implementation>(
      std::move(linear),
      std::move(rotational),
      std::move(profile),
      std::move(steering)))
{
  // Do nothing
}

//==============================================================================
VehicleTraits::Limits& VehicleTraits::linear()
{
  return _pimpl->_linear;
}

//==============================================================================
const VehicleTraits::Limits& VehicleTraits::linear() const
{
  return _pimpl->_linear;
}

//==============================================================================
VehicleTraits::Limits& VehicleTraits::rotational()
{
  return _pimpl->_rotation;
}

//==============================================================================
const VehicleTraits::Limits& VehicleTraits::rotational() const
{
  return _pimpl->_rotation;
}

//==============================================================================
Profile& VehicleTraits::profile()
{
  return _pimpl->_profile;
}

//==============================================================================
const Profile& VehicleTraits::profile() const
{
  return _pimpl->_profile;
}

//==============================================================================
VehicleTraits::Steering VehicleTraits::get_steering() const
{
  return _pimpl->_steering_mode;
}

//==============================================================================
auto VehicleTraits::set_differential(Differential parameters) -> Differential&
{
  _pimpl->_steering_mode = Steering::Differential;
  _pimpl->_differential = std::move(parameters);
  return _pimpl->_differential;
}

//==============================================================================
auto VehicleTraits::get_differential() -> Differential*
{
  if (_pimpl->_steering_mode == Steering::Differential)
    return &_pimpl->_differential;

  return nullptr;
}

//==============================================================================
auto VehicleTraits::get_differential() const -> const Differential*
{
  if (_pimpl->_steering_mode == Steering::Differential)
    return &_pimpl->_differential;

  return nullptr;
}

//==============================================================================
auto VehicleTraits::set_holonomic(Holonomic parameters) -> Holonomic&
{
  _pimpl->_steering_mode = Steering::Holonomic;
  _pimpl->_holonomic = std::move(parameters);
  return _pimpl->_holonomic;
}

//==============================================================================
auto VehicleTraits::get_holonomic() -> Holonomic*
{
  if (_pimpl->_steering_mode == Steering::Holonomic)
    return &_pimpl->_holonomic;

  return nullptr;
}

//==============================================================================
auto VehicleTraits::get_holonomic() const -> const Holonomic*
{
  if (_pimpl->_steering_mode == Steering::Holonomic)
    return &_pimpl->_holonomic;

  return nullptr;
}

//==============================================================================
bool VehicleTraits::valid() const
{
  const bool steering_valid =
    [&]() -> bool
    {
      if (_pimpl->_steering_mode == Steering::Differential)
        return get_differential()->valid();

      return true;
    } ();

  return linear().valid() && rotational().valid() && steering_valid;
}

} // namespace agv
} // namespace rmf_traffic
