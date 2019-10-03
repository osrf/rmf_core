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
class VehicleTraits::Implementation
{
public:

  struct LimitInfo
  {
    LimitInfo(
        VehicleTraits* parent,
        const double nom_vel,
        const double nom_accel)
      : parent(parent),
        nominal_velocity(nom_vel),
        nominal_acceleration(nom_accel)
    {
      // Do nothing
    }

    VehicleTraits* parent;
    double nominal_velocity;
    double nominal_acceleration;
  };

  LimitInfo _linear_info;
  Limits _linear;

  LimitInfo _rotation_info;
  Limits _rotation;

  Steering _steering;

  bool _reversible;

  Implementation(
      VehicleTraits* const parent,
      const double nom_linear_vel,
      const double nom_linear_accel,
      const double nom_rotation_vel,
      const double nom_rotation_accel,
      const Steering steering,
      const bool reversible)
    : _linear_info(parent, nom_linear_vel, nom_linear_accel),
      _linear(&_linear_info),
      _rotation_info(parent, nom_rotation_vel, nom_rotation_accel),
      _rotation(&_rotation_info),
      _steering(steering),
      _reversible(reversible)
  {
    // Do nothing
  }

  void set_parent(VehicleTraits* parent)
  {
    _linear_info.parent = parent;
    _rotation_info.parent = parent;

    _linear._pimpl = &_linear_info;
    _rotation._pimpl = &_rotation_info;
  }
};

//==============================================================================
VehicleTraits::Limits::Limits(void* pimpl)
  : _pimpl(pimpl)
{
  // Do nothing
}

//==============================================================================
VehicleTraits& VehicleTraits::Limits::set_nominal_velocity(double nom_vel)
{
  auto& info = *static_cast<Implementation::LimitInfo*>(_pimpl);
  info.nominal_velocity = nom_vel;
  return *info.parent;
}

//==============================================================================
double VehicleTraits::Limits::get_nominal_velocity() const
{
  const auto& info = *static_cast<const Implementation::LimitInfo*>(_pimpl);
  return info.nominal_velocity;
}

//==============================================================================
VehicleTraits& VehicleTraits::Limits::set_nominal_acceleration(double nom_accel)
{
  auto& info = *static_cast<Implementation::LimitInfo*>(_pimpl);
  info.nominal_acceleration = nom_accel;
  return *info.parent;
}

//==============================================================================
double VehicleTraits::Limits::get_nominal_acceleration() const
{
  const auto& info = *static_cast<const Implementation::LimitInfo*>(_pimpl);
  return info.nominal_acceleration;
}

//==============================================================================
bool VehicleTraits::Limits::valid() const
{
  const auto& info = *static_cast<const Implementation::LimitInfo*>(_pimpl);
  return info.nominal_velocity > 0.0 && info.nominal_acceleration > 0.0;
}

//==============================================================================
VehicleTraits::VehicleTraits(
    const double nom_linear_vel,
    const double nom_linear_accel,
    const double nom_rotation_vel,
    const double nom_rotation_accel,
    const Steering steering,
    const bool reversible)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(
             this, nom_linear_vel, nom_linear_accel,
             nom_rotation_vel, nom_rotation_accel, steering, reversible))
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
VehicleTraits& VehicleTraits::set_steering(Steering steering)
{
  _pimpl->_steering = steering;
  return *this;
}

//==============================================================================
VehicleTraits::Steering VehicleTraits::get_steering() const
{
  return _pimpl->_steering;
}

//==============================================================================
VehicleTraits& VehicleTraits::set_reversible(bool reversible)
{
  _pimpl->_reversible = reversible;
  return *this;
}

//==============================================================================
bool VehicleTraits::is_reversible() const
{
  return _pimpl->_reversible;
}

//==============================================================================
bool VehicleTraits::valid() const
{
  return linear().valid() && rotational().valid();
}

//==============================================================================
VehicleTraits::VehicleTraits(const VehicleTraits& other)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(*other._pimpl))
{
  _pimpl->set_parent(this);
}

//==============================================================================
VehicleTraits::VehicleTraits(VehicleTraits&& other)
  : _pimpl(std::move(other._pimpl))
{
  _pimpl->set_parent(this);
}

//==============================================================================
VehicleTraits& VehicleTraits::operator=(const VehicleTraits& other)
{
  if(_pimpl)
    *_pimpl = *other._pimpl;
  else
    _pimpl = rmf_utils::make_unique_impl<Implementation>(*other._pimpl);

  _pimpl->set_parent(this);
  return *this;
}

//==============================================================================
VehicleTraits& VehicleTraits::operator=(VehicleTraits&& other)
{
  _pimpl = std::move(other._pimpl);
  _pimpl->set_parent(this);
  return *this;
}

} // namespace agv
} // namespace rmf_traffic
