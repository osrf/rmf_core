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

#ifndef RMF_TRAFFIC__AGV__VEHICLETRAITS_HPP
#define RMF_TRAFFIC__AGV__VEHICLETRAITS_HPP

#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/Trajectory.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class VehicleTraits
{
public:

  class Limits
  {
  public:

    Limits(
      double velocity = 0.0,
      double acceleration = 0.0);

    Limits& set_nominal_velocity(double nom_vel);
    double get_nominal_velocity() const;

    Limits& set_nominal_acceleration(double nom_accel);
    double get_nominal_acceleration() const;

    /// Returns true if the values of these limits are valid, i.e. greater than
    /// zero.
    bool valid() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  enum class Steering : uint16_t
  {
    /// The vehicle uses differential steering, making it impossible to move
    /// laterally.
    Differential,

    /// The vehicle can move holonomically, so it has no limitations about how
    /// it steers.
    Holonomic,
  };

  class Differential
  {
  public:

    Differential(
      Eigen::Vector2d forward = Eigen::Vector2d::UnitX(),
      bool reversible = true);

    Differential& set_forward(Eigen::Vector2d forward);

    const Eigen::Vector2d& get_forward() const;

    Differential& set_reversible(bool reversible);
    bool is_reversible() const;

    /// Returns true if the length of the forward vector is not too close to
    /// zero. If it is too close to zero, then the direction of the forward
    /// vector cannot be reliably interpreted. Ideally the forward vector should
    /// have unit length.
    bool valid() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class Holonomic
  {
  public:

    Holonomic();

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Constructor.
  VehicleTraits(
    Limits linear,
    Limits angular,
    Profile profile,
    Differential steering = Differential());

  Limits& linear();
  const Limits& linear() const;

  Limits& rotational();
  const Limits& rotational() const;

  Profile& profile();
  const Profile& profile() const;

  Steering get_steering() const;

  Differential& set_differential(Differential parameters = Differential());

  Differential* get_differential();

  const Differential* get_differential() const;

  Holonomic& set_holonomic(Holonomic parameters);

  Holonomic* get_holonomic();

  const Holonomic* get_holonomic() const;

  /// Returns true if the values of the traits are valid. For example, this
  /// means that all velocity and acceleration values are greater than zero.
  bool valid() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__VEHICLETRAITS_HPP
