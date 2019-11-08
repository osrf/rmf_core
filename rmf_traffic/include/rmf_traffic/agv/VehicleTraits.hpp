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

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class VehicleTraits {
 public:
  class Limits {
   public:
    VehicleTraits& set_nominal_velocity(double nom_vel);
    double get_nominal_velocity() const;

    VehicleTraits& set_nominal_acceleration(double nom_accel);
    double get_nominal_acceleration() const;

    /// Returns true if the values of these limits are valid, i.e. greater than
    /// zero.
    bool valid() const;

   private:
    Limits(void* pimpl);
    friend class VehicleTraits;
    void* _pimpl;
  };

  /// Constructor. The default values of zero
  VehicleTraits(double nom_linear_vel = 0.0, double nom_linear_accel = 0.0,
                double nom_rotation_vel = 0.0, double nom_rotation_accel = 0.0,
                bool reversible = false);

  Limits& linear();
  const Limits& linear() const;

  Limits& rotational();
  const Limits& rotational() const;

  VehicleTraits& set_reversible(bool reversible);
  bool is_reversible() const;

  /// Returns true if the values of the traits are valid. Specifically this
  /// means that all velocity and acceleration values are greater than zero.
  bool valid() const;

  VehicleTraits(const VehicleTraits& other);
  VehicleTraits(VehicleTraits&& other);
  VehicleTraits& operator=(const VehicleTraits& other);
  VehicleTraits& operator=(VehicleTraits&& other);

 private:
  class Implementation;
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

}  // namespace agv
}  // namespace rmf_traffic

#endif  // RMF_TRAFFIC__AGV__VEHICLETRAITS_HPP
