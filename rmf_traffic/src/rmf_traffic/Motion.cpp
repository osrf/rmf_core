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

#include "MotionInternal.hpp"

namespace rmf_traffic {

//==============================================================================
SinglePointMotion::SinglePointMotion(const Time t, Eigen::Vector3d p,
                                     Eigen::Vector3d v)
    : _t(t), _p(std::move(p)), _v(std::move(v)) {
  // Do nothing
}

//==============================================================================
Time SinglePointMotion::start_time() const { return _t; }

//==============================================================================
Time SinglePointMotion::finish_time() const { return _t; }

//==============================================================================
Eigen::Vector3d SinglePointMotion::compute_position(Time /*t*/) const {
  return _p;
}

//==============================================================================
Eigen::Vector3d SinglePointMotion::compute_velocity(Time /*t*/) const {
  return _v;
}

//==============================================================================
Eigen::Vector3d SinglePointMotion::compute_acceleration(Time /*t*/) const {
  return Eigen::Vector3d::Zero();
}

//==============================================================================
SplineMotion::SplineMotion(Spline spline) : _spline(std::move(spline)) {
  // Do nothing
}

//==============================================================================
Time SplineMotion::start_time() const { return _spline.start_time(); }

//==============================================================================
Time SplineMotion::finish_time() const { return _spline.finish_time(); }

//==============================================================================
Eigen::Vector3d SplineMotion::compute_position(Time t) const {
  return _spline.compute_position(t);
}

//==============================================================================
Eigen::Vector3d SplineMotion::compute_velocity(Time t) const {
  return _spline.compute_velocity(t);
}

//==============================================================================
Eigen::Vector3d SplineMotion::compute_acceleration(Time t) const {
  return _spline.compute_acceleration(t);
}

}  // namespace rmf_traffic
