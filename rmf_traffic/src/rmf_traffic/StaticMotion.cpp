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

#include "StaticMotion.hpp"

#include <iostream>

namespace rmf_traffic {
namespace internal {

//==============================================================================
StaticMotion::StaticMotion(const Eigen::Isometry2d& tf)
{
  const Eigen::Vector2d& p = tf.translation();
  const auto x = fcl::Vec3f(p[0], p[1], 0.0);

  Eigen::Rotation2Dd R{tf.rotation()};
  fcl::Quaternion3f q;
  q.fromAxisAngle(fcl::Vec3f(0.0, 0.0, 1.0), R.angle());

  _tf.setTransform(q, x);
}

//==============================================================================
bool StaticMotion::integrate(double /*dt*/) const
{
  // Do nothing
  return true;
}

//==============================================================================
fcl::FCL_REAL StaticMotion::computeMotionBound(
    const fcl::BVMotionBoundVisitor&) const
{
  std::cout << " ----- OH NO, WE'RE USING StaticMotion::computeMotionBound(BVMotionBoundVisitor)!! ----- "
            << std::endl;
  throw std::runtime_error("unimplemented function: StaticMotion::computeMotionBound(BVMotionBoundVisitor)");
}

//==============================================================================
fcl::FCL_REAL StaticMotion::computeMotionBound(
    const fcl::TriangleMotionBoundVisitor&) const
{
  std::cout << " ----- OH NO, WE'RE USING StaticMotion::computeMotionBound(TriangleMotionBoundVisitor)!! ----- "
            << std::endl;
  throw std::runtime_error("unimplemented function: StaticMotion::computeMotionBound(TriangleMotionBoundVisitor)");
}

//==============================================================================
void StaticMotion::getCurrentTransform(fcl::Transform3f& tf) const
{
  tf = _tf;
}

//==============================================================================
void StaticMotion::getTaylorModel(fcl::TMatrix3&, fcl::TVector3&) const
{
  std::cout << " ----- OH NO, WE'RE USING StaticMotion::getTaylorModel()!! ----- "
            << std::endl;
  throw std::runtime_error("unimplemented function: StaticMotion::getTaylorModel()");
}

} // namespace internal
} // namespace rmf_traffic
