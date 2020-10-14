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
  const auto x = fcl::Vector3d(p[0], p[1], 0.0);

  Eigen::Rotation2Dd R{tf.rotation()};

  _tf = fcl::Translation3d(x) * fcl::AngleAxisd(R.angle(), fcl::Vector3d::UnitZ());
}

//==============================================================================
bool StaticMotion::integrate(double /*dt*/) const
{
  // Do nothing
  return true;
}

//==============================================================================
double StaticMotion::computeMotionBound(
  const fcl::BVMotionBoundVisitor<double>&) const
{
  // TODO(MXG): Investigate the legitimacy of this implementation. Make sure
  // that this function truly should always return 0.
  return 0;
}

//==============================================================================
double StaticMotion::computeMotionBound(
  const fcl::TriangleMotionBoundVisitor<double>&) const
{
  std::cout <<
    " ----- OH NO, WE'RE USING StaticMotion::computeMotionBound(TriangleMotionBoundVisitor)!! ----- "
            << std::endl;
  // *INDENT-OFF*
  throw std::runtime_error(
    "unimplemented function: StaticMotion::computeMotionBound(TriangleMotionBoundVisitor)");
  // *INDENT-ON*
}

//==============================================================================
void StaticMotion::getCurrentTransform(fcl::Transform3d& tf) const
{
  tf = _tf;
}

//==============================================================================
void StaticMotion::getTaylorModel(fcl::TMatrix3<double>&, fcl::TVector3<double>&) const
{
  std::cout <<
    " ----- OH NO, WE'RE USING StaticMotion::getTaylorModel()!! ----- "
            << std::endl;
  // *INDENT-OFF*
  throw std::runtime_error(
    "unimplemented function: StaticMotion::getTaylorModel()");
  // *INDENT-ON*
}

} // namespace internal
} // namespace rmf_traffic
