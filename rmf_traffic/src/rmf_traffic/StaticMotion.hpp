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

#ifndef SRC__RMF_TRAFFIC__STATICMOTION_HPP
#define SRC__RMF_TRAFFIC__STATICMOTION_HPP

#include "DetectConflictInternal.hpp"

#ifdef RMF_TRAFFIC__USING_FCL_0_6
#include <fcl/math/motion/motion_base.h>
#else
#include <fcl/ccd/motion.h>
#endif

namespace rmf_traffic {
namespace internal {

//==============================================================================
#ifdef RMF_TRAFFIC__USING_FCL_0_6
class StaticMotion : public fcl::MotionBased
#else
class StaticMotion : public fcl::MotionBase
#endif
{
public:

  StaticMotion() = default;

  StaticMotion(const Eigen::Isometry2d& tf);

  virtual bool integrate(double dt) const final;

#ifdef RMF_TRAFFIC__USING_FCL_0_6
  virtual double computeMotionBound(
    const fcl::BVMotionBoundVisitor<double>&) const final;

  virtual double computeMotionBound(
    const fcl::TriangleMotionBoundVisitor<double>&) const final;

  void getCurrentTransform(fcl::Transform3d& tf) const final;

  void getTaylorModel(fcl::TMatrix3<double>&, fcl::TVector3<double>&) const final;

private:
  fcl::Transform3d _tf;
#else
  virtual double computeMotionBound(
    const fcl::BVMotionBoundVisitor&) const final;

  virtual double computeMotionBound(
    const fcl::TriangleMotionBoundVisitor&) const final;

  void getCurrentTransform(fcl::Transform3f& tf) const final;

  void getTaylorModel(fcl::TMatrix3&, fcl::TVector3&) const final;

private:
  fcl::Transform3f _tf;
#endif
};

} // namespace internal
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__STATICMOTION_HPP
