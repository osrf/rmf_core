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
#ifdef RMF_TRAFFIC__USING_FCL_0_6
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/continuous_collision_object.h>

using FclCollisionGeometry = fcl::CollisionGeometryd;
using FclSphere = fcl::Sphered;
using FclBox = fcl::Boxd;
using FclCollisionObject = fcl::CollisionObjectd;
using FclTransform3 = fcl::Transform3d;
using FclContinuousCollisionRequest = fcl::ContinuousCollisionRequestd;
using FclContinuousCollisionResult = fcl::ContinuousCollisionResultd;
#else
#include <fcl/continuous_collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/ccd/motion.h>

using FclCollisionGeometry = fcl::CollisionGeometry;
using FclSphere = fcl::Sphere;
using FclBox = fcl::Box;
using FclCollisionObject = fcl::CollisionObject;
using FclTransform3 = fcl::Transform3f;
using FclContinuousCollisionRequest = fcl::ContinuousCollisionRequest;
using FclContinuousCollisionResult = fcl::ContinuousCollisionResult;
#endif

#include <rmf_utils/catch.hpp>

TEST_CASE("Verify that FCL can handle continuous collisions")
{
  // Make sphere
  std::shared_ptr<FclCollisionGeometry> sphere_geom =
    std::make_shared<FclSphere>(0.5);

  std::shared_ptr<FclCollisionObject> object_1 =
    std::make_shared<FclCollisionObject>(sphere_geom);

  FclTransform3 tf_sphere_start;
#ifdef RMF_TRAFFIC__USING_FCL_0_6
  tf_sphere_start.pretranslate(fcl::Vector3d(1.0, 0.0, 0.0));
#else
  tf_sphere_start.setTranslation(fcl::Vec3f(1.0, 0.0, 0.0));
#endif
  object_1->setTransform(tf_sphere_start);

  FclTransform3 tf_sphere_final = tf_sphere_start;

  // Make box
  std::shared_ptr<FclCollisionGeometry> box_geom =
    std::make_shared<FclBox>(1.0, 1.0, 1.0);

  std::shared_ptr<FclCollisionObject> object_2 =
    std::make_shared<FclCollisionObject>(box_geom);

  FclTransform3 tf_box_start;
#ifdef RMF_TRAFFIC__USING_FCL_0_6
  tf_box_start.pretranslate(fcl::Vector3d(0.0, 5.0, 0.0));
#else
  tf_box_start.setTranslation(fcl::Vec3f(0.0, 5.0, 0.0));
#endif
  object_2->setTransform(tf_box_start);

  FclTransform3 tf_box_final = tf_box_start;
#ifdef RMF_TRAFFIC__USING_FCL_0_6
  tf_box_final.pretranslate(fcl::Vector3d(0.0, -5.0, 0.0));
  tf_box_final.rotate(fcl::AngleAxisd(90.0*M_PI/180.0, fcl::Vector3d::UnitZ()));
#else
  tf_box_final.setTranslation(fcl::Vec3f(0.0, -5.0, 0.0));
  fcl::Quaternion3f R_box;
  R_box.fromAxisAngle(fcl::Vec3f(0.0, 0.0, 1.0), 90.0*M_PI/180.0);
  tf_box_final.setQuatRotation(R_box);
#endif

  FclContinuousCollisionRequest request(100);
  request.ccd_motion_type = fcl::CCDM_LINEAR;
  FclContinuousCollisionResult result;

  fcl::continuousCollide(
    object_1.get(), tf_sphere_final,
    object_2.get(), tf_box_final,
    request, result);

  CHECK(result.is_collide);
}
