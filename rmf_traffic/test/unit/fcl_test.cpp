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

#include <fcl/continuous_collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/ccd/motion.h>

#include <rmf_utils/catch.hpp>

TEST_CASE("Verify that FCL can handle continuous collisions")
{
  // Make sphere
  std::shared_ptr<fcl::CollisionGeometry> sphere_geom =
    std::make_shared<fcl::Sphere>(0.5);

  std::shared_ptr<fcl::CollisionObject> object_1 =
    std::make_shared<fcl::CollisionObject>(sphere_geom);

  fcl::Transform3f tf_sphere_start;
  tf_sphere_start.setTranslation(fcl::Vec3f(1.0, 0.0, 0.0));
  object_1->setTransform(tf_sphere_start);

  fcl::Transform3f tf_sphere_final = tf_sphere_start;

  // Make box
  std::shared_ptr<fcl::CollisionGeometry> box_geom =
    std::make_shared<fcl::Box>(1.0, 1.0, 1.0);

  std::shared_ptr<fcl::CollisionObject> object_2 =
    std::make_shared<fcl::CollisionObject>(box_geom);

  fcl::Transform3f tf_box_start;
  tf_box_start.setTranslation(fcl::Vec3f(0.0, 5.0, 0.0));
  object_2->setTransform(tf_box_start);

  fcl::Transform3f tf_box_final = tf_box_start;
  tf_box_final.setTranslation(fcl::Vec3f(0.0, -5.0, 0.0));

  fcl::Quaternion3f R_box;
  R_box.fromAxisAngle(fcl::Vec3f(0.0, 0.0, 1.0), 90.0*M_PI/180.0);

  tf_box_final.setQuatRotation(R_box);

  fcl::ContinuousCollisionRequest request(100);
  request.ccd_motion_type = fcl::CCDM_LINEAR;
  fcl::ContinuousCollisionResult result;

  fcl::continuousCollide(
    object_1.get(), tf_sphere_final,
    object_2.get(), tf_box_final,
    request, result);

  CHECK(result.is_collide);
}
