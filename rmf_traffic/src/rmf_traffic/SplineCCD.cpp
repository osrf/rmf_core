
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/math/motion/spline_motion.h>
#include "Spline.hpp"

namespace rmf_traffic {

// Check osrf/rmf_planner_viz for a demo of these 2 functions

// attempts 
static double max_splinemotion_advancement(double current_t,
  FclSplineMotion& motion_a, 
  FclSplineMotion& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  const Eigen::Vector3d& d_normalized, double max_dist,
  uint& dist_checks, double tolerance)
{
  assert(tolerance >= 0.0);
  
  double lower_t_limit = current_t;
  double upper_t_limit = 1.0;
  uint bilateral_adv_iter = 0;
  
  double s1 = max_dist, s2 = max_dist;
  double sample_t = 0.0;
  for (;;)
  {
#ifdef DO_LOGGING
    if (bilateral_adv_iter < 3)
      printf("#1: (%f,%f) #2: (%f,%f)\n", lower_t_limit, s1, upper_t_limit, s2);
#endif
    
    // alternate between bisection and false position methods
    if (bilateral_adv_iter & 1/* && ((s1 < 0.0 && s2 > 0.0) || (s1 > 0.0 && s2 < 0.0))*/)
    {
      // use false position method
      // solve for t where (t, tolerance) for a line with 
      // endpoints (lower_t_limit, s1) and (upper_t_limit, s2)
      // where s2 is negative and s1 is positive
      double inv_m = (upper_t_limit - lower_t_limit) / (s2 - s1);
      sample_t = lower_t_limit + (tolerance - s1) * inv_m;
    }
    else // bisection method
      sample_t = lower_t_limit + 0.5 * (upper_t_limit - lower_t_limit);
#ifdef DO_LOGGING
    printf("iteration: %d picked t: %f\n", bilateral_adv_iter, sample_t);
#endif

    // integrate
    motion_a.integrate(sample_t);
    motion_b.integrate(sample_t);

    fcl::Transform3d a_tx, b_tx;
    motion_a.getCurrentTransform(a_tx);
    motion_b.getCurrentTransform(b_tx);

    double s = DBL_MAX;
    // compute closest distance between all 4 shapes in direction d
    for (const auto& a_shape : a_shapes)
    {
      auto a_shape_tx = a_tx * a_shape._transform;
      for (const auto& b_shape : b_shapes)
      {
        auto b_shape_tx = b_tx * b_shape._transform;
        Eigen::Vector3d b_to_a = a_shape_tx.translation() - b_shape_tx.translation();
        
        double b_to_a_dist = b_to_a.norm();
        double dist_between_shapes_along_d = 0.0;
        if (b_to_a_dist > 1e-04)
        {
          auto b_to_a_norm = b_to_a / b_to_a_dist;
          double dist_along_b_to_a = b_to_a_dist - (a_shape._radius + b_shape._radius);
          auto v = dist_along_b_to_a * b_to_a_norm;
          dist_between_shapes_along_d = v.dot(d_normalized);
        }
        
        // get the minimum
        if (dist_between_shapes_along_d < s)
          s = dist_between_shapes_along_d;
      }
    }

#ifdef DO_LOGGING
    printf("dist_output: %f\n", s);
#endif
    if (abs(s) < tolerance)
    {
#ifdef DO_LOGGING
      printf("minimal dist %f within tolerance range %f\n", s, tolerance);
#endif
      break;
    }

    // our window is very small and we're hopping around, so we stop.
    // Also, this is what box2d does.
    if (bilateral_adv_iter >= 25) 
    {
#ifdef DO_LOGGING
      printf("range too small\n");
#endif
      break;
    }
    
    if (s < 0.0)
    {
      upper_t_limit = sample_t;
      s2 = s;
    }
    else if (s > 0.0)
    {
      lower_t_limit = sample_t;
      s1 = s;
    }
    ++bilateral_adv_iter;
  }
  dist_checks += bilateral_adv_iter;

  return sample_t;
}

bool collide_seperable_circles(
  FclSplineMotion& motion_a,
  FclSplineMotion& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, uint& dist_checks,
  uint safety_maximum_checks, double tolerance)
{
  if (a_shapes.empty() || b_shapes.empty())
    return false;

  auto calc_min_dist = [](
    const fcl::Transform3d& a_tx,
    const fcl::Transform3d& b_tx,
    const std::vector<ModelSpaceShape>& a_shapes,
    const std::vector<ModelSpaceShape>& b_shapes,
    Eigen::Vector3d& d, double& min_dist)
  {
    min_dist = DBL_MAX;
    for (const auto& a_shape : a_shapes)
    {
      auto a_shape_tx = a_tx * a_shape._transform;

      for (const auto& b_shape : b_shapes)
      {
        auto b_shape_tx = b_tx * b_shape._transform;
        Eigen::Vector3d b_to_a = a_shape_tx.translation() - b_shape_tx.translation();
        double dist = b_to_a.norm() - (a_shape._radius + b_shape._radius);
        if (dist < min_dist)
        {
          min_dist = dist;
          d = b_to_a;
        }
      }
    }
  };

  fcl::Transform3d a_start_tf, b_start_tf;
  fcl::Transform3d a_tf, b_tf;

  motion_a.integrate(0.0);
  motion_b.integrate(0.0);
  motion_a.getCurrentTransform(a_start_tf);
  motion_b.getCurrentTransform(b_start_tf);

  double dist_along_d_to_cover = 0.0;
  Eigen::Vector3d d(0,0,0);
  calc_min_dist(a_start_tf, b_start_tf, a_shapes, b_shapes,
    d, dist_along_d_to_cover);
  
  double t = 0.0;
  uint iter = 0;
  while (dist_along_d_to_cover > tolerance && t < 1.0)
  {
    Eigen::Vector3d d_normalized = d.normalized();
#ifdef DO_LOGGING
    printf("======= iter:%d\n", iter);
    std::cout << "d_norm: \n" << d_normalized << std::endl;
#endif

    t = max_splinemotion_advancement(t, motion_a, motion_b, a_shapes, b_shapes, 
      d_normalized, dist_along_d_to_cover, dist_checks, tolerance);
#ifdef DO_LOGGING
    printf("max_splinemotion_advancement returns t: %f\n", t);
#endif

    motion_a.integrate(t);
    motion_b.integrate(t);

    motion_a.getCurrentTransform(a_tf);
    motion_b.getCurrentTransform(b_tf);

    calc_min_dist(a_tf, b_tf, a_shapes, b_shapes,
      d, dist_along_d_to_cover);
    
    ++dist_checks;
    ++iter;

    //infinite loop prevention. you should increase safety_maximum_checks if you still want a solution
    if (dist_checks > safety_maximum_checks)
      break;
  }
  
  if (dist_checks > safety_maximum_checks)
    return false;

  if (t >= 0.0 && t < 1.0)
  {
    impact_time = t;
#ifdef DO_LOGGING
    printf("time of impact: %f\n", t);
#endif
    return true;
  }
#ifdef DO_LOGGING
  printf("no collide\n");
#endif
  return false;
}

}