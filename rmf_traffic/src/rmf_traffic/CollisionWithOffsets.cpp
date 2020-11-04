
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/math/motion/spline_motion.h>

namespace rmf_traffic {

template <typename Shape1, typename Shape2, typename NarrowPhaseSolver>
bool conservativeAdvancementWithOffset(
    const Shape1& o1, const fcl::MotionBase<typename Shape1::S>* motion1, const fcl::Transform3d& offset_o1,
    const Shape2& o2, const fcl::MotionBase<typename Shape1::S>* motion2, const fcl::Transform3d& offset_o2,
    const NarrowPhaseSolver* solver, const fcl::CollisionRequest<typename Shape1::S>& request,
    fcl::CollisionResult<typename Shape1::S>& result, typename Shape1::S& toc)
{
  using S = typename Shape1::S;
  
  fcl::Transform3<S> tf1;
  fcl::Transform3<S> tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);

  // whether the first start configuration is in collision
  if (collide(&o1, tf1 * offset_o1, &o2, tf2 * offset_o2, request, result)) {
    toc = 0;
    return true;
  }

  fcl::detail::ShapeConservativeAdvancementTraversalNode<Shape1, Shape2, NarrowPhaseSolver>
      node;

  initialize(node, o1, tf1, o2, tf2, solver);

  node.motion1 = motion1;
  node.motion2 = motion2;

  do {
    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);

    std::cout << "- tf1 linear - \n" << tf1.linear() << std::endl;
    std::cout << "- tf1 translate - \n" << tf1.translation() << std::endl;
    std::cout << "- tf2 linear - \n" << tf2.linear() << std::endl;
    std::cout << "- tf2 translate - \n" << tf2.translation() << std::endl;

    node.tf1 = tf1 * offset_o1;
    node.tf2 = tf2 * offset_o2;
/*
    std::cout << "===\n";
    std::cout << node.tf1.translation() << std::endl;
    std::cout << node.tf1.linear() << std::endl;
    std::cout << "=\n";
    std::cout << node.tf2.translation() << std::endl;
    std::cout << node.tf2.linear() << std::endl;
    std::cout << "===\n";
*/
    node.delta_t = 1;
    node.min_distance = std::numeric_limits<S>::max();

    distanceRecurse(&node, 0, 0, nullptr);
    std::cout << "toc: " << node.toc << " delta: " << node.delta_t << std::endl;

    if (node.delta_t <= node.t_err) {
      // std::cout << node.delta_t << " " << node.t_err << std::endl;
      break;
    }

    node.toc += node.delta_t;
    if (node.toc > 1) {
      node.toc = 1;
      break;
    }

    node.motion1->integrate(node.toc);
    node.motion2->integrate(node.toc);
  } while (1);

  toc = node.toc;

  if (node.toc < 1) return true;

  return false;
}

//fcl::detail::GJKSolver_libccd<double>
template <typename S>
S collide_shapes_with_offset(
    const fcl::CollisionGeometry<S>* o1,
    const fcl::MotionBase<S>* motion1,
    const fcl::Transform3d& offset_o1,
    const fcl::CollisionGeometry<S>* o2,
    const fcl::MotionBase<S>* motion2,
    const fcl::Transform3d& offset_o2,
    fcl::ContinuousCollisionResult<S>& result)
{
  fcl::detail::GJKSolver_libccd<double> nsolver;
  fcl::CollisionRequest<S> c_request;
  fcl::CollisionResult<S> c_result;
  S toc = -1.0;
  bool is_collide = false;

  fcl::NODE_TYPE node_type1 = o1->getNodeType();
  fcl::NODE_TYPE node_type2 = o2->getNodeType();

  if (node_type1 == fcl::GEOM_SPHERE && node_type2 == fcl::GEOM_SPHERE) {
    std::cout << "sphere vs sphere\n";
    const fcl::Sphere<S>* obj1 = static_cast<const fcl::Sphere<S>*>(o1);
    const fcl::Sphere<S>* obj2 = static_cast<const fcl::Sphere<S>*>(o2);

    is_collide =
        conservativeAdvancementWithOffset(
        *obj1, motion1, offset_o1, *obj2, motion2, offset_o2, &nsolver,
        c_request, c_result, toc);
  } else if (node_type1 == fcl::BV_AABB && node_type2 == fcl::BV_AABB) {
    std::cout << "box vs box\n";
    const fcl::Box<S>* obj1 = static_cast<const fcl::Box<S>*>(o1);
    const fcl::Box<S>* obj2 = static_cast<const fcl::Box<S>*>(o2);

    is_collide = conservativeAdvancementWithOffset(
        *obj1, motion1, offset_o1, *obj2, motion2, offset_o2, &nsolver,
        c_request, c_result, toc);
  } else if (node_type1 == fcl::GEOM_BOX && node_type2 == fcl::GEOM_SPHERE) {
    std::cout << "box vs sphere\n";

    const fcl::Box<S>* obj1 = static_cast<const fcl::Box<S>*>(o1);
    const fcl::Sphere<S>* obj2 = static_cast<const fcl::Sphere<S>*>(o2);

    is_collide = conservativeAdvancementWithOffset(
        *obj1, motion1, offset_o1, *obj2, motion2, offset_o2, &nsolver,
        c_request, c_result, toc);
  } else if (node_type1 == fcl::GEOM_SPHERE && node_type2 == fcl::GEOM_BOX) {
    std::cout << "sphere vs box\n";

    const fcl::Sphere<S>* obj1 = static_cast<const fcl::Sphere<S>*>(o1);
    const fcl::Box<S>* obj2 = static_cast<const fcl::Box<S>*>(o2);

    is_collide = conservativeAdvancementWithOffset(
        *obj1, motion1, offset_o1, *obj2, motion2, offset_o2, &nsolver,
        c_request, c_result, toc);
  } else {
    std::cerr << "Warning: collision function between node type " << node_type1
              << " and node type " << node_type2 << " is not supported"
              << std::endl;
  }
  result.is_collide = is_collide;
  result.time_of_contact = toc;

  if (result.is_collide) {
    motion1->integrate(result.time_of_contact);
    motion2->integrate(result.time_of_contact);

    fcl::Transform3<S> tf1;
    fcl::Transform3<S> tf2;
    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
    result.contact_tf1 = tf1;
    result.contact_tf1 = result.contact_tf1 * offset_o1;
    result.contact_tf2 = tf2;
    result.contact_tf2 = result.contact_tf2 * offset_o2;
  }

  //return res;
  return toc;
}

}