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

#include "geometry/ShapeInternal.hpp"
#include "DetectConflictInternal.hpp"
#include "ProfileInternal.hpp"
#include "Spline.hpp"
#include "StaticMotion.hpp"

#include "DetectConflictInternal.hpp"

#include <fcl/continuous_collision.h>
#include <fcl/ccd/motion.h>
#include <fcl/collision.h>

#include <unordered_map>

namespace rmf_traffic {

//==============================================================================
class invalid_trajectory_error::Implementation
{
public:

  std::string what;

  static invalid_trajectory_error make_segment_num_error(
      std::size_t num_segments)
  {
    invalid_trajectory_error error;
    error._pimpl->what = std::string()
        + "[rmf_traffic::invalid_trajectory_error] Attempted to check a "
        + "conflict with a Trajectory that has [" + std::to_string(num_segments)
        + "] segments. This is not supported. Trajectories must have at least "
        + "2 segments to check them for conflicts.";
    return error;
  }

  static invalid_trajectory_error make_missing_shape_error(
      const Time time)
  {
    invalid_trajectory_error error;
    error._pimpl->what = std::string()
        + "[rmf_traffic::invalid_trajectory_error] Attempting to check a "
        + "conflict with a Trajectory that has no shape specified for the "
        + "profile of its waypoint at time ["
        + std::to_string(time.time_since_epoch().count())
        + "ns]. This is not supported.";

    return error;
  }
};

//==============================================================================
const char* invalid_trajectory_error::what() const noexcept
{
  return _pimpl->what.c_str();
}

//==============================================================================
invalid_trajectory_error::invalid_trajectory_error()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // This constructor is a no-op, but we'll keep a definition for it in case we
  // need it in the future. Allowing the default constructor to be inferred
  // could cause issues if we want to change the implementation of this
  // exception in the future, like if we want to add more information to the
  // error message output.
}

namespace {
//==============================================================================
bool have_time_overlap(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b)
{
  const auto* t_a0 = trajectory_a.start_time();
  const auto* t_bf = trajectory_b.finish_time();

  // Neither of these can be null, because both trajectories should have at
  // least two elements.
  assert(t_a0 != nullptr);
  assert(t_bf != nullptr);

  if(*t_bf < *t_a0)
  {
    // If Trajectory `b` finishes before Trajectory `a` starts, then there
    // cannot be any conflict.
    return false;
  }

  const auto* t_b0 = trajectory_b.start_time();
  const auto* t_af = trajectory_a.finish_time();

  // Neither of these can be null, because both trajectories should have at
  // least two elements.
  assert(t_b0 != nullptr);
  assert(t_af != nullptr);

  if(*t_af < *t_b0)
  {
    // If Trajectory `a` finished before Trajectory `b` starts, then there
    // cannot be any conflict.
    return false;
  }

  return true;
}

//==============================================================================
std::tuple<Trajectory::const_iterator, Trajectory::const_iterator>
get_initial_iterators(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b)
{
  const Time& t_a0 = *trajectory_a.start_time();
  const Time& t_b0 = *trajectory_b.start_time();

  Trajectory::const_iterator a_it;
  Trajectory::const_iterator b_it;

  if(t_a0 < t_b0)
  {
    // Trajectory `a` starts first, so we begin evaluating at the time
    // that `b` begins
    a_it = trajectory_a.find(t_b0);
    b_it = ++trajectory_b.begin();
  }
  else if(t_b0 < t_a0)
  {
    // Trajectory `b` starts first, so we begin evaluating at the time
    // that `a` begins
    a_it = ++trajectory_a.begin();
    b_it = trajectory_b.find(t_a0);
  }
  else
  {
    // The Trajectories begin at the exact same time, so both will begin
    // from their start
    a_it = ++trajectory_a.begin();
    b_it = ++trajectory_b.begin();
  }

  return {a_it, b_it};
}

//==============================================================================
struct BoundingBox
{
  Eigen::Vector2d min;
  Eigen::Vector2d max;
};

//==============================================================================
struct BoundingProfile
{
  BoundingBox footprint;
  BoundingBox vicinity;
};

//==============================================================================
double evaluate_spline(
    const Eigen::Vector4d& coeffs,
    const double t)
{
  // Assume time is parameterized [0,1]
  return (coeffs[3] * t * t * t
      + coeffs[2] * t * t
      + coeffs[1] * t
      + coeffs[0]);
}

//==============================================================================
std::array<double, 2> get_local_extrema(
    const Eigen::Vector4d& coeffs)
{
  std::vector<double> extrema_candidates;
  // Store boundary values as potential extrema
  extrema_candidates.emplace_back(evaluate_spline(coeffs, 0));
  extrema_candidates.emplace_back(evaluate_spline(coeffs, 1));

  // When derivate of spline motion is not quadratic
  if (std::abs(coeffs[3]) < 1e-12)
  {
    if (std::abs(coeffs[2]) > 1e-12)
    {
      double t = -coeffs[1] / (2 * coeffs[2]);
      extrema_candidates.emplace_back(evaluate_spline(coeffs, t));
    }
  }
  else
  {
    // Calculate the discriminant otherwise
    const double D = (4 * pow(coeffs[2], 2) - 12 * coeffs[3] * coeffs[1]);

    if (std::abs(D) < 1e-4)
    {
      const double t = (-2 * coeffs[2]) / (6 * coeffs[3]);
      const double extrema = evaluate_spline(coeffs, t);
      extrema_candidates.emplace_back(extrema);
    }
    else if (D < 0)
    {
      // If D is negative, then the local extrema would be imaginary. This will
      // happen for splines that have no local extrema. When that happens, the
      // endpoints of the spline are the only extrema.
    }
    else
    {
      const double t1 = ((-2 * coeffs[2]) + std::sqrt(D)) / (6 * coeffs[3]);
      const double t2 = ((-2 * coeffs[2]) - std::sqrt(D)) / (6 * coeffs[3]);

      extrema_candidates.emplace_back(evaluate_spline(coeffs, t1));
      extrema_candidates.emplace_back(evaluate_spline(coeffs, t2));
    }
  }

  std::array<double, 2> extrema;
  assert(!extrema_candidates.empty());
  extrema[0] = *std::min_element(
      extrema_candidates.begin(),
      extrema_candidates.end());
  extrema[1] = *std::max_element(
      extrema_candidates.begin(),
      extrema_candidates.end());

  return extrema;
}

//==============================================================================
BoundingBox get_bounding_box(const rmf_traffic::Spline& spline)
{
  auto params = spline.get_params();
  std::array<double, 2> extrema_x = get_local_extrema(params.coeffs[0]);
  std::array<double, 2> extrema_y = get_local_extrema(params.coeffs[1]);

  return BoundingBox{
    Eigen::Vector2d{extrema_x[0], extrema_y[0]},
    Eigen::Vector2d{extrema_x[1], extrema_y[1]}
  };
}

//==============================================================================
/// Create a bounding box which will never overlap with any other BoundingBox
BoundingBox void_box()
{
  constexpr double inf = std::numeric_limits<double>::infinity();
  return BoundingBox{
    Eigen::Vector2d{inf, inf},
    Eigen::Vector2d{-inf, -inf}
  };
}

//==============================================================================
BoundingBox adjust_bounding_box(
    const BoundingBox& input,
    const double value)
{
  BoundingBox box = input;
  box.min -= Eigen::Vector2d{value, value};
  box.max += Eigen::Vector2d{value, value};

  return box;
}

//==============================================================================
BoundingBox get_bounding_footprint(
    const rmf_traffic::Spline& spline,
    const Profile::Implementation& profile)
{
  if (profile.footprint)
  {
    return adjust_bounding_box(
          get_bounding_box(spline),
          profile.footprint->get_characteristic_length());
  }

  return void_box();
}

//==============================================================================
BoundingProfile get_bounding_profile(
    const rmf_traffic::Spline& spline,
    const Profile::Implementation& profile)
{
  BoundingBox base_box = get_bounding_box(spline);

  const auto& footprint = profile.footprint;
  const auto f_box = footprint ?
        adjust_bounding_box(base_box, footprint->get_characteristic_length())
      : void_box();

  const auto& vicinity = profile.vicinity;
  const auto v_box = vicinity ?
        adjust_bounding_box(base_box, vicinity->get_characteristic_length())
      : void_box();

  return BoundingProfile{f_box, v_box};
}

//==============================================================================
bool overlap(
    const BoundingBox& box_a,
    const BoundingBox& box_b)
{
  for (int i=0; i < 2; ++i)
  {
    if (box_a.max[i] < box_b.min[i])
      return false;

    if (box_b.max[i] < box_a.min[i])
      return false;
  }

  return true;
}

//==============================================================================
std::shared_ptr<fcl::SplineMotion> make_uninitialized_fcl_spline_motion()
{
  // This function is only necessary because SplineMotion does not provide a
  // default constructor, and we want to be able to instantiate one before
  // we have any paramters to provide to it.
  fcl::Matrix3f R;
  fcl::Vec3f T;

  // The constructor that we are using is a no-op (apparently it was declared,
  // but its definition is just `// TODO`, so we don't need to worry about
  // unintended consequences. If we update the version of FCL, this may change,
  // so I'm going to leave a FIXME tag here to keep us aware of that.
  return std::make_shared<fcl::SplineMotion>(R, T, R, T);
}

//==============================================================================
fcl::ContinuousCollisionRequest make_fcl_request()
{
  fcl::ContinuousCollisionRequest request;
  request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
  request.gjk_solver_type = fcl::GST_LIBCCD;

  return request;
}

//==============================================================================
rmf_utils::optional<fcl::FCL_REAL> check_collision(
    const geometry::FinalConvexShape& shape_a,
    const std::shared_ptr<fcl::SplineMotion>& motion_a,
    const geometry::FinalConvexShape& shape_b,
    const std::shared_ptr<fcl::SplineMotion>& motion_b,
    const fcl::ContinuousCollisionRequest& request)
{
  const auto obj_a = fcl::ContinuousCollisionObject(
        geometry::FinalConvexShape::Implementation::get_collision(shape_a),
        motion_a);

  const auto obj_b = fcl::ContinuousCollisionObject(
        geometry::FinalConvexShape::Implementation::get_collision(shape_b),
        motion_b);

  fcl::ContinuousCollisionResult result;
  fcl::collide(&obj_a, &obj_b, request, result);

  if (result.is_collide)
    return result.time_of_contact;

  return rmf_utils::nullopt;
}

//==============================================================================
Profile::Implementation convert_profile(const Profile& profile)
{
  Profile::Implementation output = Profile::Implementation::get(profile);
  if (!output.vicinity)
    output.vicinity = output.footprint;

  return output;
}

//==============================================================================
Time compute_time(
    const fcl::FCL_REAL scaled_time,
    const Time start_time,
    const Time finish_time)
{
  const Duration delta_t{
    Duration::rep(scaled_time * (finish_time - start_time).count())
  };

  return start_time + delta_t;
}

} // anonymous namespace

//==============================================================================
bool DetectConflict::between(
    const Profile& profile_a,
    const Trajectory& trajectory_a,
    const Profile& profile_b,
    const Trajectory& trajectory_b,
    Interpolate interpolation)
{
  return Implementation::between(
        profile_a, trajectory_a,
        profile_b, trajectory_b,
        interpolation);
}

namespace {
//==============================================================================
fcl::Transform3f convert(Eigen::Vector3d p)
{
  fcl::Matrix3f R;
  R.setEulerZYX(0.0, 0.0, p[2]);
  return fcl::Transform3f(R, fcl::Vec3f(p[0], p[1], 0.0));
}

//==============================================================================
bool close_start(
    const Profile::Implementation& profile_a,
    const Trajectory::const_iterator& a_it,
    const Profile::Implementation& profile_b,
    const Trajectory::const_iterator& b_it)
{
  // If two trajectories start very close to each other, then we do not consider
  // it a conflict for them to be in each other's vicinities. This gives robots
  // an opportunity to back away from each other without it being considered a
  // schedule conflict.

  if (profile_a.footprint == profile_a.vicinity
      && profile_b.footprint == profile_b.vicinity)
  {
    // If there is no difference between the footprint and vicinity for either
    // vehicle, then there is no such thing as a "close start".
    return false;
  }

  Spline spline_a(a_it);
  Spline spline_b(b_it);
  const auto start_time =
      std::max(spline_a.start_time(), spline_b.start_time());

  using ConvexPair = std::array<geometry::ConstFinalConvexShapePtr, 2>;
  std::array<ConvexPair, 2> pairs = {
    ConvexPair{profile_a.footprint, profile_b.vicinity},
    ConvexPair{profile_a.vicinity, profile_b.footprint}
  };

  fcl::CollisionRequest request;
  fcl::CollisionResult result;
  for (const auto pair : pairs)
  {
    fcl::CollisionObject obj_a(
          geometry::FinalConvexShape::Implementation::get_collision(*pair[0]),
          convert(spline_a.compute_position(start_time)));

    fcl::CollisionObject obj_b(
          geometry::FinalConvexShape::Implementation::get_collision(*pair[1]),
          convert(spline_b.compute_position(start_time)));

    if (fcl::collide(&obj_a, &obj_b, request, result) > 0)
      return true;
  }

  return false;
}

} // anonymous namespace

//==============================================================================
bool DetectConflict::Implementation::between(
    const Profile& input_profile_a,
    const Trajectory& trajectory_a,
    const Profile& input_profile_b,
    const Trajectory& trajectory_b,
    Interpolate /*interpolation*/,
    std::vector<Conflict>* output_conflicts)
{
  const std::size_t min_size =
      std::min(trajectory_a.size(), trajectory_b.size());
  if(min_size < 2)
  {
    throw invalid_trajectory_error::Implementation
        ::make_segment_num_error(min_size);
  }

  const Profile::Implementation profile_a = convert_profile(input_profile_a);
  const Profile::Implementation profile_b = convert_profile(input_profile_b);

  // Return early if there is no geometry in the profiles
  // TODO(MXG): Should this produce an exception? Is this an okay scenario?
  if (!profile_a.footprint && !profile_b.footprint)
    return false;

  // Return early if either profile is missing both a vicinity and a footprint.
  // NOTE(MXG): Since convert_profile will promote the vicinity to have the same
  // value as the footprint when the vicinity has a nullptr value, checking if
  // a vicinity doesn't exist is the same as checking that both the vicinity and
  // footprint doesn't exist.
  if (!profile_a.vicinity || !profile_b.vicinity)
    return false;

  // Return early if there is no time overlap between the trajectories
  if (!have_time_overlap(trajectory_a, trajectory_b))
    return false;

  Trajectory::const_iterator a_it;
  Trajectory::const_iterator b_it;
  std::tie(a_it, b_it) = get_initial_iterators(trajectory_a, trajectory_b);

  rmf_utils::optional<Spline> spline_a;
  rmf_utils::optional<Spline> spline_b;

  std::shared_ptr<fcl::SplineMotion> motion_a =
      make_uninitialized_fcl_spline_motion();
  std::shared_ptr<fcl::SplineMotion> motion_b =
      make_uninitialized_fcl_spline_motion();

  const fcl::ContinuousCollisionRequest request = make_fcl_request();

  // This flag lets us know that only a collision between the footprints of the
  // vehicles will count as a conflict.
  const bool test_footprints = close_start(profile_a, a_it, profile_b, b_it);

  // This flag lets us know that we need to test both a's footprint in b's
  // vicinity and b's footprint in a's vicinity.
  const bool test_complement =
         (profile_a.vicinity != profile_a.footprint)
      || (profile_b.vicinity != profile_b.footprint);

  if (output_conflicts)
    output_conflicts->clear();

  while (a_it != trajectory_a.end() && b_it != trajectory_b.end())
  {
    if (!spline_a)
      spline_a = Spline(a_it);

    if (!spline_b)
      spline_b = Spline(b_it);

    const Time start_time =
        std::max(spline_a->start_time(), spline_b->start_time());

    const Time finish_time =
        std::min(spline_a->finish_time(), spline_b->finish_time());

    *motion_a = spline_a->to_fcl(start_time, finish_time);
    *motion_b = spline_b->to_fcl(start_time, finish_time);

    if (test_footprints)
    {
      const auto bound_a = get_bounding_footprint(*spline_a, profile_a);
      const auto bound_b = get_bounding_footprint(*spline_b, profile_b);

      if (overlap(bound_a, bound_b))
      {
        if (const auto collision = check_collision(
              *profile_a.footprint, motion_a,
              *profile_b.footprint, motion_b, request))
        {
          if (!output_conflicts)
            return true;

          output_conflicts->emplace_back(
                Conflict{
                  a_it, b_it, compute_time(*collision, start_time, finish_time)
                });
        }
      }
    }
    else
    {
      const auto bound_a = get_bounding_profile(*spline_a, profile_a);
      const auto bound_b = get_bounding_profile(*spline_b, profile_b);

      if (overlap(bound_a.footprint, bound_b.vicinity))
      {
        if (const auto collision = check_collision(
              *profile_a.footprint, motion_a,
              *profile_b.vicinity, motion_b, request))
        {
          if (!output_conflicts)
            return true;

          output_conflicts->emplace_back(
                Conflict{
                  a_it, b_it, compute_time(*collision, start_time, finish_time)
                });
        }
      }

      if (test_complement && overlap(bound_a.vicinity, bound_b.footprint))
      {
        if (const auto collision = check_collision(
              *profile_a.vicinity, motion_a,
              *profile_b.footprint, motion_b, request))
        {
          if (!output_conflicts)
            return true;

          output_conflicts->emplace_back(
                Conflict{
                  a_it, b_it, compute_time(*collision, start_time, finish_time)
                });
        }
      }
    }

    if (spline_a->finish_time() < spline_b->finish_time())
    {
      spline_a = rmf_utils::nullopt;
      ++a_it;
    }
    else if (spline_b->finish_time() < spline_a->finish_time())
    {
      spline_b = rmf_utils::nullopt;
      ++b_it;
    }
    else
    {
      spline_a = rmf_utils::nullopt;
      ++a_it;

      spline_b = rmf_utils::nullopt;
      ++b_it;
    }
  }

  if (!output_conflicts)
    return false;

  return !output_conflicts->empty();
}

namespace internal {
//==============================================================================
bool detect_conflicts(
    const Profile& profile,
    const Trajectory& trajectory,
    const Spacetime& region,
    DetectConflict::Implementation::Conflicts* output_conflicts)
{
#ifndef NDEBUG
  // This should never actually happen because this function only gets used
  // internally, and so there should be several layers of quality checks on the
  // trajectories to prevent this. But we'll put it in here just in case.
  if(trajectory.size() < 2)
  {
    std::cerr << "[rmf_traffic::internal::detect_conflicts] An invalid "
              << "trajectory was passed to detect_conflicts. This is a bug "
              << "that should never happen. Please alert the RMF developers."
              << std::endl;
    throw invalid_trajectory_error::Implementation
        ::make_segment_num_error(trajectory.size());
  }
#endif // NDEBUG

  const auto vicinity = profile.vicinity();
  if (!vicinity)
    return false;

  const Time trajectory_start_time = *trajectory.start_time();
  const Time trajectory_finish_time = *trajectory.finish_time();

  const Time start_time = region.lower_time_bound?
        std::max(*region.lower_time_bound, trajectory_start_time)
      : trajectory_start_time;

  const Time finish_time = region.upper_time_bound?
        std::min(*region.upper_time_bound, trajectory_finish_time)
      : trajectory_finish_time;

  if(finish_time < start_time)
  {
    // If the trajectory or region finishes before the other has started, that
    // means there is no overlap in time between the region and the trajectory,
    // so it is impossible for them to conflict.
    return false;
  }

  const Trajectory::const_iterator begin_it =
      trajectory_start_time < start_time?
        trajectory.find(start_time) : ++trajectory.begin();

  const Trajectory::const_iterator end_it =
      finish_time < trajectory_finish_time?
        ++trajectory.find(finish_time) : trajectory.end();

  std::shared_ptr<fcl::SplineMotion> motion_trajectory =
      make_uninitialized_fcl_spline_motion();
  std::shared_ptr<internal::StaticMotion> motion_region =
      std::make_shared<internal::StaticMotion>(region.pose);

  const fcl::ContinuousCollisionRequest request = make_fcl_request();

  const std::shared_ptr<fcl::CollisionGeometry> vicinity_geom =
      geometry::FinalConvexShape::Implementation::get_collision(*vicinity);

  if (output_conflicts)
    output_conflicts->clear();

  for(auto it = begin_it; it != end_it; ++it)
  {
    Spline spline_trajectory{it};

    const Time spline_start_time =
        std::max(spline_trajectory.start_time(), start_time);
    const Time spline_finish_time =
        std::min(spline_trajectory.finish_time(), finish_time);

    *motion_trajectory = spline_trajectory.to_fcl(
          spline_start_time, spline_finish_time);

    const auto obj_trajectory = fcl::ContinuousCollisionObject(
          vicinity_geom, motion_trajectory);

    assert(region.shape);
    const auto& region_shapes = geometry::FinalShape::Implementation
        ::get_collisions(*region.shape);
    for(const auto& region_shape : region_shapes)
    {
      const auto obj_region = fcl::ContinuousCollisionObject(
            region_shape, motion_region);

      // TODO(MXG): We should do a broadphase test here before using
      // fcl::collide

      fcl::ContinuousCollisionResult result;
      fcl::collide(&obj_trajectory, &obj_region, request, result);
      if(result.is_collide)
      {
        if (!output_conflicts)
          return true;

        output_conflicts->emplace_back(
              DetectConflict::Implementation::Conflict{
                it, it,
                compute_time(
                  result.time_of_contact,
                  spline_start_time,
                  spline_finish_time)
              });
      }
    }
  }

  if (!output_conflicts)
    return false;

  return !output_conflicts->empty();
}
} // namespace internal

} // namespace rmf_traffic
