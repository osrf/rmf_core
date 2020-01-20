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
#include "Spline.hpp"
#include "StaticMotion.hpp"

#include <rmf_traffic/Conflict.hpp>
#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <fcl/continuous_collision.h>
#include <fcl/ccd/motion.h>

#include <unordered_map>
#include <mutex>
#include <thread>

namespace rmf_traffic {

//==============================================================================
class ConflictData::Implementation
{
public:

  Time time;
  Segments segments;

};

//==============================================================================
Time ConflictData::get_time() const
{
  return _pimpl->time;
}

//==============================================================================
const ConflictData::Segments& ConflictData::get_segments() const
{
  return _pimpl->segments;
}

//==============================================================================
ConflictData::ConflictData()
{
  // Do nothing
}

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
        + "profile of its segment at time ["
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

//==============================================================================
std::vector<ConflictData> DetectConflict::between(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b,
    const bool quit_after_one)
{
  if(!broad_phase(trajectory_a, trajectory_b))
    return {};

  return narrow_phase(trajectory_a, trajectory_b, quit_after_one);
}

//==============================================================================

namespace {

using BoundingBox = std::pair<Eigen::Vector2d, Eigen::Vector2d>;
struct Coeffs
{
public:
  double a;
  double b;
  double c;
  double d;
  double ti;
  double tf;

  Coeffs()
  {
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    ti = 0;
    tf = 0;
  }

  Coeffs(
      double a_,
      double b_,
      double c_,
      double d_,
      double ti_,
      double tf_)
  : a(a_),
    b(b_),
    c(c_),
    d(d_),
    ti(ti_),
    tf(tf_)
  {}
};

// Container for storing the coefficients of x and y spline motions
using SplineCoeffs = std::pair<Coeffs, Coeffs>;

SplineCoeffs get_spline_coefficients(
    rmf_traffic::Trajectory::const_iterator seg_a,
    rmf_traffic::Trajectory::const_iterator seg_b)
{
  SplineCoeffs spline_coeffs;

  auto get_coeffs = [&](int index) -> Coeffs
  {
    assert(index >= 0 && index < 2);
    // Time in seconds and positions in meters
    double ti = seg_a->get_finish_time().time_since_epoch().count() / 1E9;
    double tf = seg_b->get_finish_time().time_since_epoch().count() / 1E9;
    double initial_position = seg_a->get_finish_position()[index];
    double final_position = seg_b->get_finish_position()[index];
    double initial_velocity = seg_a->get_finish_velocity()[index];
    double final_velocity = seg_b->get_finish_velocity()[index];

    double td = (tf - ti);
    double x0 = initial_position;
    double x1 = final_position;
    double v0 = initial_velocity;
    double v1 = final_velocity;
    double w0 = v0/td;
    double w1 = v1/td;
    
    double a = w1 + w0 -2*x1 + 2*x0;
    double b = -w1 - 2*w0 +3*x1 -3*x0;
    double c = w0;
    double d = x0;

    // For debugging purposes
    // if (index == 0)
    //   std::cout << "x(t)= " << a <<"t^3 + " << b << "t^2 + " << c << "t + " << d <<std::endl;
    // else
    //   std::cout << "y(t)= " << a <<"y^3 + " << b << "y^2 + " << c << "y + " << d <<std::endl;

    return Coeffs(a, b, c, d, ti, tf);
  };

  spline_coeffs.first = get_coeffs(0);
  spline_coeffs.second = get_coeffs(1);

  return spline_coeffs;
}

double evaluate_spline(
    const Coeffs coeffs,
    const double t)
{
  // Assume time is parameterized [0,1]
  return (coeffs.a * t * t * t
      + coeffs.b * t * t
      + coeffs.c * t
      + coeffs.d);
}

void get_local_extrema(
    const Coeffs& coeffs,
    std::vector<double>& sols)
{
  // Store boundary values as potential extrema
  sols.emplace_back(evaluate_spline(coeffs, 0));
  sols.emplace_back(evaluate_spline(coeffs, 1));

  // When derivate of spline motion is not quadratic
  if (coeffs.a == 0)
  {
    if (coeffs.b != 0)
      sols.emplace_back(-coeffs.c / coeffs.b);
  }
  else
  {
    // Calculate the discriminant otherwise
    double D = (4 * coeffs.b * coeffs.b) - 12 * coeffs.a * coeffs.c;

    if (D < 0)
    {
      // Do nothing
    }
    else if (D == 0)
    {
      double t = (-2 * coeffs.b) / (6 * coeffs.a);
      double extrema = evaluate_spline(coeffs, t);
      sols.emplace_back(extrema);
    }
    else
    {
      double t1 = ((-2 * coeffs.b) + std::sqrt(D)) / (6 * coeffs.a);
      double t2 = ((-2 * coeffs.b) - std::sqrt(D)) / (6 * coeffs.a);

      double extrema1 = evaluate_spline(coeffs, t1);
      double extrema2 = evaluate_spline(coeffs, t2);

      sols.emplace_back(extrema1);
      sols.emplace_back(extrema2);
    }
  }
}

void get_solutions(
    rmf_traffic::Trajectory::const_iterator begin,
    rmf_traffic::Trajectory::const_iterator end,
    std::vector<double>& x_sols,
    std::vector<double>& y_sols)
{
  for (auto it = begin; it < end; it++)
  {
    auto it2 = it;
    auto spline_coeffs = get_spline_coefficients(it, ++it2);
    get_local_extrema(spline_coeffs.first, x_sols);
    get_local_extrema(spline_coeffs.second, y_sols);
  }
}

BoundingBox get_bounding_box(const Trajectory& trajectory)
{
  BoundingBox bounding_box;
  std::vector<double> x_sols;
  std::vector<double> y_sols;

  auto begin_it = trajectory.find(*trajectory.start_time());
  auto end_it = trajectory.find(*trajectory.finish_time());
  assert(begin_it != trajectory.end());
  assert(end_it != trajectory.end());

  get_solutions(begin_it, end_it, x_sols, y_sols);

  Eigen::Vector2d min_coord = Eigen::Vector2d{
      *std::min_element(x_sols.begin(), x_sols.end()),
      *std::min_element(y_sols.begin(), y_sols.end())};

  Eigen::Vector2d max_coord = Eigen::Vector2d{
      *std::max_element(x_sols.begin(), x_sols.end()),
      *std::max_element(y_sols.begin(), y_sols.end())};

  // Applying offsets for profile of trajectory
  // TODO get characteristic length from geometry::FinalShape
  // Current behavior is undefined if profile is Box.
  double char_length = -1;
  try
  {
    char_length = static_cast<const rmf_traffic::geometry::Circle&>(
    trajectory.begin()->get_profile()->get_shape()->source()).get_radius();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  if (char_length > 0)
  {
    min_coord -= Eigen::Vector2d{char_length, char_length};
    max_coord += Eigen::Vector2d{char_length, char_length};
  }

  bounding_box.first = min_coord;
  bounding_box.second = max_coord;

  return bounding_box;
}

BoundingBox get_bounding_box(const rmf_traffic::Trajectory::const_iterator end)
{
  BoundingBox bounding_box;
  std::vector<double> x_sols;
  std::vector<double> y_sols;

  rmf_traffic::Trajectory::const_iterator begin =
      --rmf_traffic::Trajectory::const_iterator(end);
  auto spline_coeffs = get_spline_coefficients(begin, end);

  get_local_extrema(spline_coeffs.first, x_sols);
  get_local_extrema(spline_coeffs.second, y_sols);

  Eigen::Vector2d min_coord = Eigen::Vector2d{
      *std::min_element(x_sols.begin(), x_sols.end()),
      *std::min_element(y_sols.begin(), y_sols.end())};

  Eigen::Vector2d max_coord = Eigen::Vector2d{
      *std::max_element(x_sols.begin(), x_sols.end()),
      *std::max_element(y_sols.begin(), y_sols.end())};

  // Applying offsets for profile of trajectory
  // TODO get characteristic length from geometry::FinalShape
  // Current behavior is undefined if profile is Box.
  double char_length = -1;
  try
  {
    char_length = static_cast<const rmf_traffic::geometry::Circle&>(
    end->get_profile()->get_shape()->source()).get_radius();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  if (char_length > 0)
  {
    min_coord -= Eigen::Vector2d{char_length, char_length};
    max_coord += Eigen::Vector2d{char_length, char_length};
  }

  bounding_box.first = min_coord;
  bounding_box.second = max_coord;

  return bounding_box;
}

bool overlap(const BoundingBox& box_a, const BoundingBox& box_b)
{
  auto centre = [&](const BoundingBox& box) -> Eigen::Vector2d
  {
    return (0.5 * (box.second + box.first));
  };

  Eigen::Vector2d centre_a = centre(box_a);
  Eigen::Vector2d centre_b = centre(box_b);
  double width_a = std::abs(box_a.second[0] - box_a.first[0]);
  double height_a = std::abs(box_a.second[1] - box_a.first[1]);
  double width_b = std::abs(box_b.second[0] - box_b.first[0]);
  double height_b = std::abs(box_b.second[1] - box_b.first[1]);

  // std::cout << "Box A: << Centre [" << centre_a[0] << ", " << centre_a[1] << "] "
  //     << "Width " << width_a << " Height " << height_a <<std::endl;
  // std::cout << "Box B: << Centre [" << centre_b[0] << ", " << centre_b[1] << "] "
  //     << "Width " << width_b << " Height " << height_b <<std::endl;

  return std::abs(centre_b[0] - centre_a[0]) < 0.5 * (width_a + width_b)
      &&  std::abs(centre_b[1] - centre_a[1]) < 0.5 * (height_a + height_b);
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
std::tuple<Trajectory::const_iterator, Trajectory::const_iterator>
get_initial_iterators(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b)
{
  std::size_t min_size = std::min(trajectory_a.size(), trajectory_b.size());
  if(min_size < 2)
  {
    throw invalid_trajectory_error::Implementation
        ::make_segment_num_error(min_size);
  }

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
fcl::ContinuousCollisionRequest make_fcl_request()
{
  fcl::ContinuousCollisionRequest request;
  request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
  request.gjk_solver_type = fcl::GST_LIBCCD;

  return request;
}

} // anonymous namespace

bool DetectConflict::broad_phase(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b)
{
  std::size_t min_size = std::min(trajectory_a.size(), trajectory_b.size());
  if(min_size < 2)
  {
    throw invalid_trajectory_error::Implementation
        ::make_segment_num_error(min_size);
  }

  if(trajectory_a.get_map_name() != trajectory_b.get_map_name())
    return false;

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

  // Iterate through the segments of both trajectories to check for overlapping
  // bounding boxes
  Trajectory::const_iterator a_it;
  Trajectory::const_iterator b_it;
  std::tie(a_it, b_it) = get_initial_iterators(trajectory_a, trajectory_b);
  assert(a_it != trajectory_a.end());
  assert(b_it != trajectory_b.end());

  Spline spline_a(a_it);
  Spline spline_b(b_it);

  while(a_it != trajectory_a.end() && b_it != trajectory_b.end())
  {
    // Increment a_it until spline_a will overlap with spline_b
    if(a_it->get_finish_time() < spline_b.start_time())
    {
      ++a_it;
      continue;
    }

    // Increment b_it until spline_b will overlap with spline_a
    if(b_it->get_finish_time() < spline_a.start_time())
    {
      ++b_it;
      continue;
    }

    spline_a = Spline(a_it);
    spline_b = Spline(b_it);

    auto box_a = get_bounding_box(a_it);
    auto box_b = get_bounding_box(b_it);

    if (overlap(box_a, box_b))
      return true;

    if(spline_a.finish_time() < spline_b.finish_time())
    {
      ++a_it;
    }
    else if(spline_b.finish_time() < spline_a.finish_time())
    {
      ++b_it;
    }
    else
    {
      ++a_it;
      ++b_it;
    }
  }
  return false;
}

class DetectConflict::Implementation
{
public:
  static ConflictData make_conflict(Time time, ConflictData::Segments segments)
  {
    ConflictData result;
    result._pimpl = rmf_utils::make_impl<ConflictData::Implementation>(
          ConflictData::Implementation{time, std::move(segments)});

    return result;
  }
};

//==============================================================================
std::vector<ConflictData> DetectConflict::narrow_phase(
    const Trajectory& trajectory_a,
    const Trajectory& trajectory_b,
    const bool quit_after_one)
{
  Trajectory::const_iterator a_it;
  Trajectory::const_iterator b_it;
  std::tie(a_it, b_it) = get_initial_iterators(trajectory_a, trajectory_b);

  // Verify that neither trajectory has run into a bug. These conditions should
  // be guaranteed by
  // 1. The assumption that the trajectories overlap (this is an assumption that
  //    is made explicit to the user)
  // 2. The min_size check up above
  assert(a_it != trajectory_a.end());
  assert(b_it != trajectory_b.end());

  // Initialize the objects that will be used inside the loop
  Spline spline_a(a_it);
  Spline spline_b(b_it);
  std::shared_ptr<fcl::SplineMotion> motion_a =
      make_uninitialized_fcl_spline_motion();
  std::shared_ptr<fcl::SplineMotion> motion_b =
      make_uninitialized_fcl_spline_motion();

  const fcl::ContinuousCollisionRequest request = make_fcl_request();
  fcl::ContinuousCollisionResult result;
  std::vector<ConflictData> conflicts;

  while(a_it != trajectory_a.end() && b_it != trajectory_b.end())
  {
    // Increment a_it until spline_a will overlap with spline_b
    if(a_it->get_finish_time() < spline_b.start_time())
    {
      ++a_it;
      continue;
    }

    // Increment b_it until spline_b will overlap with spline_a
    if(b_it->get_finish_time() < spline_a.start_time())
    {
      ++b_it;
      continue;
    }

    const Trajectory::ConstProfilePtr profile_a = a_it->get_profile();
    const Trajectory::ConstProfilePtr profile_b = b_it->get_profile();

    // TODO(MXG): Consider using optional<Spline> so that we can easily keep
    // track of which needs to be updated. There's some wasted computational
    // cycles here whenever we are using the same spline as a previous iteration
    spline_a = Spline(a_it);
    spline_b = Spline(b_it);

    const Time start_time =
        std::max(spline_a.start_time(), spline_b.start_time());
    const Time finish_time =
        std::min(spline_a.finish_time(), spline_b.finish_time());

    *motion_a = spline_a.to_fcl(start_time, finish_time);
    *motion_b = spline_b.to_fcl(start_time, finish_time);

    assert(profile_a->get_shape());
    assert(profile_b->get_shape());
    const auto obj_a = fcl::ContinuousCollisionObject(
          geometry::FinalConvexShape::Implementation::get_collision(
            *profile_a->get_shape()), motion_a);
    const auto obj_b = fcl::ContinuousCollisionObject(
          geometry::FinalConvexShape::Implementation::get_collision(
            *profile_b->get_shape()), motion_b);

    fcl::collide(&obj_a, &obj_b, request, result);
    if(result.is_collide)
    {
      const double scaled_time = result.time_of_contact;
      const Duration delta_t{
        Duration::rep(scaled_time * (finish_time - start_time).count())};
      const Time time = start_time + delta_t;
      conflicts.emplace_back(Implementation::make_conflict(time, {a_it, b_it}));
      if (quit_after_one)
        return conflicts;
    }

    if(spline_a.finish_time() < spline_b.finish_time())
    {
      ++a_it;
    }
    else if(spline_b.finish_time() < spline_a.finish_time())
    {
      ++b_it;
    }
    else
    {
      ++a_it;
      ++b_it;
    }
  }

  return conflicts;
}

namespace internal {
//==============================================================================
bool detect_conflicts(
    const Trajectory& trajectory,
    const Spacetime& region,
    std::vector<Trajectory::const_iterator>* output_iterators)
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

  bool collision_detected = false;

  for(auto it = begin_it; it != end_it; ++it)
  {
    const Trajectory::ConstProfilePtr profile = it->get_profile();

    Spline spline_trajectory{it};

    const Time spline_start_time =
        std::max(spline_trajectory.start_time(), start_time);
    const Time spline_finish_time =
        std::min(spline_trajectory.finish_time(), finish_time);

    *motion_trajectory = spline_trajectory.to_fcl(
          spline_start_time, spline_finish_time);

    assert(profile->get_shape());
    const auto obj_trajectory = fcl::ContinuousCollisionObject(
          geometry::FinalConvexShape::Implementation::get_collision(
            *profile->get_shape()), motion_trajectory);

    assert(region.shape);
    const auto& region_shapes = geometry::FinalShape::Implementation
        ::get_collisions(*region.shape);
    for(const auto& region_shape : region_shapes)
    {
      const auto obj_region = fcl::ContinuousCollisionObject(
            region_shape, motion_region);

      fcl::ContinuousCollisionResult result;
      fcl::collide(&obj_trajectory, &obj_region, request, result);
      if(result.is_collide)
      {
        if(output_iterators)
        {
          output_iterators->push_back(it);
          collision_detected = true;
        }
        else
        {
          return true;
        }
      }
    }
  }

  return collision_detected;
}
} // namespace internal

} // namespace rmf_traffic
