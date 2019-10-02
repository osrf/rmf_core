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

#include "ShapeInternal.hpp"
#include "Spline.hpp"

#include <rmf_traffic/Conflict.hpp>

#include <fcl/continuous_collision.h>
#include <fcl/ccd/motion.h>

#include <unordered_map>

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
ConflictData::Segments ConflictData::get_segments()
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
    const Trajectory& trajectory_b)
{
  if(!broad_phase(trajectory_a, trajectory_b))
    return {};

  return narrow_phase(trajectory_a, trajectory_b);
}

//==============================================================================
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

  return true;
}

namespace {

using GeometryMap = std::unordered_map<
    geometry::ConstConvexShapePtr,
    geometry::Shape::Internal::CollisionGeometryPtr>;

//==============================================================================
GeometryMap make_geometry_map(std::vector<const Trajectory*> trajectories)
{
  GeometryMap result;
  for(const Trajectory* trajectory : trajectories)
  {
    for(Trajectory::const_iterator it = trajectory->cbegin();
        it != trajectory->cend(); ++it)
    {
      const Trajectory::ConstProfilePtr profile = it->get_profile();
      const geometry::ConstConvexShapePtr shape = profile->get_shape();
      if(!shape)
      {
        throw invalid_trajectory_error::Implementation
          ::make_missing_shape_error(it->get_finish_time());
      }

      auto insertion = result.emplace(shape, nullptr);
      if(insertion.second)
      {
        const auto fcl_shapes = shape->_get_internal()->make_fcl();
        assert(fcl_shapes.size() == 1);
        insertion.first->second = fcl_shapes.front();
      }
    }
  }

  return result;
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

} // anonymous namespace

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
    const Trajectory& trajectory_b)
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

  const GeometryMap geometries = make_geometry_map(
      std::vector<const Trajectory*>{&trajectory_a, &trajectory_b});

  // Initialize the objects that will be used inside the loop
  Spline spline_a(a_it);
  Spline spline_b(b_it);
  std::shared_ptr<fcl::SplineMotion> motion_a =
      make_uninitialized_fcl_spline_motion();
  std::shared_ptr<fcl::SplineMotion> motion_b =
      make_uninitialized_fcl_spline_motion();
  const fcl::ContinuousCollisionRequest request; // Using default values for now
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

    // TODO(MXG): When we can use C++17, we can change spline_a and spline_b to
    // use std::optional<Spline> so that we can easily keep track of which needs
    // to be updated. There's some wasted computational cycles here whenever we
    // are using the same spline as a previous iteration.
    spline_a = Spline(a_it);
    spline_b = Spline(b_it);

    const Time start_time =
        std::max(spline_a.start_time(), spline_b.start_time());
    const Time finish_time =
        std::min(spline_a.finish_time(), spline_b.finish_time());

    *motion_a = spline_a.to_fcl(start_time, finish_time);
    *motion_b = spline_b.to_fcl(start_time, finish_time);

    const auto obj_a = fcl::ContinuousCollisionObject(
          geometries.at(profile_a->get_shape()), motion_a);
    const auto obj_b = fcl::ContinuousCollisionObject(
          geometries.at(profile_b->get_shape()), motion_b);

    fcl::collide(&obj_a, &obj_b, request, result);
    if(result.is_collide)
    {
      const double scaled_time = result.time_of_contact;
      const Duration delta_t{
        Duration::rep(scaled_time * (finish_time - start_time).count())};
      const Time time = start_time + delta_t;
      conflicts.emplace_back(Implementation::make_conflict(time, {a_it, b_it}));
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

} // namespace rmf_traffic
