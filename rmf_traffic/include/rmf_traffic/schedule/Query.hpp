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

#ifndef RMF_TRAFFIC__SCHEDULE__QUERY_HPP
#define RMF_TRAFFIC__SCHEDULE__QUERY_HPP

#include <rmf_traffic/geometry/Shape.hpp>

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <Eigen/Geometry>

namespace rmf_traffic {
namespace schedule {

namespace detail {
/// \internal We declare this private PIMPL class outside of the
/// Query::base_iterator class so that it does not need to be templated.
class QueryIteratorImplementation;
} // namespace detail

//==============================================================================
class Query
{
public:

  class SpaceTime
  {
  public:

    SpaceTime(
        Time lower_bound,
        Time upper_bound,
        geometry::ConstShapePtr shape,
        Eigen::Isometry2d tf);

    Time get_lower_time_bound() const;
    SpaceTime& set_lower_bound_time(Time time);

    Time get_upper_time_bound() const;
    SpaceTime& set_upper_time_bound(Time time);

    geometry::ConstShapePtr get_shape() const;
    SpaceTime& set_shape(geometry::ConstShapePtr shape);

    Eigen::Isometry2d get_transform() const;
    SpaceTime& set_transform(Eigen::Isometry2d tf);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  template<typename S>
  class base_iterator;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__QUERY_HPP
