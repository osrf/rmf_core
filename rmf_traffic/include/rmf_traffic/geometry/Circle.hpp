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


#ifndef RMF_TRAFFIC__GEOMETRY__CIRCLE_HPP
#define RMF_TRAFFIC__GEOMETRY__CIRCLE_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

namespace rmf_traffic {
namespace geometry {

//==============================================================================
/// \brief This class represent a circle shape which can be added into a Zone or
/// Trajectory.
class Circle : public ConvexShape
{
public:

  Circle(double radius);

  // The typical copy constructor/assignment operator
  Circle(const Circle& other);
  Circle& operator=(const Circle& other);

  void set_radius(double r);

  double get_radius() const;

};

} // namespace geometry
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__GEOMETRY__CIRCLE_HPP
