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


#ifndef RMF_TRAFFIC_CONTROLLER__GEOMETRY__BOX_HPP
#define RMF_TRAFFIC_CONTROLLER__GEOMETRY__BOX_HPP

#include <rmf_traffic_controller/geometry/ConvexShape.hpp>

namespace rmf_traffic_controller {
namespace geometry {

//==============================================================================
/// \brief This class represent a box shape which can be added into a Zone or
/// Trajectory.
class Box : public ConvexShape
{
public:

  /// \brief Create a box with the indicated dimensions.
  ///
  /// The origin of the box will be in its center, so the box will extend:
  /// [-x_length/2.0, x_length/2.0] along its x axis and
  /// [-y_length/2.0, y_length/2.0] along its y axis.
  Box(double x_length, double y_length);

  /// \brief Set the length of the box along its x axis
  void set_x_length(double x_length);

  /// \brief Set the length of the box along its y axis
  void set_y_length(double y_length);

  /// \brief Get the length of the box along its x axis
  double get_x_length() const;

  /// \brief Get the length of the box along its y axis
  double get_y_length() const;

};


} // namespace geometry
} // namespace rmf_traffic_controller

#endif // RMF_TRAFFIC_CONTROLLER__GEOMETRY__BOX_HPP
