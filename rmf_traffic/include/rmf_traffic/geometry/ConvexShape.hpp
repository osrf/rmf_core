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

#ifndef RMF_TRAFFIC__CONVEXSHAPE_HPP
#define RMF_TRAFFIC__CONVEXSHAPE_HPP

#include <rmf_traffic/geometry/Shape.hpp>

#include <memory>

namespace rmf_traffic {
namespace geometry {

//==============================================================================
/// \brief This class is a more specific type of Shape. The Zone class can
/// consume any kind of Shape, but the Trajectory class can only consume
/// ConvexShape types.
///
/// \sa Box, Circle
class ConvexShape : public Shape {
 protected:
  ConvexShape(std::unique_ptr<Shape::Internal> internal);
};

using ConvexShapePtr = std::shared_ptr<ConvexShape>;
using ConstConvexShapePtr = std::shared_ptr<const ConvexShape>;

}  // namespace geometry
}  // namespace rmf_traffic

#endif  // RMF_TRAFFIC__CONVEXSHAPE_HPP
