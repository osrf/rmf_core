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

class FinalConvexShape;

//==============================================================================
/// \brief This class is a more specific type of Shape. The Zone class can
/// consume any kind of Shape, but the Trajectory class can only consume
/// ConvexShape types.
///
/// \sa Box, Circle
class ConvexShape : public Shape
{
public:

  /// Finalize the shape more specifically as a ConvexShape
  virtual FinalConvexShape finalize_convex() const = 0;

protected:

  ConvexShape(std::unique_ptr<Shape::Internal> internal);

};

using ConvexShapePtr = std::shared_ptr<ConvexShape>;
using ConstConvexShapePtr = std::shared_ptr<const ConvexShape>;

//==============================================================================
/// This is a finalized ConvexShape whose parameters can no longer be mutated
class FinalConvexShape : public FinalShape
{
public:

  // No API is needed here

  class Implementation;
protected:
  FinalConvexShape();
};

using FinalConvexShapePtr = std::shared_ptr<FinalConvexShape>;
using ConstFinalConvexShapePtr = std::shared_ptr<const FinalConvexShape>;

//==============================================================================
template<typename T, typename... Args>
FinalConvexShapePtr make_final_convex(Args&& ... args)
{
  return std::make_shared<FinalConvexShape>(
    T(std::forward<Args>(args)...).finalize_convex());
}

//==============================================================================
template<typename T>
FinalConvexShapePtr make_final_convex(const T& convex)
{
  return std::make_shared<FinalConvexShape>(convex.finalize_convex());
}

} // namespace geometry
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__CONVEXSHAPE_HPP
