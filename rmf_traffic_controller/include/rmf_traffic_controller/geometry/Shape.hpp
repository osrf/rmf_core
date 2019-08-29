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


#ifndef RMF_TRAFFIC_CONTROLLER__GEOMETRY__SHAPE_HPP
#define RMF_TRAFFIC_CONTROLLER__GEOMETRY__SHAPE_HPP

#include <memory>

namespace rmf_traffic_controller {
namespace geometry {

//==============================================================================
/// \brief This is the base class of different shape classes that can be used
/// by the rmf_traffic_controller library. This cannot (currently) be extended
/// by downstream libraries; instead, users must choose one of the pre-defined
/// shape types belonging to this library.
///
/// \sa Box, Circle, Polygon
class Shape
{
public:

  /// \internal
  class Internal;

  /// \internal
  /// These accessors can only be used by the rmf_traffic_controller library
  /// internally. The Internal class cannot be accessed by downstream users
  /// (and downstream users should never need it anyway).
  Internal* _get_internal();

  /// \internal
  const Internal* _get_internal() const;

  virtual ~Shape();

protected:

  /// \internal
  Shape(std::unique_ptr<Internal> internal);

private:

  std::unique_ptr<Internal> _internal;

};

} // namespace geometry
} // namespace rmf_traffic_controller

#endif // RMF_TRAFFIC_CONTROLLER__GEOMETRY__SHAPE_HPP
