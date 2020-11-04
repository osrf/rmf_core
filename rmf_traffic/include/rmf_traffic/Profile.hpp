/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef RMF_TRAFFIC__PROFILE_HPP
#define RMF_TRAFFIC__PROFILE_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

#include <rmf_utils/optional.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <Eigen/Dense>

namespace rmf_traffic {

//==============================================================================
class Profile
{
public:

  /// Constructor
  ///
  /// \param[in] footprint
  ///   An estimate of the space that this participant occupies.
  ///
  /// \param[in] vicinity
  ///   An estimate of the vicinity around the participant in which the presence
  ///   of other traffic would disrupt its operations. If a nullptr is used for
  ///   this, the footprint shape will be used as the vicinity.
  ///
  Profile(
    geometry::ConstFinalConvexShapePtr footprint,
    geometry::ConstFinalConvexShapePtr vicinity = nullptr);

  /// Add shape to the footprint of the participant
  ///
  /// \param[in] shape
  ///   An estimate of the space that this extra shape occupies.
  ///
  /// \param[in] offset
  ///   Offset to the additional shape, in the robot's coordinate frame
  ///
  void addFootPrintShape(geometry::ConstFinalConvexShapePtr shape, Eigen::Vector3d offset);

  /// Set the footprint of the participant.
  Profile& footprint(geometry::ConstFinalConvexShapePtr shape);

  /// Get the footprint of the participant.
  const geometry::ConstFinalConvexShapePtr& footprint() const;

  /// Set the vicinity of this participant.
  Profile& vicinity(geometry::ConstFinalConvexShapePtr shape);

  /// Get the vicinity of this participant.
  const geometry::ConstFinalConvexShapePtr& vicinity() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__PROFILE_HPP
