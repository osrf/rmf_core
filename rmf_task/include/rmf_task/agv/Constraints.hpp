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

#ifndef RMF_TASK__AGV__CONSTRAINTS_HPP
#define RMF_TASK__AGV__CONSTRAINTS_HPP

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {
namespace agv {

class Constraints
{
public:

  /// Constructor
  ///
  /// \param[in] threshold_soc
  ///   Minimum charge level the battery is allowed to deplete to. This
  ///   value needs to be between 0.0 and 1.0.
  Constraints(double threshold_soc);

  /// Gets the battery state of charge threshold value.
  double threshold_soc() const;

  /// Sets a new battery state of charge threshold value. This value needs to be
  /// between 0.0 and 1.0.
  Constraints& threshold_soc(double threshold_soc);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_task

#endif // RMF_TASK__AGV__CONSTRAINTS_HPP
