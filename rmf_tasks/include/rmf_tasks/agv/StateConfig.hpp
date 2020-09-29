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

#ifndef INCLUDE__RMF_TASKS__AGV__STATECONFIG_HPP
#define INCLUDE__RMF_TASKS__AGV__STATECONFIG_HPP

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_tasks {
namespace agv {

class StateConfig
{
public:

  /// Constructor
  ///
  /// \param[in] threshold_soc
  StateConfig(double threshold_soc);

  double threshold_soc() const;

  StateConfig& threshold_soc(double threshold_soc);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_tasks

#endif // INCLUDE__RMF_TASKS__AGV__STATE_HPP
