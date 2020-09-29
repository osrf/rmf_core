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

#include <rmf_tasks/agv/StateConfig.hpp>

namespace rmf_tasks {
namespace agv {

class StateConfig::Implementation
{
public:
  double threshold_soc;

};

StateConfig::StateConfig(double threshold_soc)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation
    {
      threshold_soc
    }))
{
  // Do nothing
}

double StateConfig::threshold_soc() const
{
  return _pimpl->threshold_soc;
}

auto StateConfig::threshold_soc(double threshold_soc) -> StateConfig&
{
  _pimpl->threshold_soc = threshold_soc;
  return *this;
}


} // namespace agv
} // namespace rmf_tasks