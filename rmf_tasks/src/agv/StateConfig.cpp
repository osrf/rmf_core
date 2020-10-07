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

#include <stdexcept>

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
  if (threshold_soc < 0.0 || threshold_soc > 1.0)
    throw std::invalid_argument(
      "Battery State of Charge threshold needs be between 0.0 and 1.0.");
}

double StateConfig::threshold_soc() const
{
  return _pimpl->threshold_soc;
}

auto StateConfig::threshold_soc(double threshold_soc) -> StateConfig&
{
  if (threshold_soc < 0.0 || threshold_soc > 1.0)
    throw std::invalid_argument(
      "Battery State of Charge threshold needs be between 0.0 and 1.0.");

  _pimpl->threshold_soc = threshold_soc;
  return *this;
}


} // namespace agv
} // namespace rmf_tasks