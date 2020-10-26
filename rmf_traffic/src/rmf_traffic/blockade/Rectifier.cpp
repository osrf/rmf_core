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

#include "internal_Rectifier.hpp"

namespace rmf_traffic {
namespace blockade {

//==============================================================================
Rectifier Rectifier::Implementation::make(
    Participant::Implementation& participant)
{
  Rectifier output;
  output._pimpl = rmf_utils::make_unique_impl<Implementation>(
        Implementation{participant});

  return output;
}

//==============================================================================
void Rectifier::check(const Status& status)
{
  _pimpl->participant.check(status);
}

//==============================================================================
Rectifier::Rectifier()
{
  // Do nothing
}

} // namespace blockade
} // namespace rmf_traffic
