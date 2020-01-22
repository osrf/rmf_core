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

#include <rmf_traffic/schedule/Participant.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Participant::Implementation
{
public:

  std::string name;
  std::string owner;
  Rx responsiveness;
  geometry::ConstFinalConvexShapePtr footprint;
  geometry::ConstFinalConvexShapePtr vicinity;

};

//==============================================================================
Participant::Participant(
    std::string name,
    std::string owner,
    Rx responsiveness,
    geometry::ConstFinalConvexShapePtr footprint,
    geometry::ConstFinalConvexShapePtr vicinity)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(name),
               std::move(owner),
               responsiveness,
               std::move(footprint),
               std::move(vicinity)
             }))
{
  // Do nothing
}

//==============================================================================
Participant& Participant::name(std::string value)
{
  _pimpl->name = std::move(value);
  return *this;
}

//==============================================================================
const std::string& Participant::name() const
{
  return _pimpl->name;
}

//==============================================================================
Participant& Participant::owner(std::string value)
{
  _pimpl->owner = std::move(value);
  return *this;
}

//==============================================================================
const std::string& Participant::owner() const
{
  return _pimpl->owner;
}

//==============================================================================
Participant& Participant::responsiveness(Rx value)
{
  _pimpl->responsiveness = value;
  return *this;
}

//==============================================================================
Participant::Rx Participant::responsiveness() const
{
  return _pimpl->responsiveness;
}

//==============================================================================
Participant& Participant::footprint(geometry::ConstFinalConvexShapePtr shape)
{
  _pimpl->footprint = std::move(shape);
  return *this;
}

//==============================================================================
const geometry::ConstFinalConvexShapePtr& Participant::footprint() const
{
  return _pimpl->footprint;
}

//==============================================================================
Participant& Participant::vicinity(geometry::ConstFinalConvexShapePtr shape)
{
  _pimpl->vicinity = std::move(shape);
  return *this;
}

//==============================================================================
const geometry::ConstFinalConvexShapePtr& Participant::vicinity() const
{
  return _pimpl->vicinity;
}

} // namespace schedule
} // namespace rmf_traffic
