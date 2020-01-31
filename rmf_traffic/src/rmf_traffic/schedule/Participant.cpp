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
class ParticipantDescription::Implementation
{
public:

  std::string name;
  std::string owner;
  Rx responsiveness;
  geometry::ConstFinalConvexShapePtr footprint;
  geometry::ConstFinalConvexShapePtr vicinity;

};

//==============================================================================
ParticipantDescription::ParticipantDescription(
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
ParticipantDescription& ParticipantDescription::name(std::string value)
{
  _pimpl->name = std::move(value);
  return *this;
}

//==============================================================================
const std::string& ParticipantDescription::name() const
{
  return _pimpl->name;
}

//==============================================================================
ParticipantDescription& ParticipantDescription::owner(std::string value)
{
  _pimpl->owner = std::move(value);
  return *this;
}

//==============================================================================
const std::string& ParticipantDescription::owner() const
{
  return _pimpl->owner;
}

//==============================================================================
ParticipantDescription& ParticipantDescription::responsiveness(Rx value)
{
  _pimpl->responsiveness = value;
  return *this;
}

//==============================================================================
ParticipantDescription::Rx ParticipantDescription::responsiveness() const
{
  return _pimpl->responsiveness;
}

//==============================================================================
ParticipantDescription& ParticipantDescription::footprint(geometry::ConstFinalConvexShapePtr shape)
{
  _pimpl->footprint = std::move(shape);
  return *this;
}

//==============================================================================
const geometry::ConstFinalConvexShapePtr& ParticipantDescription::footprint() const
{
  return _pimpl->footprint;
}

//==============================================================================
ParticipantDescription& ParticipantDescription::vicinity(geometry::ConstFinalConvexShapePtr shape)
{
  _pimpl->vicinity = std::move(shape);
  return *this;
}

//==============================================================================
const geometry::ConstFinalConvexShapePtr& ParticipantDescription::vicinity() const
{
  return _pimpl->vicinity;
}

} // namespace schedule
} // namespace rmf_traffic
