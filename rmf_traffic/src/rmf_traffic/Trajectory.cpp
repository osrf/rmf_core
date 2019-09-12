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

#include <rmf_traffic/Trajectory.hpp>

namespace rmf_traffic {

//==============================================================================
class Trajectory::Profile::Implementation
{
public:

  Implementation(geometry::ConstConvexShapePtr shape)
    : shape(std::move(shape)),
      queue_info(this)
  {
    // Do nothing
  }

  // Basic information
  geometry::ConstConvexShapePtr shape;

  // Queue information (only used when in Queue movement mode)
  QueueInfo queue_info;
  std::string queue_id;

  // Movement mode
  Movement movement;
};

//==============================================================================
Trajectory::ProfilePtr Trajectory::Profile::make_strict(
    geometry::ConstConvexShapePtr shape)
{
  ProfilePtr result(new Profile(std::move(shape)));
  result->set_to_strict();
  return result;
}

//==============================================================================
Trajectory::ProfilePtr Trajectory::Profile::make_autonomous(
    geometry::ConstConvexShapePtr shape)
{
  ProfilePtr result(new Profile(std::move(shape)));
  result->set_to_autonomous();
  return result;
}

//==============================================================================
Trajectory::ProfilePtr Trajectory::Profile::make_queued(
    geometry::ConstConvexShapePtr shape,
    const std::string& queue_id,
    const uint32_t queue_number)
{
  ProfilePtr result(new Profile(std::move(shape)));
  result->set_to_queued(queue_id);
  return result;
}

//==============================================================================
geometry::ConstConvexShapePtr Trajectory::Profile::get_shape() const
{
  return _pimpl->shape;
}

//==============================================================================
void Trajectory::Profile::set_shape(geometry::ConstConvexShapePtr new_shape)
{
  _pimpl->shape = std::move(new_shape);
}

//==============================================================================
Trajectory::Profile::Movement Trajectory::Profile::get_movement() const
{
  return _pimpl->movement;
}

//==============================================================================
void Trajectory::Profile::set_to_strict()
{
  _pimpl->movement = Strict;
}

//==============================================================================
void Trajectory::Profile::set_to_autonomous()
{
  _pimpl->movement = Autonomous;
}

//==============================================================================
void Trajectory::Profile::set_to_queued(const std::string& queue_id)
{
  _pimpl->movement = Queued;
  _pimpl->queue_id = queue_id;
}

//==============================================================================
std::string Trajectory::Profile::QueueInfo::get_queue_id() const
{
  return static_cast<const Profile::Implementation*>(_pimpl)->queue_id;
}

//==============================================================================
Trajectory::Profile::QueueInfo::QueueInfo(void* pimpl)
  : _pimpl(pimpl)
{
  // Do nothing
}

//==============================================================================
auto Trajectory::Profile::get_queue_info() const -> const QueueInfo*
{
  if(Queued == _pimpl->movement)
    return &_pimpl->queue_info;

  return nullptr;
}

//==============================================================================
Trajectory::Profile::Profile(geometry::ConstConvexShapePtr shape)
  : _pimpl(rmf_utils::make_impl<Implementation>(std::move(shape)))
{
  // Do nothing
}

} // namespace rmf_traffic
