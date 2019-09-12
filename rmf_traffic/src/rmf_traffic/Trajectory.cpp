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

#include <map>

namespace rmf_traffic {

//==============================================================================
namespace {
struct SegmentData
{
  Trajectory::ConstProfilePtr profile;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

using SegmentDataMap = std::map<Trajectory::Time, SegmentData>;

} // anonymous namespace

//==============================================================================
class Trajectory::Implementation
{
public:

  SegmentDataMap segments;

};

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
    const std::string& queue_id)
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

//==============================================================================
class Trajectory::Segment::Implementation
{
public:

  Implementation(Trajectory::Implementation* parent)
    : parent(parent)
  {
    // Do nothing
  }

  SegmentData& data()
  {
    return it->second;
  }

  const SegmentData& data() const
  {
    return it->second;
  }

  Time time() const
  {
    return it->first;
  }

  SegmentDataMap::iterator it;
  Trajectory::Implementation* const parent;

};

//==============================================================================
auto Trajectory::Segment::get_profile() const -> ConstProfilePtr
{
  return _pimpl->data().profile;
}

//==============================================================================
void Trajectory::Segment::set_profile(ConstProfilePtr new_profile)
{
  _pimpl->data().profile = std::move(new_profile);
}

//==============================================================================
Eigen::Vector3d Trajectory::Segment::get_position() const
{
  return _pimpl->data().position;
}

//==============================================================================
void Trajectory::Segment::set_position(Eigen::Vector3d new_position)
{
  _pimpl->data().position = std::move(new_position);
}

//==============================================================================
Eigen::Vector3d Trajectory::Segment::get_velocity() const
{
  return _pimpl->data().velocity;
}

//==============================================================================
void Trajectory::Segment::set_velocity(Eigen::Vector3d new_velocity)
{
  _pimpl->data().velocity = std::move(new_velocity);
}

//==============================================================================
Trajectory::Time Trajectory::Segment::get_finish_time() const
{
  return _pimpl->time();
}

//==============================================================================
void Trajectory::Segment::set_finish_time(const Time new_time)
{
  SegmentDataMap::iterator& it = _pimpl->it;
  if(it->first == new_time)
  {
    // Short-circuit, since nothing is changing. The erase and re-insertion
    // would be a waste of time in this case.
    return;
  }

  SegmentDataMap& segments = _pimpl->parent->segments;

  // See if we can use the next iterator as a hint for where to insert the
  // new element
  const SegmentDataMap::iterator possible_hint = ++SegmentDataMap::iterator(it);

  const SegmentData data = std::move(it->second);
  // Note: erasing an iterator does not invalidate any iterators for the map,
  // except the one that was erased. Remember that `it` is a mutable reference,
  // so we have its current value erased in this next line, but we will be
  // updating its value in a moment when we insert the new version.
  segments.erase(it);

  if(possible_hint == segments.end() || new_time < possible_hint->first)
  {
    // This will make a valid hint for the insertion
    it = segments.emplace_hint(possible_hint, new_time, std::move(data));
  }
  else
  {
    // We do not have a hint readily available for the new entry, so we'll just
    // use the normal map insertion method. We also don't know if the new time
    // might conflict with an existing waypoint's time, so we should check for
    // that as well.
    const auto result = segments.emplace(new_time, std::move(data));

    if(!result.second)
    {
      // The new time conflicts with an existing time, so we will throw an
      // exception.
      throw std::invalid_argument(
            "[Trajectory::Segment::set_finish_time] Attempted to set time to "
            + std::to_string(new_time.time_since_epoch().count())
            + "ns, but a waypoint already exists at that timestamp.");
    }

    it = result.first;
  }
}

} // namespace rmf_traffic
