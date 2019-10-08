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

#include "ViewerInternal.hpp"

#include <rmf_traffic/schedule/Database.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Database::Change::Implementation
{
public:

  Mode mode;
  Insert insert;
  Interrupt interrupt;
  Delay delay;
  Replace replace;
  Erase erase;
  Cull cull;
};

namespace {
//==============================================================================
// TODO(MXG): Consider generalizing this class using templates if it ends up
// being useful in any other context.
class DeepOrShallowTrajectory
{
public:

  const Trajectory* get() const
  {
    if(is_deep)
      return deep.get();

    return shallow;
  }

  rmf_utils::impl_ptr<const Trajectory> deep;
  const Trajectory* shallow = nullptr;
  bool is_deep = true;

};

//==============================================================================
DeepOrShallowTrajectory make_deep(const Trajectory* deep)
{
  DeepOrShallowTrajectory traj;
  traj.is_deep = true;
  if(deep)
    traj.deep = rmf_utils::make_impl<const Trajectory>(*deep);

  return traj;
}

DeepOrShallowTrajectory make_shallow(const Trajectory* shallow)
{
  DeepOrShallowTrajectory traj;
  traj.is_deep = false;
  traj.shallow = shallow;

  return traj;
}

} // anonymous namespace

//==============================================================================
class Database::Change::Insert::Implementation
{
public:

  /// The trajectory that was inserted
  DeepOrShallowTrajectory trajectory;

  /// The id of the change
  std::size_t id;

  /// This is used to create a Change whose lifetime is not tied to the Database
  /// instance that constructed it.
  static Insert make_copy(
      const Trajectory* trajectory,
      std::size_t id)
  {
    Insert result;

    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{make_deep(trajectory), id});

    return result;
  }

  /// This is used by Database instances to create a Change without the overhead
  /// of copying a Trajectory instance. The Change's lifetime will be tied to
  /// the lifetime of the Database object.
  static Insert make_ref(
      const Trajectory* trajectory,
      std::size_t id)
  {
    Insert result;

    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{make_shallow(trajectory), id});

    return result;
  }
};

//==============================================================================
Database::Change::Insert::Insert()
{
  // Do nothing
}

//==============================================================================
Database::Change::Change()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto Database::Change::make_insert(
    const Trajectory* trajectory,
    std::size_t id) -> Change
{
  Change change;
  change._pimpl->mode = Mode::Insert;
  change._pimpl->insert = Insert::Implementation::make_copy(trajectory, id);

  return change;
}

//==============================================================================
class Database::Change::Interrupt::Implementation
{
public:

  std::size_t original_id;
  DeepOrShallowTrajectory trajectory;
  Duration delay;
  std::size_t id;

};

//==============================================================================
Database::Change::Interrupt::Interrupt()
{
  // Do nothing
}

//==============================================================================
auto Database::Change::make_interrupt(
    std::size_t original_id,
    const Trajectory* interruption_trajectory,
    Duration delay,
    std::size_t id) -> Change
{
  Change change;
  change._pimpl->mode = Mode::Interrupt;
  change._pimpl->interrupt = Interrupt::Implementation::make
}

} // namespace schedule
} // namespace rmf_traffic
