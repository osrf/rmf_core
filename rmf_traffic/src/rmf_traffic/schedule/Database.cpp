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
class DeepOrShallowTrajectory
{
public:

  // TODO(MXG): Use this to store the Trajectories in the Change interface
  // classes

};
} // anonymous namespace

//==============================================================================
class Database::Change::Insert::Implementation
{
public:

  /// Is true if the Trajectory for this Change is a deep copy instead of a
  /// reference
  bool deep_copy = true;

  /// If deep_copy is true, this is a copy of the inserted Trajectory
  std::unique_ptr<Trajectory> copied;

  /// If deep_copy is false, this is a reference to the inserted Trajectory
  const Trajectory* ref;

  /// The id of the change
  std::size_t id;

  /// This is used to create a Change whose lifetime is not tied to the Database
  /// instance that constructed it.
  static Insert make_copy(
      const Trajectory* trajectory,
      std::size_t id)
  {
    Insert result;

    if(trajectory)
    {
      result._pimpl = rmf_utils::make_impl<Implementation>(
            Implementation{true, std::make_unique<Trajectory>(*trajectory),
                           nullptr, id});
    }
    else
    {
      result._pimpl = rmf_utils::make_impl<Implementation>(
            Implementation{true, nullptr, nullptr, id});
    }

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
          Implementation{false, nullptr, trajectory, id});

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
  change._pimpl->mode Mode::Interrupt;

}

} // namespace schedule
} // namespace rmf_traffic
