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

#include <rmf_traffic/schedule/Mirror.hpp>

#include "ViewerInternal.hpp"


namespace rmf_traffic {
namespace schedule {

//==============================================================================
Mirror::Mirror()
{
  _pimpl->changers[static_cast<std::size_t>(Database::Change::Mode::Insert)]
      = [&](const Database::Change& change)
  {
    const Database::Change::Insert& insertion = *change.insert();

    std::cout << "Getting insertion [" << insertion.trajectory()->get_map_name()
              << "]" << std::endl;

    _pimpl->add_entry(
          std::make_shared<internal::Entry>(
            *insertion.trajectory(),
            change.id()));
  };

  _pimpl->changers[static_cast<std::size_t>(Database::Change::Mode::Interrupt)]
      = [&](const Database::Change& change)
  {
    const Database::Change::Interrupt& interruption = *change.interrupt();

    const internal::EntryPtr& entry =
        _pimpl->get_entry_iterator(
          interruption.original_id(), "interruption")->second;

    std::cout << "Getting interruption [" << entry->trajectory.get_map_name()
              << "] <<< [" << interruption.interruption()->get_map_name()
              << "]" << std::endl;;

    Trajectory new_trajectory = add_interruption(
          entry->trajectory,
          *interruption.interruption(),
          interruption.delay());

    _pimpl->modify_entry(entry, std::move(new_trajectory), change.id());
  };

  _pimpl->changers[static_cast<std::size_t>(Database::Change::Mode::Delay)]
      = [&](const Database::Change& change)
  {
    const Database::Change::Delay& delay = *change.delay();

    const internal::EntryPtr& entry =
        _pimpl->get_entry_iterator(delay.original_id(), "delay")->second;

    std::cout << "Getting delay [" << entry->version
              << "] --> [" << change.id() << "]" << std::endl;

    Trajectory new_trajectory = add_delay(
          entry->trajectory,
          delay.from(),
          delay.duration());

    _pimpl->modify_entry(entry, std::move(new_trajectory), change.id());
  };

  _pimpl->changers[static_cast<std::size_t>(Database::Change::Mode::Replace)]
      = [&](const Database::Change& change)
  {
    const Database::Change::Replace& replace = *change.replace();

    try
    {
      const internal::EntryPtr& entry =
          _pimpl->get_entry_iterator(
            replace.original_id(), "replacement")->second;

      std::cout << "Getting replacement [" << entry->version
                << "] --> [" << change.id() << "]" << std::endl;

      _pimpl->modify_entry(entry, *replace.trajectory(), change.id());
    }
    catch(const std::runtime_error& e)
    {
      // Sometimes replacements have been failing because the previous entry
      // somehow doesn't exist anymore, so we'll just treat this replacement
      // like an insertion
      // TODO(MXG): This shouldn't really be happening, so debug this when time
      // permits.
      _pimpl->add_entry(
            std::make_shared<internal::Entry>(
              *replace.trajectory(),
              change.id()));

      // We'll continue with throwing the exception so that noise keeps getting
      // made about this issue.
      throw e;
    }
  };

  _pimpl->changers[static_cast<std::size_t>(Database::Change::Mode::Erase)]
      = [&](const Database::Change& change)
  {
    const Database::Change::Erase& erase = *change.erase();

    {
      const internal::EntryPtr& entry =
          _pimpl->get_entry_iterator(erase.original_id(), "erase")->second;
      std::cout << "Getting erase [" << entry->trajectory.get_map_name()
                << "]" << std::endl;
    }

    _pimpl->erase_entry(erase.original_id());
  };

  _pimpl->changers[static_cast<std::size_t>(Database::Change::Mode::Cull)]
      = [&](const Database::Change& change)
  {

    std::cout << "GETTING CULL????!!@!" << std::endl;

    const Database::Change::Cull& cull = *change.cull();
    _pimpl->cull(change.id(), cull.time());
  };
}

//==============================================================================
Version Mirror::update(const Database::Patch& patch)
{
  for(const auto& change : patch)
    _pimpl->changers[static_cast<std::size_t>(change.get_mode())](change);

  _pimpl->latest_version = patch.latest_version();

  return _pimpl->latest_version;
}

} // schedule
} // rmf_traffic
