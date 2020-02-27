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

#include "InconsistencyTracker.hpp"
#include "InconsistenciesInternal.hpp"
#include "Modular.hpp"

namespace rmf_traffic {
namespace schedule {

InconsistencyTracker::InconsistencyTracker(
    RangesSet& ranges,
    ItineraryVersion& last_known_version)
: _ranges(ranges),
  _last_known_version(last_known_version)
{
  // Do nothing
}

//==============================================================================
void InconsistencyTracker::Ticket::set(std::function<void ()> change)
{
  _set = true;
  _callback = change;
}

//==============================================================================
InconsistencyTracker::Ticket::Ticket(
    InconsistencyTracker& parent,
    std::function<void()>& callback)
: _parent(parent),
  _callback(callback)
{
  // Do nothing
}

//==============================================================================
InconsistencyTracker::Ticket::~Ticket()
{
  // I think it would be better to do something stronger than an assert here,
  // but throwing an exception in a destructor is considered bad practice.
  assert(_set);

  // TODO(MXG): Consider whether this operation should be performed here in the
  // destructor or above in the set() function. An argument for doing it here is
  // that there would be no negative side-effects of the Ticket-holder calling
  // set() multiple times, although it would be dubious for the caller to do
  // that in the first place.
  if (_parent._ready)
  {
    _parent._apply_changes();
  }
}

//==============================================================================
auto InconsistencyTracker::check(
    const ItineraryVersion version,
    const bool nullifying)
-> std::unique_ptr<Ticket>
{
  using Range = Inconsistencies::Ranges::Range;

  // Check if this is the lastest itinerary version for this trajectory
  if (modular(_last_known_version).less_than(version))
    _last_known_version = version;

  if (_ranges.empty())
  {
    if (version == _expected_version)
    {
      // This means we have no inconsistencies to worry about
      ++_expected_version;
      return nullptr;
    }

    // This should have been checked earlier by the caller, but we will assert
    // it here just to make sure.
    assert(!modular(version).less_than(_expected_version));

    // This is the only inconsistency, so it should be easy to fill in the set:
    _ranges.insert(Range{_expected_version, version-1});

    const auto c_it = _changes.insert(std::make_pair(version, nullptr)).first;
    return std::make_unique<Ticket>(*this, c_it->second);
  }
  else
  {
    // There are already inconsistencies with this participant, so we will check
    // how this new entry affects the current ranges of inconsistencies and
    // adjust them as needed.

    const auto c_insertion = _changes.insert(std::make_pair(version, nullptr));
    auto& callback = c_insertion.first->second;

    // TODO(MXG): Should we care about cases where a repeat ticket is issued for
    // the same inconsistent change number? It would be largely irrelevant for a
    // new copy of the change to replace the old copy, because they should be
    // functionally the same as long as the Participant is a reliable actor. It
    // would duplicate some allocation and copying needlessly, but that probably
    // won't have a noticeable impact. When we have benchmark tests for the
    // schedule we can try to make it more efficient by exposing when repeat
    // tickets are issued and see if that improves performance.
    if (!c_insertion.second)
    {
      // We have already received this change in the past, so we don't need to
      // modify the inconsistency ranges. We're assuming that repeats of the
      // changes we receive from participants will be consistent with themselves
      return std::make_unique<Ticket>(*this, callback);
    }

    if (nullifying)
    {
      // If this is a nullifying change, then we can erase all recorded changes
      // that predate it.
      auto c_old = _changes.begin();
      while (c_old->first < version)
      {
        _changes.erase(c_old++);
        assert(c_old != _changes.end());
      }

      // We can update the expected version because earlier changes no longer
      // matter.
      //
      // NOTE(MXG): We should make it equal to version, not equal to version+1
      // because otherwise the database will ignore it later when it's time to
      // apply all the changes that we've been saving up.
      _expected_version = version;
    }

    // Symbol key for the example illustrations below
    //
    // o: received change
    //    (we received it while we were still expecting an earlier change
    //
    // x: known lost change
    //    (we never received it, but we did receive a higher change)
    //
    // _: completely unknown change
    //    (we never received it, and we never received a higher change)
    //
    // ^: location of the current change

    const auto range_it = _ranges.lower_bound(Range{version, version});
    if (range_it == _ranges.end())
    {
      // x x o x x x o _ _ _ _
      //                     ^

      // This change version is higher than any of the versions that are missing

      // Since we have never received this change, and it has a greater version
      // than any of the missing entries, we must have never received any change
      // with a higher version. If either of those beliefs are false, then there
      // is a software bug somewhere.
      assert(!_changes.empty() && (_changes.rbegin())->first < version);

      if (nullifying)
      {
        // This is a nullifying change, so we can wipe out all earlier changes
        // and all earlier inconsistencies and start fresh.

        _changes.clear();
        _ranges.clear();
        _expected_version = version + 1;
        return nullptr;
      }
      else
      {
        // This is the highest inconsistent version number that we have seen so
        // far. We will create an inconsistency range between this and the
        // highest change value that we have received so far.

        const ItineraryVersion lower = _changes.rbegin()->first + 1;
        const ItineraryVersion upper = version - 1;

        if (modular(lower).less_than_or_equal(upper))
        {
          // Less than:
          // x x o x x x o _ _ _
          //                   ^

          // Equal to:
          // x x o x x x o _ _
          //                 ^

          _ranges.insert(Range{lower, upper});
        }
        else
        {
          // If lower is not less than or equal to upper, then the only way that
          // should occur is if lower is equal to the version of the incoming
          // change.

          // x x o x x x o _
          //               ^

          assert(lower == version);

          // If lower is equal to the new version, then we do not need to insert
          // any new range of inconsistency.
        }

        return std::make_unique<Ticket>(*this, callback);
      }
    }
    else
    {
      const ItineraryVersion lower = range_it->lower;
      const ItineraryVersion upper = range_it->upper;

      if (nullifying)
      {
        // Since this is a nullifying change, it eliminates all ranges of
        // inconsistencies that come before it.
        const auto hint_it =
            _ranges.erase(_ranges.begin(), ++RangesSet::iterator(range_it));

        if (version == upper)
        {
          // x x o x x x o x x o
          //           ^

          // The new change also eliminates the range of inconsistencies that it
          // belongs to, because it is the upper end of its range.

          if (_ranges.empty())
          {
            // x x o x x x o o o
            //           ^

            // All inconsistencies have been eliminated.
            _ready = true;
          }
        }
        else
        {
          // x x o x x x o x x o
          //         ^

          // The lesser ranges are eliminated completely, while the range that
          // this change is inside of will simply be reduced.
          _ranges.insert(hint_it, Range{version + 1, upper});
        }

        return std::make_unique<Ticket>(*this, callback);
      }
      else if (version == upper)
      {
        if (lower == version)
        {
          // x x o x o
          //       ^

          // The new version eliminates this unit-sized range of inconsistencies
          _ranges.erase(range_it);

          if (_ranges.empty())
          {
            // o o o x o o
            //       ^

            // The new version eliminates all outstanding inconsistencies for
            // this participant, so we are ready to start applying the changes
            // that have been accumulating.
            _ready = true;
          }
        }
        else
        {
          // x x o x x x o
          //           ^

          // The new version shrinks this range of inconsistencies by 1 from the
          // top.

          // The lower end of a range must be less than or equal to the upper
          // end of the range. We already confirmed that it is not equal, so it
          // must be less.
          assert(modular(lower).less_than(version));

          _ranges.insert(range_it, Range{lower, version-1});
          _ranges.erase(range_it);
        }

        return std::make_unique<Ticket>(*this, callback);
      }
      else
      {
        assert(modular(version).less_than(upper));
        if (lower == version)
        {
          // x x o x x x o
          //       ^

          // The new version shrinks this range of inconsistencies by 1 from the
          // bottom.
          const auto hint_it = _ranges.erase(range_it);
          _ranges.insert(hint_it, Range{version, upper});
        }
        else
        {
          // It should not be possible for the version to be less than the lower
          // bound of this range. If that were the case, then it should either
          // be a change that was already received (so we should have exited
          // this function already) or the call to ranges.lower_bound(version)
          // should have returned a different iterator.
          assert(modular(lower).less_than(version));

          // x x o x x x o
          //         ^

          // The new version is splitting this range of inconsistencies.
          const auto hint_it = _ranges.erase(range_it);
          _ranges.insert(hint_it, Range{lower, version-1});
          _ranges.insert(hint_it, Range{version+1, upper});
        }

        return std::make_unique<Ticket>(*this, callback);
      }
    }
  }
}

//==============================================================================
void InconsistencyTracker::_apply_changes()
{
  _ready = false;
  for (const auto& c : _changes)
    c.second();

  _changes.clear();
}

} // namespace schedule
} // namespace rmf_traffic
