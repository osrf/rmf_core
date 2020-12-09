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

#ifndef SRC__RMF_TRAFFIC__TRAJECTORYINTERNAL_HPP
#define SRC__RMF_TRAFFIC__TRAJECTORYINTERNAL_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <list>
#include <map>

namespace rmf_traffic {
namespace internal {

//==============================================================================
struct WaypointElement;
using WaypointList = std::list<WaypointElement>;

//==============================================================================
template <typename Key, typename Value>
class TemplateOrderMap
{
public:

  struct Element
  {
    Key key;
    Value value;

    template <typename... Args>
    static Element make(const Key& key, Args&&... args)
    {
      return Element{key, Value{std::forward<Args>(args)...}};
    }
  };

  struct Comparison
  {
    bool operator()(const Element& element, const Key& key)
    {
      return element.key < key;
    }
  };

  using Storage = std::vector<Element>;
  using iterator = typename Storage::iterator;
  using const_iterator = typename Storage::const_iterator;

  iterator begin()
  {
    return _storage.begin();
  }

  const_iterator begin() const
  {
    return _storage.begin();
  }

  iterator end()
  {
    return _storage.end();
  }

  const_iterator end() const
  {
    return _storage.end();
  }

  iterator lower_bound(const Key& k)
  {
    return std::lower_bound(_storage.begin(), _storage.end(), k, Comparison());
  }

  const_iterator lower_bound(const Key& k) const
  {
    return std::lower_bound(_storage.begin(), _storage.end(), k, Comparison());
  }

  template <typename... Args>
  iterator emplace_hint(iterator hint, const Key& key, Args&&... args)
  {
    if (_storage.empty())
    {
      _storage.push_back(Element::make(key, std::forward<Args>(args)...));
      return _storage.begin();
    }

    if (hint == _storage.end())
    {
      if ( (--const_iterator(hint))->key < key)
      {
        return _storage.emplace(
              _storage.end(), Element::make(key, std::forward<Args>(args)...));
      }

      // If the above test failed, then we were given a bad hint. Let's just use
      // std::lower_bound to find the right place for this insertion.
      const auto new_hint = lower_bound(key);

      if (new_hint->key == key)
        return new_hint;

      return _storage.emplace(
            new_hint, Element::make(key, std::forward<Args>(args)...));
    }

    if (hint->key == key)
      return hint;

    if (key < hint->key)
    {
      if (hint == _storage.begin())
      {
        return _storage.emplace(
              hint, Element::make(key, std::forward<Args>(args)...));
      }
      else if ( (--const_iterator(hint))->key < key)
      {
        return _storage.emplace(
              hint, Element::make(key, std::forward<Args>(args)...));
      }

      // If the above tests failed, then we don't have a perfect hint, but at
      // least we know that it's on the correct side of hint, so we'll use that
      // for the upper bound on the search.
      const auto new_hint =
          std::lower_bound(_storage.begin(), hint, key, Comparison());

      if (new_hint->key == key)
        return new_hint;

      return _storage.emplace(
            new_hint, Element::make(key, std::forward<Args>(args)...));
    }

    // If the above tests failed, then we have been given a bad hint, but at
    // least we know hint is on the lower end of the bound.
    const auto new_hint =
        std::lower_bound(hint, _storage.end(), key, Comparison());

    if (new_hint->key == key)
      return new_hint;

    return _storage.emplace(
          new_hint, Element::make(key, std::forward<Args>(args)...));
  }

  iterator erase(const Key& key)
  {
    const auto it = lower_bound(key);
    if (it->key == key)
      return _storage.erase(it);

    return _storage.end();
  }

  iterator erase(iterator begin, iterator end)
  {
    return _storage.erase(begin, end);
  }

  iterator erase(iterator it)
  {
    return _storage.erase(it);
  }

  iterator erase(const_iterator it)
  {
    return _storage.erase(it);
  }

  iterator find(const Key& key)
  {
    const auto it = lower_bound(key);
    if (it->key == key)
      return it;

    return _storage.end();
  }

  const_iterator find(const Key& key) const
  {
    const auto it = lower_bound(key);
    if (it->key == key)
      return it;

    return _storage.end();
  }

  Element& operator[](const std::size_t index)
  {
    return _storage[index];
  }

  const Element& operator[](const std::size_t index) const
  {
    return _storage[index];
  }

  Element& at(const std::size_t index)
  {
    return _storage.at(index);
  }

  const Element& at(const std::size_t index) const
  {
    return _storage.at(index);
  }

  std::size_t size() const
  {
    return _storage.size();
  }

  bool empty() const
  {
    return _storage.empty();
  }

private:
  Storage _storage;
};

using OrderMap = TemplateOrderMap<Time, WaypointList::iterator>;

//==============================================================================
struct WaypointElement
{
  struct Data
  {
    Time time;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
  };

  Data data;

  // We store a Trajectory::Waypoint in this struct so that we can always safely
  // return a reference to a Trajectory::Waypoint object. As long as this
  // WaypointData is alive, any Trajectory::Waypoint reference that refers to it
  // will remain valid.
  std::unique_ptr<Trajectory::Waypoint> myself;

  WaypointElement(Data input_data)
  : data(std::move(input_data))
  {
    // Do nothing
  }

  WaypointElement(const WaypointElement& other)
  : data(other.data)
  {
    // Do nothing
  }

  WaypointElement& operator=(const WaypointElement& other)
  {
    data = other.data;
    return *this;
  }

  WaypointElement(WaypointElement&&) = default;
  WaypointElement& operator=(WaypointElement&&) = default;
};

//==============================================================================
WaypointList::const_iterator get_raw_iterator(
  const Trajectory::const_iterator& iterator);

} // namespace internal
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__TRAJECTORYINTERNAL_HPP
