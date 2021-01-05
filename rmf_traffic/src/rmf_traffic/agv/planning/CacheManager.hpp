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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__CACHEMANAGER_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__CACHEMANAGER_HPP

#include <optional>
#include <memory>
#include <mutex>
#include <functional>
#include <unordered_map>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
template <typename StorageArg>
class Generator
{
public:

  using Storage = StorageArg;
  using Key = typename Storage::key_type;
  using Value = typename Storage::mapped_type;

  virtual Value generate(
      const Key& key,
      const Storage& old_items,
      Storage& new_items) const = 0;

  virtual ~Generator() = default;
};

//==============================================================================
template <typename GeneratorArg>
class Factory
{
public:

  using Generator = GeneratorArg;
  using ConstGeneratorPtr = std::shared_ptr<const Generator>;

  // TODO(MXG): Should we take orientation into account here? It could matter
  // for cases where the goal's orientation is constrained.
  virtual ConstGeneratorPtr make(const std::size_t goal) const = 0;

  virtual ~Factory() = default;
};

//==============================================================================
template <typename GeneratorArg>
class Upstream
{
public:

  using Generator = GeneratorArg;
  using Storage = typename Generator::Storage;

  Upstream(
    std::function<Storage()> storage_initializer,
    std::shared_ptr<const Generator> generator_)
  : storage(std::make_shared<Storage>(storage_initializer())),
    generator(std::move(generator_))
  {
    // Do nothing
  }

  std::shared_ptr<Storage> storage;
  const std::shared_ptr<const Generator> generator;
};

//==============================================================================
template <typename> class CacheManager;

//==============================================================================
template <typename GeneratorArg>
class Cache
{
public:

  using Generator = GeneratorArg;
  using Storage = typename Generator::Storage;
  using Self = Cache<Generator>;
  using Upstream_type = Upstream<Generator>;

  Cache(
    std::shared_ptr<const Upstream_type> upstream,
    std::shared_ptr<const CacheManager<Self>> manager,
    std::function<Storage()> storage_initializer);


  using Key = typename Storage::key_type;
  using Value = typename Storage::mapped_type;

  Value get(const Key& key) const;

  ~Cache();

private:
  std::shared_ptr<const Upstream_type> _upstream;
  std::shared_ptr<const CacheManager<Self>> _manager;
  std::function<Storage()> _storage_initializer;
  mutable Storage _all_items;
  mutable Storage _new_items;
};

//==============================================================================
template <typename CacheArg>
class CacheManager : public std::enable_shared_from_this<CacheManager<CacheArg>>
{
public:

  using Storage = typename CacheArg::Storage;
  using Generator = typename CacheArg::Generator;
  using Upstream_type = Upstream<Generator>;
  using Self = CacheManager<CacheArg>;

  template <typename... Args>
  static std::shared_ptr<const Self> make(Args&&... args)
  {
    return std::shared_ptr<Self>(new Self(std::forward<Args>(args)...));
  }

  CacheArg get() const;

private:

  CacheManager(
    std::shared_ptr<const Generator> generator,
    std::function<Storage()> storage_initializer = [](){ return Storage(); });

  void _update(Storage new_items) const;

  std::unique_lock<std::mutex> _lock() const;

  template <typename G> friend class Cache;
  std::shared_ptr<Upstream_type> _upstream;
  const std::function<Storage()> _storage_initializer;
  mutable std::mutex _update_mutex;
};

//==============================================================================
template<typename T>
using CacheManagerPtr = std::shared_ptr<const CacheManager<Cache<T>>>;

//==============================================================================
template <typename GeneratorFactoryArg>
class CacheManagerMap
{
public:

  using GeneratorFactory = GeneratorFactoryArg;
  using Generator = typename GeneratorFactory::Generator;
  using Cache_type = Cache<Generator>;
  using CacheManager_type = CacheManager<Cache_type>;
  using CacheManagerPtr = std::shared_ptr<const CacheManager_type>;
  using Storage = typename Cache_type::Storage;

  CacheManagerMap(
    std::shared_ptr<const GeneratorFactory> factory,
    std::function<Storage()> storage_initializer = [](){ return Storage() ; });

  CacheManagerPtr get(std::size_t goal_index) const;

private:
  // NOTE(MXG): We take some significant liberties with mutability here because
  // this cache manager is always logically const, even as its physical state is
  // changing significantly. Besides memoizing the results of previous
  // computations, the cache manager does not actually have any internal state.
  mutable std::unordered_map<std::size_t, CacheManagerPtr> _managers;
  mutable std::mutex _map_mutex;
  const std::shared_ptr<const GeneratorFactory> _generator_factory;
  const std::function<Storage()> _storage_initializer;
};

//==============================================================================
template <typename GeneratorArg>
Cache<GeneratorArg>::Cache(std::shared_ptr<const Upstream_type> upstream,
  std::shared_ptr<const CacheManager<Self>> manager,
  std::function<Storage()> storage_initializer)
: _upstream(std::move(upstream)),
  _manager(std::move(manager)),
  _storage_initializer(std::move(storage_initializer)),
  _all_items(*_upstream->storage),
  _new_items(_storage_initializer())
{
  // Do nothing
}

//==============================================================================
template <typename GeneratorArg>
auto Cache<GeneratorArg>::get(const Key& key) const -> Value
{
  const auto it = _all_items.find(key);
  if (it != _all_items.end())
    return it->second;

  Storage new_items = _storage_initializer();
  auto result = _upstream->generator->generate(key, _all_items, new_items);

  for (const auto& item : new_items)
  {
    _all_items.insert(item);
    _new_items.insert(item);
  }

  return result;
}

//==============================================================================
template <typename GeneratorArg>
Cache<GeneratorArg>::~Cache()
{
  if (!_new_items.empty())
    _manager->_update(std::move(_new_items));
}

//==============================================================================
template <typename CacheArg>
CacheManager<CacheArg>::CacheManager(
  std::shared_ptr<const Generator> generator,
  std::function<Storage()> storage_initializer)
: _upstream(
    std::make_shared<Upstream_type>(storage_initializer, std::move(generator))),
  _storage_initializer(std::move(storage_initializer))
{
  // Do nothing
}

//==============================================================================
template <typename CacheArg>
CacheArg CacheManager<CacheArg>::get() const
{
  auto lock = _lock();
  return CacheArg{_upstream, this->shared_from_this(), _storage_initializer};
}

//==============================================================================
template <typename CacheArg>
void CacheManager<CacheArg>::_update(Storage new_items) const
{
  auto lock = _lock();
  auto new_storage = std::make_shared<Storage>(*_upstream->storage);
  for (auto&& item : new_items)
    (*new_storage)[item.first] = std::move(item.second);

  _upstream->storage = std::move(new_storage);
}

//==============================================================================
template <typename CacheArg>
std::unique_lock<std::mutex> CacheManager<CacheArg>::_lock() const
{
  std::unique_lock<std::mutex> lock(_update_mutex, std::defer_lock);
  while (!lock.try_lock())
  {
    // Intentionally busy-wait to get the lock as soon as possible. Other lock
    // holders should only be holding the lock very briefly.
  }

  return lock;
}

//==============================================================================
template <typename CacheArg>
CacheManagerMap<CacheArg>::CacheManagerMap(
  std::shared_ptr<const GeneratorFactory> factory,
  std::function<Storage()> storage_initializer)
: _generator_factory(std::move(factory)),
  _storage_initializer(std::move(storage_initializer))
{
  // Do nothing
}

//==============================================================================
template <typename CacheArg>
auto CacheManagerMap<CacheArg>::get(std::size_t goal_index) const
-> CacheManagerPtr
{
  std::unique_lock<std::mutex> lock(_map_mutex, std::defer_lock);
  while (!lock.try_lock())
  {
    // Intentionally busy-wait to get the lock as soon as possible. Other lock
    // holders should only be holding the lock very briefly.
  }

  const auto it = _managers.insert({goal_index, nullptr});
  auto& manager = it.first->second;
  if (manager == nullptr)
  {
    manager = CacheManager_type::make(
          _generator_factory->make(goal_index),
          _storage_initializer);
  }

  return manager;
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__CACHEMANAGER_HPP
