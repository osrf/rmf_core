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

  Upstream(std::shared_ptr<const Generator> generator_)
  : storage(std::make_shared<Storage>()),
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
  using Upstream = Upstream<Generator>;

  Cache(
    std::shared_ptr<const Upstream> upstream,
    std::shared_ptr<const CacheManager<Self>> manager);


  using Key = typename Storage::key_type;
  using Value = typename Storage::mapped_type;

  Value get(const Key& key);

  ~Cache();

private:
  std::shared_ptr<const Upstream> _upstream;
  std::shared_ptr<const CacheManager<Self>> _manager;
  Storage _all_items;
  Storage _new_items;
};

//==============================================================================
template <typename CacheArg>
class CacheManager : public std::enable_shared_from_this<CacheManager<CacheArg>>
{
public:

  using Storage = typename CacheArg::Storage;
  using Generator = typename CacheArg::Generator;
  using Upstream = Upstream<Generator>;

  CacheManager(std::shared_ptr<const Generator> generator);

  CacheArg get() const;

private:

  void update(Storage new_items) const;

  template <typename G> friend class Cache;
  std::shared_ptr<Upstream> _upstream;
  mutable std::mutex _update_mutex;
};

//==============================================================================
template <typename GeneratorFactoryArg>
class CacheManagerMap
{
public:

  using GeneratorFactory = GeneratorFactoryArg;
  using Generator = typename GeneratorFactory::Generator;
  using Cache = Cache<Generator>;
  using CacheManager = CacheManager<Cache>;
  using CacheManagerPtr = std::shared_ptr<CacheManager>;

  CacheManagerMap(std::shared_ptr<const GeneratorFactory> factory);

  CacheManagerPtr get(std::size_t goal_index) const;

private:
  // NOTE(MXG): We take some significant liberties with mutability here because
  // this cache manager is always logically const, even as its physical state is
  // changing significantly. Besides memoizing the results of previous
  // computations, the cache manager does not actually have any internal state.
  mutable std::unordered_map<std::size_t, CacheManagerPtr> _managers;
  mutable std::mutex _map_mutex;
  const std::shared_ptr<const GeneratorFactory> _generator_factory;
};

//==============================================================================
template <typename GeneratorArg>
Cache<GeneratorArg>::Cache(std::shared_ptr<const Upstream> upstream,
  std::shared_ptr<const CacheManager<Self> > manager)
: _upstream(std::move(upstream)),
  _manager(std::move(manager)),
  _all_items(*_upstream->storage)
{
  // Do nothing
}

//==============================================================================
template <typename GeneratorArg>
auto Cache<GeneratorArg>::get(const Key& key) -> Value
{
  const auto it = _all_items.find(key);
  if (it != _all_items.end())
    return it->second;

  Storage new_items;
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
    _manager->update(std::move(_new_items));
}

//==============================================================================
template <typename CacheArg>
CacheManager<CacheArg>::CacheManager(
    std::shared_ptr<const Generator> generator)
: _upstream(std::make_shared<Upstream>(std::move(generator)))
{
  // Do nothing
}

//==============================================================================
template <typename CacheArg>
CacheArg CacheManager<CacheArg>::get() const
{
  std::lock_guard<std::mutex> lock(_update_mutex);
  return CacheArg{_upstream, this->shared_from_this()};
}

//==============================================================================
template <typename CacheArg>
void CacheManager<CacheArg>::update(Storage new_items) const
{
  std::lock_guard<std::mutex> lock(_update_mutex);
  auto new_storage = std::make_shared<Storage>(*_upstream->storage);
  for (auto&& item : new_items)
    (*new_storage)[item.first] = std::move(item.second);

  _upstream->storage = std::move(new_storage);
}

//==============================================================================
template <typename CacheArg>
CacheManagerMap<CacheArg>::CacheManagerMap(
  std::shared_ptr<const GeneratorFactory> factory)
: _generator_factory(std::move(factory))
{
  // Do nothing
}

//==============================================================================
template <typename CacheArg>
auto CacheManagerMap<CacheArg>::get(std::size_t goal_index) const
-> CacheManagerPtr
{
  std::lock_guard<std::mutex> lock(_map_mutex);
  const auto it = _managers.insert({goal_index, nullptr});
  auto& manager = it.first->second;
  if (manager == nullptr)
  {
    manager = std::make_shared<CacheManager>(
          _generator_factory->make(goal_index));
  }

  return manager;
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__CACHEMANAGER_HPP
