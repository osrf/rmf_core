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

#ifndef RMF_TRAFFIC__SCHEDULE__QUERY_HPP
#define RMF_TRAFFIC__SCHEDULE__QUERY_HPP

#include <rmf_traffic/geometry/Space.hpp>

#include <rmf_traffic/detail/bidirectional_iterator.hpp>

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Query
{
public:

  class Spacetime
  {
  public:

    /// This enumerator determines what Spacetime mode the query will be in.
    enum class Mode
    {
      /// Request trajectories throughout all of space and time. This will still
      /// be constrained by the version field.
      All,

      /// Request trajectories in specific regions spacetime regions.
      Regions,
    };

    Mode get_mode();

    //==========================================================================
    /// This is a placeholder class in case we ever want to extend the features
    /// of the `All` mode.
    class All
    {
    public:

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    All query_all();

    //==========================================================================
    class Region
    {
    public:

      Region(
          std::string map,
          Time lower_bound,
          Time upper_bound,
          std::vector<geometry::Space> spaces);

      std::string get_map() const;
      Region& set_map(const std::string& map);

      Time get_lower_time_bound() const;
      Region& set_lower_time_bound(Time time);

      Time get_upper_time_bound() const;
      Region& set_upper_time_bound(Time time);

      class IterImpl;

      using iterator =
        rmf_traffic::detail::bidirectional_iterator<
          geometry::Space, IterImpl, Region>;

      using const_iterator =
        rmf_traffic::detail::bidirectional_iterator<
          const geometry::Space, IterImpl, Region>;

      void push_back(geometry::Space space);

      void pop_back();

      iterator erase(iterator it);

      iterator erase(iterator first, iterator last);

      iterator begin();

      const_iterator begin() const;

      const_iterator cbegin() const;

      iterator end();

      const_iterator end() const;

      const_iterator cend() const;

      std::size_t num_spaces() const;

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    class Regions
    {
    public:

      class IterImpl;
      using iterator =
        rmf_traffic::detail::bidirectional_iterator<
          Region, IterImpl, Regions>;

      using const_iterator =
        rmf_traffic::detail::bidirectional_iterator<
          const Region, IterImpl, Regions>;

      void push_back(Region region);

      void pop_back();

      iterator erase(iterator it);

      iterator erase(iterator first, iterator last);

      iterator begin();

      const_iterator begin() const;

      const_iterator cbegin() const;

      iterator end();

      const_iterator end() const;

      const_iterator cend() const;

      std::size_t size() const;

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    Regions& query_regions();

    Regions* regions();

    const Regions* regions() const;


    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  Spacetime& spacetime();

  const Spacetime& spacetime() const;

  class Versions
  {
  public:

    enum class Mode
    {

      All,

      After,
    };

    /// This is a placeholder class in case we ever want to extend the features
    /// of the `All` mode.
    class All
    {
    public:


      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    All& query_all();

    class After
    {
    public:

      std::size_t get_version() const;
      After& set_version(std::size_t version);

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    After& query_after();
    After* after();
    const After* after() const;

  };

  Versions& versions();

  const Versions& versions() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


/// Query for all Trajectories in a schedule database
Query query_everything();

/// Query for all Trajectories in a schedule database that were introduced
/// after a specified version of the schedule.
///
/// \param[in] after_version
///   Only query Trajectories that were added to the schedule after this
///   version number.
Query make_query(
    std::size_t after_version);

/// Query for all Trajectories that intersect with this set of spacetime
/// regions.
///
/// \param[in] regions
///   Only query Trajectories that intersect with the specified regions.
Query make_query(
    std::vector<Query::Spacetime::Region> regions);

/// Query for all Trajectories that were introduced after a specified version of
/// the schedule, and which intersect with this set of spacetime regions.
///
/// \param[in] after_version
///   Only query Trajectories that were added to the schedule after this
///   version number.
///
/// \param[in] regions
///   Only query Trajectories that intersect with the specified regions.
Query make_query(
    std::size_t after_version,
    std::vector<Query::Spacetime::Region> regions);

} // namespace schedule

namespace detail {

extern template class bidirectional_iterator<
    schedule::Query::Spacetime::Region,
    schedule::Query::Spacetime::Regions::IterImpl,
    schedule::Query::Spacetime::Regions
>;

extern template class bidirectional_iterator<
    const schedule::Query::Spacetime::Region,
    schedule::Query::Spacetime::Regions::IterImpl,
    schedule::Query::Spacetime::Regions
>;

extern template class bidirectional_iterator<
    geometry::Space,
    schedule::Query::Spacetime::Region::IterImpl,
    schedule::Query::Spacetime::Region
>;

extern template class bidirectional_iterator<
    const geometry::Space,
    schedule::Query::Spacetime::Region::IterImpl,
    schedule::Query::Spacetime::Region
>;

} // namespace detail
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__QUERY_HPP
