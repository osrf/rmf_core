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

#include <vector>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A class to define a query into a schedule database.
class Query
{
public:

  template<typename E, typename I, typename F>
  using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

  /// A class to describe spacetime filters for a schedule Query.
  class Spacetime
  {
  public:

    using Space = geometry::Space;

    /// This enumerator determines what Spacetime mode the query will be in.
    enum class Mode : uint16_t
    {
      /// Request trajectories throughout all of space and time. This will still
      /// be constrained by the version field.
      All,

      /// Request trajectories in specific regions spacetime regions.
      Regions,

      // TODO(MXG): Add a Timespan Mode
    };

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

    //==========================================================================
    /// A class to describe a region within spacetime.
    ///
    /// This specifies the map whose coordinates should be used, a lower and
    /// upper bound to define a time range, and a set of geometry::Space objects
    /// to define regions with space.
    ///
    /// For the geometry::Space objects, this class acts like an STL container
    /// and provides an iterator interface to specify, access, and modify them.
    class Region
    {
    public:

      /// Construct a region given the parameters.
      ///
      /// \param[in] map
      ///   The map whose coordinates will be used to define the regions in
      ///   space.
      ///
      /// \param[in] lower_bound
      ///   The lower bound for the time range.
      ///
      /// \param[in] upper_bound
      ///   The upper bound for the time range.
      ///
      /// \param[in] spaces
      ///   A vector of geometry::Space objects to define the desired regions
      ///   in space.
      Region(
          std::string map,
          Time lower_bound,
          Time upper_bound,
          std::vector<Space> spaces);

      /// Construct a region with no time constraints.
      ///
      /// \param[in] map
      ///   The map whose coordinates will be used to define the regions in
      ///   space.
      ///
      /// \param[in] spaces
      ///   A vector of geometry::Space objects to define the desired regions
      ///   in space.
      Region(
          std::string map,
          std::vector<Space> spaces);

      /// Get the name of the map that this Spacetime refers to.
      const std::string& get_map() const;

      /// Set the name of the map that this Spacetime refers to.
      Region& set_map(std::string map);

      /// Get the lower bound for the time range.
      ///
      /// If there is no lower bound for the time range, then this returns
      /// a nullptr.
      const Time* get_lower_time_bound() const;

      /// Set the lower bound for the time range.
      Region& set_lower_time_bound(Time time);

      /// Remove the lower bound for the time range.
      Region& remove_lower_time_bound();

      /// Get the upper bound for the time range.
      ///
      /// If there is no upper bound for the time range, then this returns
      /// a nullptr.
      const Time* get_upper_time_bound() const;

      /// Set the upper bound for the time range.
      Region& set_upper_time_bound(Time time);

      /// Remove the upper bound for the time range.
      Region& remove_upper_time_bound();

      class IterImpl;
      using iterator = base_iterator<Space, IterImpl, Region>;
      using const_iterator = base_iterator<const Space, IterImpl, Region>;

      /// Add a region of space.
      void push_back(Space space);

      /// Remove the last region of space that was added.
      void pop_back();

      /// Erase a specific region of space based on its iterator.
      iterator erase(iterator it);

      /// Erase a specific sets of regions of space based on their iterators.
      iterator erase(iterator first, iterator last);

      /// Get the beginning iterator for the regions of space.
      iterator begin();

      /// const-qualified begin()
      const_iterator begin() const;

      /// Explicitly const-qualified alternative for begin()
      const_iterator cbegin() const;

      /// Get the one-past-the-end iterator for the regions of space.
      iterator end();

      /// const-qualified end()
      const_iterator end() const;

      /// Explicitly const-qualified alternative for end()
      const_iterator cend() const;

      /// Get the number of Space regions in this Spacetime region.
      std::size_t num_spaces() const;

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// A container class for Spacetime::Region instances. This acts like an
    /// STL container for Spacetime::Region elements.
    class Regions
    {
    public:

      class IterImpl;
      using iterator = base_iterator<Region, IterImpl, Regions>;
      using const_iterator = base_iterator<const Region, IterImpl, Regions>;

      /// Instantiate a container of Region instances using a vector.
      Regions(std::vector<Region> regions = {});

      /// Add a Region to this container.
      void push_back(Region region);

      /// Remove the last Region that was added to this container.
      void pop_back();

      /// Erase a Region based on its iterator.
      iterator erase(iterator it);

      /// Erase a range of Regions based on their iterators.
      iterator erase(iterator first, iterator last);

      /// Get the beginning iterator of this container.
      iterator begin();

      /// const-qualified begin()
      const_iterator begin() const;

      /// Explicitly const-qualified alternative to begin()
      const_iterator cbegin() const;

      /// Get the one-past-the-end iterator of this container.
      iterator end();

      /// const-qualified end()
      const_iterator end() const;

      /// Explicitly const-qualified alternative to end()
      const_iterator cend() const;

      /// Get the number of Spacetime Region elements in this container.
      std::size_t size() const;

      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// Default constructor, uses All mode.
    Spacetime();

    /// Regions mode constructor
    ///
    /// \param[in] regions
    ///   The regions to use
    Spacetime(std::vector<Region> regions);

    /// Get the current Spacetime Mode of this query.
    Mode get_mode() const;

    /// Set the mode of this Spacetime to query for All Trajectories throughout
    /// Spacetime.
    All& query_all();

    /// Set the mode of this Spacetime to query for specific Regions.
    ///
    /// \param[in] regions
    ///   Specify the regions of Spacetime to use.
    Regions& query_regions(std::vector<Region> regions = {});

    /// Get the Regions of Spacetime to use for this Query. If this Spacetime is
    /// not in Regions mode, then this will return a nullptr.
    Regions* regions();

    /// const-qualified regions()
    const Regions* regions() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A class to describe a filter on what version changes to query from a
  /// schedule.
  class Versions
  {
  public:

    /// The mode for how to filter versions in a schedule database query.
    enum class Mode : uint16_t
    {
      /// Get everything, regardless of version.
      All,

      /// Get every version after the specified one.
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

    /// The interface for the Versions::After mode.
    class After
    {
    public:

      /// Constructor.
      After(std::size_t version);

      /// Get the specified version. The Query will only return Trajectories
      /// which were introduced after this version of the schedule.
      std::size_t get_version() const;

      /// Set the version.
      ///
      /// \param version
      ///   The Query will only return Trajectories which were introduced after
      ///   this version of the schedule.
      After& set_version(std::size_t version);

      class Implementation;
    private:
      /// Default constructor. This is not accessible to users because it leaves
      /// the After instance null.
      After();
      friend class Versions;
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// Default constructor, uses All mode.
    Versions();

    /// Constructor to use After mode.
    ///
    /// \param[in] version
    ///   The Query will only return Trajectories which were introduced after
    ///   this version of the schedule.
    Versions(std::size_t version);

    /// Get the current Versions mode of this query.
    Mode get_mode() const;

    /// Set the mode of this Versions interface to query for All Trajectories
    /// regardless of version.
    All& query_all();

    /// Set the mode of this Versions interface to query for only Trajectories
    /// that changed after the given version.
    ///
    /// \param[in] version
    ///   The Query will only return Trajectories which were introduced after
    ///   this version of the schedule.
    After& query_after(std::size_t version);

    /// Get the Versions After interface to use for this Query. If this Versions
    /// is not in the After mode, then this will return a nullptr.
    After* after();

    /// const-qualified after()
    const After* after() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Get the Spacetime component of this Query.
  Spacetime& spacetime();

  /// const-qualified spacetime()
  const Spacetime& spacetime() const;

  /// Get the Versions component of this Query.
  Versions& versions();

  /// const-qualified versions()
  const Versions& versions() const;

  class Implementation;
private:
  /// \internal The default constructor is private because users are expected
  /// to construct a Query using one of the functions below.
  Query();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Query for all Trajectories in a schedule database
Query query_everything();

//==============================================================================
/// Query for all Trajectories in a schedule database that were introduced
/// after a specified version of the schedule.
///
/// \param[in] after_version
///   Only query Trajectories that were added to the schedule after this
///   version number.
Query make_query(
    std::size_t after_version);

//==============================================================================
/// Query for all Trajectories that intersect with this set of spacetime
/// regions.
///
/// \param[in] regions
///   Only query Trajectories that intersect with the specified regions.
Query make_query(
    std::vector<Query::Spacetime::Region> regions);

//==============================================================================
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
