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

#ifndef RMF_TRAFFIC__TRAJECTORY_HPP
#define RMF_TRAFFIC__TRAJECTORY_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <vector>

namespace rmf_traffic {

//==============================================================================
namespace internal {
/// \internal We declare this private PIMPL class outside of the
/// Trajectory::base_iterator class so that it does not need to be templated.
class TrajectoryIteratorImplementation;
} // namespace internal

//==============================================================================
class Trajectory
{
public:

  class Waypoint
  {
  public:

    /// Get the intended physical location of the robot at the end of this
    /// Trajectory Waypoint.
    ///
    /// This is a 2D homogeneous position. The first two values in the vector
    /// are x and y coordinates, while the third is rotation about the z-axis.
    Eigen::Vector3d position() const;

    /// Set the intended physical location of the robot at the end of this
    /// Trajectory Waypoint.
    ///
    /// This is a 2D homogeneous position. The first two values in the vector
    /// are x and y coordinates, while the third is rotation about the z-axis.
    ///
    /// \param[in] new_position
    ///   The new position for this Trajectory Waypoint.
    Waypoint& position(Eigen::Vector3d new_position);

    /// Get the intended velocity of the robot at the end of this Trajectory
    /// Waypoint.
    ///
    /// This is a 2D homogeneous position. The first two values in the vector
    /// are x and y velocities, while the third is rotational velocity about the
    /// z-axis.
    Eigen::Vector3d velocity() const;

    /// Set the intended velocity of the robot at the end of this Trajectory
    /// Waypoint.
    ///
    /// This is a 2D homogeneous position. The first two values in the vector
    /// are x and y coordinates, while the third is rotation about the z-axis.
    ///
    /// \param[in] new_velocity
    ///   The new velocity at this Trajectory Waypoint.
    Waypoint& velocity(Eigen::Vector3d new_velocity);

    /// Get the time that the trajectory will reach this Waypoint.
    Time time() const;

    /// Change the timing of this Trajectory Waypoint. Note that this function
    /// will only affect this waypoint, and may cause this waypoint to be
    /// reordered within the Trajectory.
    ///
    /// To change the timing for this waypoint while preserving the relative
    /// times of all subsequent Trajectory Waypoints, use adjust_times()
    /// instead.
    ///
    /// \warning If you change the time value of this Waypoint such that it
    /// falls directly on another Waypoint's time, you will get a
    /// std::invalid_argument exception, because discontinuous jumps are not
    /// supported, and indicate a significant mishandling of trajectory data,
    /// which is most likely a serious bug that should be remedied.
    ///
    /// \note If this Waypoint's time crosses over another Waypoint's time, that
    /// signficantly changes the topology of the Trajectory, because it will
    /// change the order in which the positions are traversed.
    ///
    /// \param[in] new_time
    ///   The new timing for this Trajectory Waypoint.
    ///
    /// \sa adjust_times(Time new_time)
    Waypoint& change_time(Time new_time);

    /// Adjust the timing of this waypoint and all subsequent waypoints by the
    /// given duration. This is guaranteed to maintain the ordering of the
    /// Trajectory Waypoints, and is more efficient than changing all the times
    /// directly.
    ///
    /// \warning If a negative delta_t is given, it must not cause this
    /// Waypoint's time to be less than or equal to the time of its preceding
    /// Waypoint, or else a std::invalid_argument exception will be thrown.
    ///
    /// \param[in] delta_t
    ///   How much to change the timing of this waypoint and all later
    ///   waypoints. If negative, it must not cross over the time of the
    ///   previous waypoint, or else a std::invalid_argument will be thrown.
    ///
    /// \sa change_time(Time new_time)
    void adjust_times(Duration delta_t);

    class Implementation;
  private:

    /// \internal Private constructor. Use Trajectory::insert() to create
    /// a new Trajectory Waypoint.
    Waypoint();
    Waypoint(const Waypoint&) = delete;
    Waypoint(Waypoint&&) = default;
    Waypoint& operator=(const Waypoint&) = delete;
    Waypoint& operator=(Waypoint&&) = default;
    friend class Trajectory;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  // These classes allow users to traverse the contents of the Trajectory.
  // The trajectory operates much like a typical C++ container, but only for
  // Trajectory::Waypoint information.
  template<typename> class base_iterator;
  using iterator = base_iterator<Waypoint>;
  using const_iterator = base_iterator<const Waypoint>;

  /// Create an empty Trajectory
  Trajectory();

  // Copy construction/assignment
  Trajectory(const Trajectory& other);
  Trajectory& operator=(const Trajectory& other);

  /// \warning After using the move constructor or move assignment operator,
  /// the Trajectory that was moved from will be unusable until a fresh
  /// Trajectory instance is assigned to it (using either the copy or move
  /// constructor). Attempting to use a Trajectory that was moved from will
  /// result in a segfault if you do not assign it a new instance.
  Trajectory(Trajectory&&) = default;
  Trajectory& operator=(Trajectory&&) = default;

  /// Contains two fields:
  /// * iterator it:   contains the iterator for the Waypoint that ends at the
  ///                  given time
  /// * bool inserted: true if the Waypoint was inserted, false if a Waypoint
  ///                  with the exact same time already existed
  struct InsertionResult;

  /// Add a Waypoint to this Trajectory.
  ///
  /// The Waypoint will be inserted into the Trajectory according to its time,
  /// ensuring correct ordering of all Waypoints.
  InsertionResult insert(
    Time time,
    Eigen::Vector3d position,
    Eigen::Vector3d velocity);

  /// Insert a copy of another Trajectory's Waypoint into this one.
  InsertionResult insert(const Waypoint& other);

  // TODO(MXG): Consider an insert() function that accepts a range of iterators
  // from another Trajectory instance.

  /// Find the Waypoint of this Trajectory that comes after or exactly on the
  /// given time.
  ///
  /// \note This will return Trajectory::end() if the time is before the
  /// Trajectory starts or after the Trajectory finishes.
  ///
  /// \param[in] time
  ///   The time of interest.
  ///
  /// \return an iterator to the Waypoint that is active during the given time,
  /// or Trajectory::end() if the time falls outside the range of the Trajectory
  iterator find(Time time);

  /// const-qualified version of find()
  const_iterator find(Time time) const;

  /// Get a reference to the Waypoint at the specified index. No bounds checking
  /// is performed, so there will be undefined behavior if the index is out of
  /// bounds.
  Waypoint& operator[](std::size_t index);

  /// Const-qualified index operator
  const Waypoint& operator[](std::size_t index) const;

  /// Get a reference to the Waypoint at the specified index. Bound checking
  /// will be performed, and an exception will be thrown if index is out of
  /// bounds.
  Waypoint& at(std::size_t index);

  /// Const-qualified at()
  const Waypoint& at(std::size_t index) const;

  /// Get the first waypoint of this Trajectory that occurs at a time greater
  /// than or equal to the specified time. This is effectively the same as
  /// find(Time), except it will return Trajectory::begin() if the time comes
  /// before the start of the Trajectory.
  ///
  /// \param[in] time
  ///   The lower bound on the time of interest.
  ///
  /// \return an iterator to the first Waypoint that occurs at a time on or
  /// after the given time, or Trajectory::end() if the time is after the end
  /// of the Trajectory.
  iterator lower_bound(Time time);

  /// const-qualified version of lower_bound()
  const_iterator lower_bound(Time time) const;

  /// Erase the specified waypoint.
  ///
  /// \return an iterator following the last removed element
  iterator erase(iterator waypoint);

  /// Erase the range of elements: [first, last).
  ///
  /// \note The `last` element is not included in the range.
  ///
  /// \return an iterator following the last removed element
  iterator erase(iterator first, iterator last);

  /// Returns an iterator to the fist Waypoint of the Trajectory.
  ///
  /// If the Trajectory is empty, the returned iterator will be equal to end().
  iterator begin();

  /// const-qualified version of begin()
  const_iterator begin() const;

  /// Explicitly call the const-qualified version of begin()
  const_iterator cbegin() const;

  /// Returns an iterator to the element following the last Waypoint of the
  /// Trajectory. This iterator acts as a placeholder; attempting to dereference
  /// it results in undefined behavior.
  ///
  /// \note In compliance with C++ standards, this is really a one-past-the-end
  /// iterator and must not be dereferenced. It should only be used to identify
  /// when an iteration must end. See:
  /// https://en.cppreference.com/w/cpp/container/list/end
  iterator end();

  /// const-qualified version of end()
  const_iterator end() const;

  /// Explicitly call the const-qualified version of end()
  const_iterator cend() const;

  /// Get a mutable reference to the first Waypoint in this Trajectory.
  ///
  /// \warning Calling this function on an empty trajectory is undefined.
  Waypoint& front();

  /// Get a const reference to the first Waypoint in this Trajectory.
  ///
  /// \warning Calling this function on an empty trajectory is undefined.
  const Waypoint& front() const;

  /// Get a mutable reference to the last Waypoint in this Trajectory.
  ///
  /// \warning Calling this function on an empty trajectory is undefined.
  Waypoint& back();

  /// Get a const reference to the last Waypoint in this Trajectory.
  ///
  /// \warning Calling this function on an empty trajectory is undefined.
  const Waypoint& back() const;

  /// Get the start time, if available. This will return a nullptr if the
  /// Trajectory is empty.
  const Time* start_time() const;

  /// Get the finish time of the Trajectory, if available. This will return a
  /// nullptr if the Trajectory is empty.
  const Time* finish_time() const;

  /// Get the duration of the Trajectory. This will be 0 if the Trajectory is
  /// empty or if it has only one Waypoint.
  Duration duration() const;

  /// Get the number of Waypoints in the Trajectory. To be used in conflict
  /// detection, the Trajectory must have a size of at least 2.
  std::size_t size() const;

  /// Return true if the trajectory has no waypoints, false otherwise.
  bool empty() const;

  /// \internal Used internally by unit and integration tests so we can test
  /// private imeplementation details.
  class Debug;

private:
  friend class internal::TrajectoryIteratorImplementation;
  class Implementation;
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;

};

//==============================================================================
template<typename W>
class Trajectory::base_iterator
{
public:

  /// Dereference operator
  W& operator*() const;

  /// Drill-down operator
  W* operator->() const;

  /// Pre-increment operator: ++it
  ///
  /// \note This is more efficient than the post-increment operator.
  ///
  /// \return a reference to the iterator that was operated on
  base_iterator& operator++();

  /// Pre-decrement operator: --it
  ///
  /// \note This is more efficient than the post-decrement operator
  ///
  /// \return a reference to the iterator that was operated on
  base_iterator& operator--();

  /// Post-increment operator: it++
  ///
  /// \return a copy of the iterator before it was incremented
  base_iterator operator++(int);

  /// Post-decrement operator: it--
  ///
  /// \return a copy of the iterator before it was decremented
  base_iterator operator--(int);


  // TODO(MXG): Consider the spaceship operator when we can use C++20

  /// Equality comparison operator
  bool operator==(const base_iterator& other) const;

  /// Inequality comparison operator
  bool operator!=(const base_iterator& other) const;

  /// Less-than comparison operator (the left-hand side is earlier in the
  /// trajectory than the right-hand side)
  bool operator<(const base_iterator& other) const;

  /// Greater-than comparison operator (the left-hand side is later in the
  /// trajectory than the right-hand side)
  bool operator>(const base_iterator& other) const;

  /// Less-than-or-equal comparison operator
  bool operator<=(const base_iterator& other) const;

  /// Greater-than-or-equal comparison operator
  bool operator>=(const base_iterator& other) const;


  // Allow regular iterator to be cast to const_iterator
  operator const_iterator() const;


  // Allow typical copying and moving
  base_iterator(const base_iterator& other) = default;
  base_iterator(base_iterator&& other) = default;
  base_iterator& operator=(const base_iterator& other) = default;
  base_iterator& operator=(base_iterator&& other) = default;

  // Default constructor. This will leave the iterator uninitialized, so it is
  // UNDEFINED BEHAVIOR to use it without using one of the Trajectory functions
  // (like insert, find, etc) to initialize it first.
  base_iterator();

private:
  friend class Trajectory;
  friend class internal::TrajectoryIteratorImplementation;
  rmf_utils::impl_ptr<internal::TrajectoryIteratorImplementation> _pimpl;
};

extern template class Trajectory::base_iterator<Trajectory::Waypoint>;
extern template class Trajectory::base_iterator<const Trajectory::Waypoint>;

//==============================================================================
struct Trajectory::InsertionResult
{
  iterator it;
  bool inserted;
};

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__TRAJECTORY_HPP
