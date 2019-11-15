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
#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <vector>

namespace rmf_traffic {

//==============================================================================
namespace detail {
/// \internal We declare this private PIMPL class outside of the
/// Trajectory::base_iterator class so that it does not need to be templated.
class TrajectoryIteratorImplementation;
} // namespace detail

//==============================================================================
class Trajectory
{
public:

  class Profile;
  using ProfilePtr = std::shared_ptr<Profile>;
  using ConstProfilePtr = std::shared_ptr<const Profile>;

  /// The Trajectory::Profile class describes how the trajectory will take up
  /// space at a single instant in time. A profile description is assigned to
  /// each segment of a trajectory, so that the profile of a trajectory may
  /// change over time, for example as the robot changes its mode of movement.
  ///
  /// There are two factors to describe the profile:
  ///  * shape - Describes the occupied space
  ///  * movement - Describes how the robot is moving
  ///
  /// For Queued movement, the profile has an additional parameter:
  ///  * queue_id
  ///
  /// The queue_id indicates which queue the robot will be waiting in.
  class Profile
  {
  public:

    /// The Trajectory::Profile::Autonomy enum describes how much freedom the
    /// robot has during this phase of its Trajectory.
    enum class Autonomy : uint16_t
    {
      /// This autonomy type is illegal and will always be rejected by the
      /// schedule verifier. Having this movement type implies a major bug in
      /// the code and should be reported immediately.
      Unspecified = 0,

      /// The robot will try to follow the specified trajectory exactly, and
      /// the collision geometry represents all of the clearance space that the
      /// robot will need while traveling.
      Guided,

      /// The robot is waiting in a queue, and will wait to traverse the
      /// trajectory segment until the rmf_traffic_monitor tells it to proceed.
      /// The collision geometry represents all the clearance space that the
      /// robot will need while waiting.
      Queued,

      /// The robot will autonomously navigate within the specified space. The
      /// robot will only occupy a subset of the space that is specified by the
      /// collision geometry, but the Fleet Adapter cannot predict ahead of time
      /// exactly what that occupied space will be.
      ///
      /// If this plan is accepted by the Schedule, then any time another plan
      /// is submitted where a Guided or Queued trajectory segment conflicts
      /// with this Trajectory's Autonomous space, the Fleet Adapter that
      /// submitted this Trajectory will be asked to approve or reject the other
      /// plan based on whether the other plan will interfere with this
      /// Trajectory.
      Autonomous,
    };

    /// Create a profile with Guided movement
    static ProfilePtr make_guided(geometry::ConstFinalConvexShapePtr shape);

    /// Create a profile with Autonomous movement
    static ProfilePtr make_autonomous(geometry::ConstFinalConvexShapePtr shape);

    /// Create a profile for a Queued segment
    static ProfilePtr make_queued(
        geometry::ConstFinalConvexShapePtr shape,
        const std::string& queue_id);

    /// Get the shape being used for this profile
    geometry::ConstFinalConvexShapePtr get_shape() const;

    /// Set the shape that will be used by this profile
    Profile& set_shape(geometry::ConstFinalConvexShapePtr new_shape);

    /// Get the autonomy type being used for this profile
    Autonomy get_autonomy() const;

    //==========================================================================
    /// This class is a placeholder in case we ever want to extend the features
    /// of the Guided mode. Currently it does not do anything.
    class GuidedInfo
    {
    public:

      // There are no special information features for Guided mode yet.

    private:
      GuidedInfo(void* pimpl);
      friend class Profile;
      const void* const _pimpl;
    };

    /// Set the movement of this profile to Guided
    GuidedInfo& set_to_guided();

    //==========================================================================
    /// This class is a placeholder in case we ever want to extend the features
    /// of the Autonomous mode. Currently it does not do anything.
    class AutonomousInfo
    {
    public:

      // There are no special information features for Autonomous mode yet.

    private:
      AutonomousInfo(void* pimpl);
      friend class Profile;
      const void* const _pimpl;
    };

    /// Set the movement of this profile to Autonomous
    AutonomousInfo& set_to_autonomous();

    //==========================================================================
    class QueueInfo
    {
    public:

      /// Get the id of the queue that this profile is waiting in
      std::string get_queue_id() const;

    private:
      QueueInfo(void* pimpl);
      friend class Profile;
      const void* const _pimpl;
    };

    /// Set the movement of this profile to queued
    QueueInfo& set_to_queued(const std::string& queue_id);

    /// If this Profile is queued, this will return a pointer to its queue
    /// information. If it is not in a queue, this will return a nullptr.
    ///
    /// This pointer is potentially invalidated any time a modification is made
    /// to the Profile object that provided it.
    const QueueInfo* get_queue_info() const;

  private:

    Profile(geometry::ConstFinalConvexShapePtr shape);

    class Implementation;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class Segment
  {
  public:

    /// Get the profile of this Trajectory Segment
    ConstProfilePtr get_profile() const;

    /// Change the profile of this Trajectory Segment
    ///
    /// \param[in] new_profile
    ///   The new profile for this Trajectory Segment.
    Segment& set_profile(ConstProfilePtr new_profile);

    /// Get the intended physical location of the robot at the end of this
    /// Trajectory Segment.
    ///
    /// This is a 2D homogeneous position. The first two values in the vector
    /// are x and y coordinates, while the third is rotation about the z-axis.
    Eigen::Vector3d get_finish_position() const;

    /// Set the intended physical location of the robot at the end of this
    /// Trajectory Segment.
    ///
    /// This is a 2D homogeneous position. The first two values in the vector
    /// are x and y coordinates, while the third is rotation about the z-axis.
    ///
    /// \param[in] new_position
    ///   The new finishing position for this Trajectory Segment.
    Segment& set_finish_position(Eigen::Vector3d new_position);

    /// Get the intended velocity of the robot at the end of this Trajectory
    /// Segment.
    ///
    /// This is a 2D homogeneous position. The first two values in the vector
    /// are x and y velocities, while the third is rotational velocity about the
    /// z-axis.
    Eigen::Vector3d get_finish_velocity() const;

    /// Set the intended velocity of the robot at the end of this Trajectory
    /// Segment.
    ///
    /// This is a 2D homogeneous position. The first two values in the vector
    /// are x and y coordinates, while the third is rotation about the z-axis.
    ///
    /// \param[in] new_velocity
    ///   The new finishing velocity for this Trajectory Segment.
    Segment& set_finish_velocity(Eigen::Vector3d new_velocity);

    /// Get the time that this Trajectory Segment is meant to finish.
    Time get_finish_time() const;

    /// Change the finish time of this Trajectory Segment. Note that this
    /// function will only affect this segment, and may cause this Segment to be
    /// reordered within the Trajectory.
    ///
    /// To change the finish time for this segment while preserving the relative
    /// times of all subsequent Trajectory segments, use adjust_finish_times()
    /// instead.
    ///
    /// \warning If you change the finishing time value of this Segment such
    /// that it falls directly on another Segment's finish time, you will get a
    /// std::invalid_argument exception, because discontinuous jumps are not
    /// supported, and indicate a significant mishandling of trajectory data,
    /// which is most likely a serious bug that should be remedied.
    ///
    /// \note If this Segment's finish time crosses over another Segment's
    /// finish time, that signficantly changes the topology of the Trajectory,
    /// because it will change the order in which the positions are passed
    /// through.
    ///
    /// \param[in] new_time
    ///   The new finishing time for this Trajectory Segment.
    ///
    /// \sa adjust_finish_times(Time new_time)
    Segment& set_finish_time(Time new_time);

    /// Adjust the finishing time of this segment and all subsequent segments by
    /// the given duration. This is guaranteed to maintain the ordering of the
    /// Trajectory Segments, and is more efficient than changing all the times
    /// directly.
    ///
    /// \warning If a negative delta_t is given, it must not cause this
    /// Segment's finish time to be less than or equal to the finish time of its
    /// preceding Segment, or else a std::invalid_argument exception will be
    /// thrown.
    ///
    /// \param[in] delta_t
    ///   How much to change the finishing time of this segment and all later
    ///   segments. If negative, it must not cross over the finish time of the
    ///   previous segment, or else a std::invalid_argument will be thrown.
    ///
    /// \sa set_finish_time(Time new_time)
    void adjust_finish_times(Duration delta_t);

    /// Compute the motion across this Trajectory segment
    std::unique_ptr<Motion> compute_motion() const;

    class Implementation;
  private:

    /// \internal Private constructor. Use Trajectory::add_segment() to create
    /// a new Trajectory Segment.
    Segment();
    Segment(const Segment&) = delete;
    Segment(Segment&&) = default;
    Segment& operator=(const Segment&) = delete;
    Segment& operator=(Segment&&) = default;
    friend class Trajectory;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  // These classes allow users to traverse the contents of the Trajectory.
  // The trajectory operates much like a typical C++ container, but only for
  // Trajectory::Segment information.
  template<typename SegT>
  class base_iterator;
  using iterator = base_iterator<Segment>;
  using const_iterator = base_iterator<const Segment>;

  /// Create a Trajectory that takes place on the specified map
  Trajectory(std::string map_name);

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

  /// Get the name of the map that this Trajectory takes place in
  std::string get_map_name() const;

  /// Set which map this Trajectory takes place in
  Trajectory& set_map_name(std::string name);

  /// Contains two fields:
  /// * iterator it:   contains the iterator for the Segment that ends at the
  ///                  given finish_time
  /// * bool inserted: true if the Segment was inserted, false if a Segment with
  ///                  the exact same finish_time already existed
  struct InsertionResult;

  /// Add a Segment to this Trajectory.
  ///
  /// The Segment will be inserted into the Trajectory according to its
  /// finish_time, ensuring correct ordering of all Segments.
  InsertionResult insert(
      Time finish_time,
      ConstProfilePtr profile,
      Eigen::Vector3d position,
      Eigen::Vector3d velocity);

  /// Insert a copy of another Trajectory's Segment into this one.
  InsertionResult insert(const Segment& other);

  /// Find the Segment of this Trajectory that is active during the given time.
  ///
  /// \note This will return Trajectory::end() if the time is before the
  /// Trajectory starts or after the Trajectory finishes.
  ///
  /// \param[in] time
  ///   The time of interest.
  ///
  /// \return the Segment that is active during the given time, or
  /// Trajectory::end() if the time falls outside the range of the Trajectory.
  iterator find(Time time);

  /// const-qualified version of find()
  const_iterator find(Time time) const;

  /// Erase the specified segment.
  ///
  /// \return an iterator following the last removed element
  iterator erase(iterator segment);

  /// Erase the range of elements: [first, last).
  ///
  /// \note The `last` element is not included in the range.
  ///
  /// \return an iterator following the last removed element
  iterator erase(iterator first, iterator last);

  /// Returns an iterator to the fist Segment of the Trajectory.
  ///
  /// If the Trajectory is empty, the returned iterator will be equal to end().
  iterator begin();

  /// const-qualified version of begin()
  const_iterator begin() const;

  /// Explicitly call the const-qualified version of begin()
  const_iterator cbegin() const;

  /// Returns an iterator to the element following the last Segment of the
  /// Trajectory. This iterator acts as a placeholder; attempting to dereference
  /// it results in undefined behavior.
  ///
  /// \note In compliance with C++ standards, this is really a one-past-the-end
  /// iterator and must not be dereferenced. It should only be used to identify
  /// when an iteration must end. See: https://en.cppreference.com/w/cpp/container/list/end
  iterator end();

  /// const-qualified version of end()
  const_iterator end() const;

  /// Explicitly call the const-qualified version of end()
  const_iterator cend() const;

  /// Get a mutable reference to the first Segment in this Trajectory.
  ///
  /// \warning Calling this function on an empty trajectory is undefined.
  Segment& front();

  /// Get a const reference to the first Segment in this Trajectory.
  ///
  /// \warning Calling this function on an empty trajectory is undefined.
  const Segment& front() const;

  /// Get a mutable reference to the last Segment in this Trajectory.
  ///
  /// \warning Calling this function on an empty trajectory is undefined.
  Segment& back();

  /// Get a const reference to the last Segment in this Trajectory.
  ///
  /// \warning Calling this function on an empty trajectory is undefined.
  const Segment& back() const;

  /// Get the start time, if available. This will return a nullptr if the
  /// Trajectory is empty.
  const Time* start_time() const;

  /// Get the finish time, if available. This will return a nullptr if the
  /// Trajectory is empty.
  const Time* finish_time() const;

  /// Get the duration of the Trajectory. This will be 0 if the Trajectory is
  /// empty or if it has only one Segment.
  Duration duration() const;

  /// Get the number of Segments in the Trajectory. To be used in conflict
  /// detection, the Trajectory must have a size of at least 2.
  std::size_t size() const;

  /// \internal Used internally by unit and integration tests so we can test
  /// private imeplementation details.
  class Debug;

private:
  friend class detail::TrajectoryIteratorImplementation;
  class Implementation;
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;

};

//==============================================================================
template<typename SegT>
class Trajectory::base_iterator
{
public:

  /// Dereference operator
  SegT& operator*() const;

  /// Drill-down operator
  SegT* operator->() const;

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
  friend class detail::TrajectoryIteratorImplementation;
  rmf_utils::impl_ptr<detail::TrajectoryIteratorImplementation> _pimpl;
};

extern template class Trajectory::base_iterator<Trajectory::Segment>;
extern template class Trajectory::base_iterator<const Trajectory::Segment>;

//==============================================================================
struct Trajectory::InsertionResult
{
  iterator it;
  bool inserted;
};

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__TRAJECTORY_HPP
