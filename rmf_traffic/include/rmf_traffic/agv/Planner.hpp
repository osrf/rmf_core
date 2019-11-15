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

#ifndef RMF_TRAFFIC__AGV__PLANNER_HPP
#define RMF_TRAFFIC__AGV__PLANNER_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rmf_traffic/schedule/Viewer.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
// Forward declaration
class Plan;

//==============================================================================
class Planner
{
public:

  /// The Configuration class contains planning parameters that are immutable
  /// for each Planner instance.
  ///
  /// These parameters generally describe the capabilities or behaviors of the
  /// AGV that is being planned for, so they shouldn't need to change in between
  /// plans anyway.
  class Configuration
  {
  public:

    /// Constructor
    ///
    /// \param[in] vehicle_traits
    ///   The traits of the vehicle that is being planned for
    ///
    /// \param[in] graph
    ///   The graph which is being planned over
    Configuration(
        Graph graph,
        VehicleTraits traits,
        Interpolate::Options interpolation = Interpolate::Options());

    /// Set the graph to use for planning
    Configuration& graph(Graph graph);

    /// Get a mutable reference to the graph
    Graph& graph();

    /// Get a const reference to the graph
    const Graph& graph() const;

    /// Set the vehicle traits to use for planning
    Configuration& vehicle_traits(VehicleTraits traits);

    /// Get a mutable reference to the vehicle traits
    VehicleTraits& vehicle_traits();

    /// Get a const reference to the vehicle traits
    const VehicleTraits& vehicle_traits() const;

    /// Set the interpolation options for the planner
    Configuration& interpolation(Interpolate::Options interpolate);

    /// Get a mutable reference to the interpolation options
    Interpolate::Options& interpolation();

    /// Get a const reference to the interpolation options
    const Interpolate::Options& interpolation() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The Options class contains planning parameters that can change between
  /// each planning attempt.
  class Options
  {
  public:

    /// Constructor
    ///
    /// \warning You are expected to maintain the lifetime of the schedule
    /// viewer for as long as this Options instance is alive. The Options
    /// instance will only retain a reference to the viewer, not a copy of it.
    ///
    /// \param[in] viewer
    ///   The schedule viewer which will be used to check for conflicts
    ///
    /// \param[in] min_hold_time
    ///   The minimum amount of time that the planner should spend waiting at
    ///   holding points. Smaller values will make the plan more aggressive
    ///   about being time-optimal, but the plan may take longer to produce.
    ///   Larger values will add some latency to the execution of the plan as
    ///   the robot may wait at a holding point longer than necessary, but the
    ///   plan will usually be generated more quickly.
    Options(
        const schedule::Viewer& viewer,
        Duration min_hold_time = std::chrono::seconds(5));

    /// Change the schedule viewer to use for planning.
    ///
    /// \warning The Options instance will store a reference to the viewer; it
    /// will not store a copy. Therefore you are responsible for keeping the
    /// schedule viewer alive while this Options class is being used.
    Options& schedule_viewer(const schedule::Viewer& viewer);

    /// Get a const reference to the schedule viewer that will be used for
    /// planning. It is undefined behavior to call this function is called after
    /// the schedule viewer has been destroyed.
    const schedule::Viewer& schedule_viewer() const;

    /// Set the minimal amount of time to spend waiting at holding points
    Options& minimum_holding_time(Duration holding_time);

    /// Get the minimal amount of time to spend waiting at holding points
    Duration minimum_holding_time() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Describe the starting conditions of a plan.
  class Start
  {
  public:

    /// Constructor
    ///
    /// \param[in] inital_time
    ///   The starting time of the plan.
    ///
    /// \param[in] initial_waypoint
    ///   The waypoint index that the plan will begin from.
    ///
    /// \param[in] initial_orientation
    ///   The orientation that the AGV will start with.
    Start(
        Time initial_time,
        std::size_t initial_waypoint,
        double initial_orientation);

    /// Set the starting time of a plan
    Start& time(Time initial_time);

    /// Get the starting time
    Time time() const;

    /// Set the starting waypoint of a plan
    Start& waypoint(std::size_t initial_waypoint);

    /// Get the starting waypoint
    std::size_t waypoint() const;

    /// Set the starting orientation of a plan
    Start& orientation(double initial_orientation);

    /// Get the starting orientation
    double orientation() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Describe the goal conditions of a plan.
  class Goal
  {
  public:

    // TODO(MXG): Use std::optional for the goal orientation when we can have
    // support for C++17.

    // TODO(MXG): Consider supporting goals that have multiple acceptable goal
    // orientations.

    /// Constructor
    ///
    /// \note With this constructor, any final orientation will be accepted.
    ///
    /// \param[in] goal_waypoint
    ///   The waypoint that the AGV needs to reach.
    Goal(std::size_t goal_waypoint);

    /// Constructor
    ///
    /// \param[in] goal_waypoint
    ///   The waypoint that the AGV needs to reach.
    ///
    /// \param[in] goal_orientation
    ///   The orientation that the AGV needs to end with.
    Goal(std::size_t goal_waypoint, double goal_orientation);

    /// Set the goal waypoint.
    Goal& waypoint(std::size_t goal_waypoint);

    /// Get the goal waypoint.
    std::size_t waypoint() const;

    /// Set the goal orientation.
    Goal& orientation(double goal_orientation);

    /// Accept any orientation for the final goal.
    Goal& any_orientation();

    /// Get a reference to the goal orientation (or a nullptr if any orientation
    /// is acceptable).
    const double* orientation() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Constructor
  ///
  /// \param[in] config
  ///   This is the Configuration for the Planner. The Planner instance will
  ///   maintain a cache while it performs planning requests. This cache will
  ///   offer potential speed ups to subsequent planning requests, but the
  ///   correctness of the cache depends on the fields in the Configuration to
  ///   remain constant. Therefore you are not permitted to modify a Planner's
  ///   Configuration after the Planner is constructed. To change the planning
  ///   Configuration, you will need to create a new Planner instance with the
  ///   desired Configuration.
  ///
  /// \param[in] default_options
  ///   Unlike the Configuration, you are allowed to change a Planner's Options.
  ///   The parameter given here will be used as the default options, so you can
  ///   set them here and then forget about them. These options can be overriden
  ///   each time you request a plan.
  Planner(
      Configuration config,
      Options default_options);

  /// Get a const reference to the configuration for this Planner. Note that the
  /// configuration of a planner cannot be changed once it is set.
  ///
  /// \note The Planner maintains a cache that allows searches to become
  /// progressively faster. This cache depends on the fields in the Planner's
  /// configuration, so those fields cannot be changed without invalidating that
  /// cache. To plan using a different configuration, you should create a new
  /// Planner instance with the desired configuration.
  const Configuration& get_configuration() const;

  /// Change the default planning options.
  Planner& set_default_options(Options default_options);

  /// Get a mutable reference to the default planning options.
  Options& get_default_options();

  /// Get a const reference to the default planning options.
  const Options& get_default_options() const;

  /// Produce a plan for the given starting conditions and goal. The default
  /// Options of this Planner instance will be used.
  ///
  /// \param[in] start
  ///   The starting conditions
  ///
  /// \param[in] goal
  ///   The goal conditions
  Plan plan(Start start, Goal goal) const;

  /// Product a plan for the given start and goal conditions. Override the
  /// default options.
  ///
  /// \param[in] start
  ///   The starting conditions
  ///
  /// \param[in] goal
  ///   The goal conditions
  ///
  /// \param[in] options
  ///   The Options to use for this plan. This overrides the default Options of
  ///   the Planner instance.
  Plan plan(Start start, Goal goal, Options options) const;


  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

//==============================================================================
class Plan
{
public:

  using Start = Planner::Start;
  using Goal = Planner::Goal;
  using Options = Planner::Options;
  using Configuration = Planner::Configuration;

  /// A Waypoint within a Plan.
  ///
  /// This class helps to discretize a Plan based on the Waypoints belonging to
  /// the agv::Graph. Each Graph::Waypoint that the Plan stops or turns at will
  /// be accounted for by a Plan::Waypoint.
  ///
  /// To indicate the intended orientation, each of these Waypoints provides an
  /// Eigen::Vector3d where the third element is the orientation.
  ///
  /// The time that the position is meant to be arrived at is also given by the
  /// Waypoint.
  ///
  /// \note Users are not allowed to make their own Waypoint instances, because
  /// it is too easy to accidentally get inconsistencies in the position and
  /// graph_index fields. Plan::Waypoints can only be created by Plan instances
  /// and can only be retrieved using Plan::get_waypoints().
  class Waypoint
  {
  public:

    /// Get the position for this Waypoint
    const Eigen::Vector3d& position() const;

    /// Get the time for this Waypoint
    rmf_traffic::Time time() const;

    /// Get the graph index of this Waypoint
    std::size_t graph_index() const;

    /// A reference to the graph edge that needs to be traversed to reach this
    /// waypoint.
    ///
    /// Sometimes
    const std::size_t* graph_edge() const;

    class Implementation;
  private:
    Waypoint();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Returns true if this instance contains a valid plan and returns false
  /// otherwise. Using any member function besides valid() when the Plan is
  /// invalid is undefined behavior, potentially resulting in a segmentation
  /// fault or worse.
  ///
  /// \note A default-constructed Plan instance will be invalid until a valid
  /// plan is copy-assigned or move-assigned into it.
  //
  // TODO(MXG): When C++17 is supported, consider using std::optional to express
  // the validity of a plan so that Plan instances are always valid and its
  // member functions are always well-behaved.
  bool valid() const;

  /// Implicitly cast this Plan instance to the return value of valid()
  operator bool() const;

  /// If this Plan is valid, this will return the trajectory of the successful
  /// plan.
  ///
  /// \warning If this plan is not valid, this will have undefined behavior, and
  /// will cause a segmentation fault if this Plan is uninitialized
  /// (default-constructed).
  const std::vector<Trajectory>& get_trajectories() const;

  /// If this plan is valid, this will return the waypoints of the successful
  /// plan.
  ///
  /// \warning If this plan is not valid, this will have undefined behavior, and
  /// will cause a segmentation fault if this Plan is uninitialized
  /// (default-constructed).
  const std::vector<Waypoint>& get_waypoints() const;

  /// If this Plan was valid, this will ask for a new plan to the same goal
  /// using the same options but with new starting conditions. This can be used
  /// to replan after the schedule has changed or if the robot is not tracking
  /// the original plan well.
  ///
  /// \warning If this Plan instance is not valid, this will have undefined
  /// behavior, and will cause a segmentation fault if this Plan is
  /// uninitialized.
  ///
  /// \param[in] new_start
  ///   The starting conditions that should be used for replanning.
  Plan replan(Start new_start) const;

  /// If this Plan was valid, this will ask for a new plan to the same goal.
  ///
  /// \warning If this Plan instance is not valid, this will have undefined
  /// behavior, and will cause a segmentation fault if this Plan is
  /// uninitialized.
  ///
  /// \param[in] new_start
  ///   The starting conditions that should be used for replanning.
  ///
  /// \param[in] new_options
  ///   The options that should be used for replanning.
  Plan replan(Start new_start, Options new_options) const;

  /// If this Plan is valid, this will return the Planner::Start that was used
  /// to produce it.
  const Start& get_start() const;

  /// If this Plan is valid, this will return the Planner::Goal that was used
  /// to produce it.
  ///
  /// If replan() is called, this goal will be used to produce the new Plan.
  const Goal& get_goal() const;

  /// If this Plan is valid, this will return the Planner::Options that were
  /// used to produce it.
  ///
  /// If replan(Planner::Start) is called, these Planner::Options will be used
  /// to produce the new Plan.
  const Options& get_options() const;

  /// If this Plan is valid, this will return the Planner::Configuration that
  /// was used to produce it.
  ///
  /// If replan() is called, this Planner::Configuration will be used to produce
  /// the new Plan.
  const Configuration& get_configuration() const;

  // TODO(MXG): Create a feature that can diff two plans to produce the most
  // efficient schedule::Database::Change to get from the original plan to the
  // new plan.

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__PLANNER_HPP
