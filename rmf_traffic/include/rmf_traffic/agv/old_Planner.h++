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
class Planner
{
public:

  /// The Options class contains planning parameters that are generally expected
  /// to remain constant between planning operations.
  class Options
  {
  public:

    /// Constructor
    ///
    /// \warning You are expected to maintain the lifetime of the schedule
    /// viewer for as long as this Options instance is alive. The Options
    /// instance will only retain a reference to the viewer, not a copy of it.
    ///
    /// \param[in] vehicle_traits
    ///   The traits of the vehicle that is being planned for
    ///
    /// \param[in] graph
    ///   The graph which is being planned over
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
        VehicleTraits vehicle_traits,
        Graph graph,
        const schedule::Viewer& viewer,
        Duration min_hold_time = std::chrono::seconds(5),
        Interpolate::Options interpolation = Interpolate::Options());

    /// Set the vehicle traits to use for planning
    Options& set_vehicle_traits(VehicleTraits traits);

    /// Get a mutable reference to the vehicle traits
    VehicleTraits& get_vehicle_traits();

    /// Get a const reference to the vehicle traits
    const VehicleTraits& get_vehicle_traits() const;

    // TODO(MXG): Instead of having Graph be one of the options, we should have
    // a Planner class whose constructor takes in a Graph instance. That Planner
    // instance can cache results as it solves planning problems on the Graph,
    // allowing for better performance with each run. The solve() function would
    // no longer be static.

    /// Set the graph to use for planning
    Options& set_graph(Graph graph);

    /// Get a mutable reference to the graph
    Graph& get_graph();

    /// Get a const reference to the graph
    const Graph& get_graph() const;

    /// Change the schedule viewer to use for planning.
    ///
    /// \warning The Options instance will store a reference to the viewer; it
    /// will not store a copy. Therefore you are responsible for keeping the
    /// schedule viewer alive while this Options class is being used.
    Options& change_schedule_viewer(const schedule::Viewer& viewer);

    /// Get a const reference to the schedule viewer that will be used for
    /// planning. It is undefined behavior to call this function is called after
    /// the schedule viewer has been destroyed.
    const schedule::Viewer& get_schedule_viewer() const;

    /// Set the minimal amount of time to spend waiting at holding points
    Options& set_mininum_holding_time(Duration holding_time);

    /// Get the minimal amount of time to spend waiting at holding points
    Duration get_minimum_holding_time() const;

    /// Set the interpolation options for the planner
    Options& set_interpolation(Interpolate::Options interpolate);

    /// Get a mutable reference to the interpolation options
    Interpolate::Options& get_interpolation();

    /// Get a const reference to the interpolation options
    const Interpolate::Options& get_interpolation() const;

    // TODO(MXG): Consider adding an option to cap the maximum amount of time
    // that can be searched through. For example, if the planner has traversed
    // through 2 hours worth of time in its planning state, then it should
    // give up and return false. Otherwise it's possible for the search to
    // continue indefinitely if there are any holding points in the graph.

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  struct Waypoint
  {
    Eigen::Vector3d location;
    rmf_traffic::Time time;
  };

  /// Find a solution to a planning problem.
  ///
  /// \param[in] initial_time
  ///   The time that the robot will be able to begin following the plan
  ///
  /// \param[in] initial_waypoint
  ///   The waypoint that the robot is starting from
  ///
  /// \param[in] initial_orientation
  ///   The orientation that the robot begins with
  ///
  /// \param[in] goal_waypoint
  ///   The waypoint that the planner should try to reach
  ///
  /// \param[in] goal_orientation
  ///   The orientation that the vehicle should try to reach. If the orientation
  ///   at the end of the plan is irrelevant, then a nullptr can be given for
  ///   this argument. TODO(MXG): Add an overload that uses std::optional when
  ///   we can have C++17 support.
  ///
  /// \param[in] options
  ///   The options to use while planning.
  ///
  /// \param[out] solution
  ///   A vector of Trajectory instances that bring the robot from the
  ///   initial waypoint to the goal waypoint. If the goal could not be reached,
  ///   then this vector will be empty, and the function will return false. If
  ///   the initial conditions already satisfy the goals, the solution will contain 
  ///   an empty trajectory while the function will return true.
  ///   
  ///
  /// \param[out] positions
  ///   If non-null, this vector will be filled with the waypoint positions that
  ///   the robot is expected to pass through.
  ///
  /// \return True if a solution was found, false otherwise.
  static bool solve(
      Time initial_time,
      std::size_t initial_waypoint,
      double initial_orientation,
      std::size_t goal_waypoint,
      const double* goal_orientation,
      const Options& options,
      std::vector<Trajectory>& solution,
      std::vector<Waypoint>* waypoints = nullptr);

  // TODO(MXG): Instead of returning a boolean and having an output parameter,
  // the solve() function should return a Plan class (TBD), which will contain
  // information about whether a solution was found, what the trajectory of the
  // solution is, and what waypoints were traversed for the solution. It could
  // also have functions for replanning when a schedule conflict arrives.

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


} // namespace agv
} // namespace rmf_traffic


#endif // RMF_TRAFFIC__AGV__PLANNER_HPP
