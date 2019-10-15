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

#include "GraphInternal.hpp"

#include <rmf_traffic/agv/Graph.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Graph::Waypoint::Implementation
{
public:

  std::size_t index;

  std::string map_name;

  Eigen::Vector2d location;

  bool holding_point;

  template<typename... Args>
  static Waypoint make(Args&&... args)
  {
    Waypoint result;
    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{std::forward<Args>(args)...});

    return result;
  }
};

//==============================================================================
const std::string& Graph::Waypoint::get_map_name() const
{
  return _pimpl->map_name;
}

//==============================================================================
auto Graph::Waypoint::set_map_name(std::string map) -> Waypoint&
{
  _pimpl->map_name = std::move(map);
  return *this;
}

//==============================================================================
const Eigen::Vector2d& Graph::Waypoint::get_location() const
{
  return _pimpl->location;
}

//==============================================================================
auto Graph::Waypoint::set_location(Eigen::Vector2d location) -> Waypoint&
{
  _pimpl->location = std::move(location);
  return *this;
}

//==============================================================================
bool Graph::Waypoint::is_holding_point() const
{
  return _pimpl->holding_point;
}

//==============================================================================
auto Graph::Waypoint::set_holding_point(bool _is_holding_point) -> Waypoint&
{
  _pimpl->holding_point = _is_holding_point;
  return *this;
}

//==============================================================================
std::size_t Graph::Waypoint::index() const
{
  return _pimpl->index;
}

//==============================================================================
Graph::Waypoint::Waypoint()
{
  // Do nothing
}

//==============================================================================
class Graph::Door::Implementation
{
public:

  std::size_t index;

  std::string name;

  Duration delay;

  template<typename... Args>
  static Door make(Args&&... args)
  {
    Door result;
    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{std::forward<Args>(args)...});

    return result;
  }
};

//==============================================================================
const std::string& Graph::Door::get_name() const
{
  return _pimpl->name;
}

//==============================================================================
auto Graph::Door::set_name(std::string name) -> Door&
{
  _pimpl->name = std::move(name);
  return *this;
}

//==============================================================================
Duration Graph::Door::get_delay() const
{
  return _pimpl->delay;
}

//==============================================================================
auto Graph::Door::set_delay(const Duration duration) -> Door&
{
  _pimpl->delay = duration;
  return *this;
}

//==============================================================================
std::size_t Graph::Door::index() const
{
  return _pimpl->index;
}

//==============================================================================
Graph::Door::Door()
{
  // Do nothing
}

namespace {
//==============================================================================
double wrap_to_pi(double value)
{
  while(value < -M_PI)
    value += 2.0*M_PI;

  while(M_PI < value)
    value -= 2.0*M_PI;

  return value;
}

//==============================================================================
class AcceptableOrientationConstraint : public Graph::OrientationConstraint
{
public:

  AcceptableOrientationConstraint(std::vector<double> acceptable)
    : orientations(std::move(acceptable))
  {
    // Do nothing
  }

  std::vector<double> orientations;

  bool apply(Eigen::Vector3d& position,
             const Eigen::Vector2d& /*course_vector*/) const final
  {
    assert(!orientations.empty());
    // This constraint can never be satisfied if there are no acceptable
    // orientations.
    if(orientations.empty())
      return false;

    const double p = position[2];
    double closest = p;
    double best_diff = std::numeric_limits<double>::infinity();
    for(const double theta : orientations)
    {
      const double diff = std::abs(wrap_to_pi(theta - p));
      if(diff < best_diff)
      {
        closest = theta;
        best_diff = diff;
      }
    }

    position[2] = closest;
    return true;
  }

  std::unique_ptr<OrientationConstraint> clone() const final
  {
    return std::make_unique<AcceptableOrientationConstraint>(*this);
  }

};
} // anonymous namespace

//==============================================================================
std::unique_ptr<Graph::OrientationConstraint>
Graph::OrientationConstraint::make(std::vector<double> acceptable_orientations)
{
  return std::make_unique<AcceptableOrientationConstraint>(
        std::move(acceptable_orientations));
}

//==============================================================================
class Graph::Lane::Node::Implementation
{
public:

  std::size_t waypoint;

  struct Constraints
  {
    Constraints(
        std::unique_ptr<OrientationConstraint> _orientation,
        std::unique_ptr<VelocityConstraint> _velocity)
      : orientation(std::move(_orientation)),
        velocity(std::move(_velocity))
    {
      // Do nothing
    }

    std::unique_ptr<OrientationConstraint> orientation;
    std::unique_ptr<VelocityConstraint> velocity;

    Constraints(const Constraints& other)
    {
      *this = other;
    }

    Constraints& operator=(const Constraints& other)
    {
      orientation = other.orientation->clone();
      velocity = other.velocity->clone();
      return *this;
    }

    Constraints(Constraints&&) = default;
    Constraints& operator=(Constraints&&) = default;
  };

  Constraints constraints;

};

//==============================================================================
Graph::Lane::Node::Node(
    std::size_t waypoint_index,
    std::unique_ptr<OrientationConstraint> orientation,
    std::unique_ptr<VelocityConstraint> velocity)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               waypoint_index,
               Implementation::Constraints(
                std::move(orientation),
                std::move(velocity))}))
{
  // Do nothing
}

//==============================================================================
std::size_t Graph::Lane::Node::waypoint_index() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Graph::Lane::Node::orientation_constraint() const
-> const OrientationConstraint*
{
  return _pimpl->constraints.orientation.get();
}

//==============================================================================
auto Graph::Lane::Node::velocity_constraint() const
-> const VelocityConstraint*
{
  return _pimpl->constraints.velocity.get();
}

//==============================================================================
class Graph::Lane::Implementation
{
public:

  std::size_t index;

  Node entry;

  Node exit;

  bool has_door;
  std::size_t door_index;

  template<typename... Args>
  static Lane make(Args&&... args)
  {
    Lane lane;
    lane._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{std::forward<Args>(args)...});

    return lane;
  }
};

//==============================================================================
auto Graph::Lane::entry() const -> const Node&
{
  return _pimpl->entry;
}

//==============================================================================
auto Graph::Lane::exit() const -> const Node&
{
  return _pimpl->exit;
}

//==============================================================================
std::size_t Graph::Lane::index() const
{
  return _pimpl->index;
}

//==============================================================================
const std::size_t* Graph::Lane::door_index() const
{
  if(_pimpl->has_door)
    return &_pimpl->door_index;

  return nullptr;
}

//==============================================================================
Graph::Lane::Lane()
{
  // Do nothing
}

//==============================================================================
auto Graph::add_waypoint(
    std::string map_name,
    Eigen::Vector2d location,
    const bool is_holding_point) -> Waypoint&
{
  _pimpl->waypoints.emplace_back(
        Waypoint::Implementation::make(
          _pimpl->waypoints.size(),
          std::move(map_name), std::move(location), is_holding_point));

  _pimpl->lanes_from.push_back({});

  return _pimpl->waypoints.back();
}

//==============================================================================
auto Graph::get_waypoint(const std::size_t index) -> Waypoint&
{
  return _pimpl->waypoints.at(index);
}

//==============================================================================
auto Graph::get_waypoint(const std::size_t index) const -> const Waypoint&
{
  return _pimpl->waypoints.at(index);
}

//==============================================================================
std::size_t Graph::num_waypoints() const
{
  return _pimpl->waypoints.size();
}

//==============================================================================
auto Graph::add_door(std::string name, Duration delay) -> Door&
{
  _pimpl->doors.emplace_back(
        Door::Implementation::make(
          _pimpl->doors.size(), std::move(name), delay));

  return _pimpl->doors.back();
}

//==============================================================================
auto Graph::get_door(const std::size_t index) -> Door&
{
  return _pimpl->doors.at(index);
}

//==============================================================================
auto Graph::get_door(const std::size_t index) const -> const Door&
{
  return _pimpl->doors.at(index);
}

//==============================================================================
std::size_t Graph::num_doors() const
{
  return _pimpl->doors.size();
}

//==============================================================================
auto Graph::add_lane(Lane::Node entry, Lane::Node exit) -> Lane&
{
  assert(entry.waypoint_index() < _pimpl->waypoints.size());
  assert(exit.waypoint_index() < _pimpl->waypoints.size());

  const std::size_t lane_id = _pimpl->lanes.size();
  _pimpl->lanes_from[entry.waypoint_index()].push_back(lane_id);

  _pimpl->lanes.emplace_back(
        Lane::Implementation::make(
          _pimpl->lanes.size(),
          std::move(entry),
          std::move(exit),
          false, std::size_t()));

  return _pimpl->lanes.back();
}

//==============================================================================
auto Graph::add_lane(Lane::Node entry, Lane::Node exit, const Door& door)
-> Lane&
{
  _pimpl->lanes.emplace_back(
        Lane::Implementation::make(
          _pimpl->lanes.size(),
          std::move(entry),
          std::move(exit),
          true, door.index()));

  return _pimpl->lanes.back();
}

//==============================================================================
std::size_t Graph::num_lanes() const
{
  return _pimpl->lanes.size();
}

} // namespace avg
} // namespace rmf_traffic
