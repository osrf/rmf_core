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

#include "../utils.hpp"

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

namespace {
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
      const double diff = std::abs(internal::wrap_to_pi(theta - p));
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

//==============================================================================
// TODO(MXG): Think about how to refactor this constraint so that it can share
// an implementation with DifferentialOrientationConstraint. Maybe instead of
// a single direction it could have a std::vector of acceptable directions.
// Or it can have `bool forward_okay` and `bool backward_okay` fields.
class DirectionConstraint : public Graph::OrientationConstraint
{
public:

  static Eigen::Rotation2Dd compute_forward_offset(
      const Eigen::Vector2d& forward)
  {
    return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
  }

  static const Eigen::Rotation2Dd R_pi;

  DirectionConstraint(
      Direction _direction,
      const Eigen::Vector2d& _forward_vector)
    : R_f(compute_forward_offset(_forward_vector)),
      R_f_inv(R_f.inverse()),
      direction(_direction)
  {
    // Do nothing
  }

  Eigen::Rotation2Dd R_f;
  Eigen::Rotation2Dd R_f_inv;
  Direction direction;

  Eigen::Rotation2Dd compute_R_final(
      const Eigen::Vector2d& course_vector) const
  {
    const Eigen::Rotation2Dd R_c(
          std::atan2(course_vector[1], course_vector[0]));

    if(Direction::Backward == direction)
      return R_pi * R_c * R_f_inv;

    return R_c * R_f_inv;
  }

  bool apply(
      Eigen::Vector3d& position,
      const Eigen::Vector2d& course_vector) const final
  {
    position[2] = internal::wrap_to_pi(compute_R_final(course_vector).angle());
    return true;
  }

  std::unique_ptr<OrientationConstraint> clone() const final
  {
    return std::make_unique<DirectionConstraint>(*this);
  }
};

//==============================================================================
const Eigen::Rotation2Dd DirectionConstraint::R_pi = Eigen::Rotation2Dd(M_PI);

} // anonymous namespace

//==============================================================================
std::unique_ptr<Graph::OrientationConstraint>
Graph::OrientationConstraint::make(std::vector<double> acceptable_orientations)
{
  return std::make_unique<AcceptableOrientationConstraint>(
        std::move(acceptable_orientations));
}

//==============================================================================
std::unique_ptr<Graph::OrientationConstraint>
Graph::OrientationConstraint::make(
    Direction direction,
    const Eigen::Vector2d& forward)
{
  return std::make_unique<DirectionConstraint>(direction, forward);
}

//==============================================================================
class Graph::Lane::Door::Implementation
{
public:

  std::string name;
  Duration duration;

};

//==============================================================================
Graph::Lane::Door::Door(
    std::string name,
    Duration duration)
: _pimpl(rmf_utils::make_impl<Implementation>(
           Implementation{
             std::move(name),
             duration
           }))
{
  // Do nothing
}

//==============================================================================
const std::string& Graph::Lane::Door::name() const
{
  return _pimpl->name;
}

//==============================================================================
auto Graph::Lane::Door::name(std::string name) -> Door&
{
  _pimpl->name = std::move(name);
  return *this;
}

//==============================================================================
Duration Graph::Lane::Door::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
auto Graph::Lane::Door::duration(Duration duration_) -> Door&
{
  _pimpl->duration = duration_;
  return *this;
}

//==============================================================================
class Graph::Lane::LiftDoor::Implementation
{
public:

  std::string lift_name;
  std::string floor_name;
  Duration duration;

};

//==============================================================================
Graph::Lane::LiftDoor::LiftDoor(
    std::string lift_name,
    std::string floor_name,
    Duration duration)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(lift_name),
               std::move(floor_name),
               duration
             }))
{
  // Do nothing
}

//==============================================================================
const std::string& Graph::Lane::LiftDoor::lift_name() const
{
  return _pimpl->lift_name;
}

//==============================================================================
auto Graph::Lane::LiftDoor::lift_name(std::string name) -> LiftDoor&
{
  _pimpl->lift_name = std::move(name);
  return *this;
}

//==============================================================================
const std::string& Graph::Lane::LiftDoor::floor_name() const
{
  return _pimpl->floor_name;
}

//==============================================================================
auto Graph::Lane::LiftDoor::floor_name(std::string name) -> LiftDoor&
{
  _pimpl->floor_name = std::move(name);
  return *this;
}

//==============================================================================
Duration Graph::Lane::LiftDoor::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
auto Graph::Lane::LiftDoor::duration(Duration duration_) -> LiftDoor&
{
  _pimpl->duration = duration_;
  return *this;
}

//==============================================================================
class Graph::Lane::LiftMove::Implementation
{
public:

  std::string lift_name;

  std::string destination_floor;

  Duration duration;

};

//==============================================================================
Graph::Lane::LiftMove::LiftMove(
    std::string lift_name,
    std::string destination_floor_name,
    Duration duration)
: _pimpl(rmf_utils::make_impl<Implementation>(
           Implementation{
             std::move(lift_name),
             std::move(destination_floor_name),
             duration
           }))
{
  // Do nothing
}

//==============================================================================
const std::string& Graph::Lane::LiftMove::lift_name() const
{
  return _pimpl->lift_name;
}

//==============================================================================
auto Graph::Lane::LiftMove::lift_name(std::string name) -> LiftMove&
{
  _pimpl->lift_name = std::move(name);
  return *this;
}

//==============================================================================
const std::string& Graph::Lane::LiftMove::destination_floor() const
{
  return _pimpl->destination_floor;
}

//==============================================================================
auto Graph::Lane::LiftMove::destination_floor(std::string name) -> LiftMove&
{
  _pimpl->destination_floor = std::move(name);
  return *this;
}

//==============================================================================
Duration Graph::Lane::LiftMove::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
auto Graph::Lane::LiftMove::duration(Duration duration) -> LiftMove&
{
  _pimpl->duration = duration;
  return *this;
}

namespace {
//==============================================================================
template<
    typename EventT,
    void (Graph::Lane::Executor::*callback)(const EventT&)>
class TemplateEvent : public Graph::Lane::Event
{
public:

  using This = TemplateEvent<EventT, callback>;

  TemplateEvent(EventT event)
  : _event(std::move(event))
  {
    // Do nothing
  }

  Duration duration() const final
  {
    return _event.duration();
  }

  void execute(Graph::Lane::Executor& executor) const final
  {
    (executor.*callback)(_event);
  }

  static std::unique_ptr<Event> make(EventT _event)
  {
    return std::make_unique<This>(std::move(_event));
  }

  std::unique_ptr<Event> clone() const final
  {
    return make(_event);
  }

private:

  EventT _event;

};
} // anonymous namespace

//==============================================================================
auto Graph::Lane::Event::make(DoorOpen open) -> std::unique_ptr<Event>
{
  return TemplateEvent<DoorOpen, &Executor::door_open>::make(
        std::move(open));
}

//==============================================================================
auto Graph::Lane::Event::make(DoorClose close) -> std::unique_ptr<Event>
{
  return TemplateEvent<DoorClose, &Executor::door_close>::make(
        std::move(close));
}

//==============================================================================
auto Graph::Lane::Event::make(LiftDoorOpen open) -> std::unique_ptr<Event>
{
  return TemplateEvent<LiftDoorOpen, &Executor::lift_door_open>::make(
        std::move(open));
}

//==============================================================================
auto Graph::Lane::Event::make(LiftDoorClose close) -> std::unique_ptr<Event>
{
  return TemplateEvent<LiftDoorClose, &Executor::lift_door_close>::make(
        std::move(close));
}

//==============================================================================
auto Graph::Lane::Event::make(LiftMove move) -> std::unique_ptr<Event>
{
  return TemplateEvent<LiftMove, &Executor::lift_move>::make(
        std::move(move));
}

//==============================================================================
class Graph::Lane::Node::Implementation
{
public:

  std::size_t waypoint;

  struct Constraints
  {
    Constraints(
        std::unique_ptr<Event> _event,
        std::unique_ptr<OrientationConstraint> _orientation,
        std::unique_ptr<VelocityConstraint> _velocity)
      : event(std::move(_event)),
        orientation(std::move(_orientation)),
        velocity(std::move(_velocity))
    {
      // Do nothing
    }

    std::unique_ptr<Event> event;
    std::unique_ptr<OrientationConstraint> orientation;
    std::unique_ptr<VelocityConstraint> velocity;

    Constraints(const Constraints& other)
    {
      *this = other;
    }

    Constraints& operator=(const Constraints& other)
    {
      if(other.event)
        event = other.event->clone();
      else
        event = nullptr;

      if(other.orientation)
        orientation = other.orientation->clone();
      else
        orientation = nullptr;

      if(other.velocity)
        velocity = other.velocity->clone();
      else
        velocity = nullptr;

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
    std::unique_ptr<Event> event,
    std::unique_ptr<OrientationConstraint> orientation,
    std::unique_ptr<VelocityConstraint> velocity)
: _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               waypoint_index,
               Implementation::Constraints(
                std::move(event),
                std::move(orientation),
                std::move(velocity))}))
{
  // Do nothing
}

//==============================================================================
Graph::Lane::Node::Node(
    std::size_t waypoint_index,
    std::unique_ptr<OrientationConstraint> orientation)
: _pimpl(rmf_utils::make_impl<Implementation>(
           Implementation{
             waypoint_index,
             Implementation::Constraints(
              nullptr,
              std::move(orientation),
              nullptr)
           }))
{
  // Do nothing
}

//==============================================================================
std::size_t Graph::Lane::Node::waypoint_index() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Graph::Lane::Node::event() const -> const Event*
{
  return _pimpl->constraints.event.get();
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
Graph::Lane::Lane()
{
  // Do nothing
}

//==============================================================================
Graph::Graph()
  : _pimpl(rmf_utils::make_impl<Implementation>())
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
auto Graph::get_lane(const std::size_t index) -> Lane&
{
  return _pimpl->lanes.at(index);
}

//==============================================================================
auto Graph::get_lane(const std::size_t index) const -> const Lane&
{
  return _pimpl->lanes.at(index);
}

//==============================================================================
std::size_t Graph::num_lanes() const
{
  return _pimpl->lanes.size();
}

} // namespace avg
} // namespace rmf_traffic
