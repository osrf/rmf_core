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

#include <rmf_traffic/agv/Negotiator.hpp>



#include <iostream>


namespace rmf_traffic {
namespace agv {

//==============================================================================
class Negotiator::Options::Implementation
{
public:

  Duration minimum_holding_time;

};

//==============================================================================
Negotiator::Options::Options(Duration min_hold_time)
  : _pimpl(rmf_utils::make_impl<Implementation>(Implementation{min_hold_time}))
{
  // Do nothing
}

//==============================================================================
auto Negotiator::Options::minimum_holding_time(Duration value) -> Options&
{
  _pimpl->minimum_holding_time = value;
  return *this;
}

//==============================================================================
Duration Negotiator::Options::minimum_holding_time() const
{
  return _pimpl->minimum_holding_time;
}

//==============================================================================
class Negotiator::Implementation
{
public:

  Planner::Start start;
  Planner::Goal goal;
  Planner::Options options;
  Planner planner;

  Implementation(
      Planner::Start start_,
      Planner::Goal goal_,
      Planner::Configuration configuration_,
      Planner::Options options_)
    : start(std::move(start_)),
      goal(std::move(goal_)),
      options(std::move(options_)),
      planner(std::move(configuration_), options)
  {
    // Do nothing
  }

};

//==============================================================================
Negotiator::Negotiator(
    Planner::Start start,
    Planner::Goal goal,
    Planner::Configuration planner_configuration,
    const Options& options)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             std::move(start),
             std::move(goal),
             std::move(planner_configuration),
             Planner::Options(nullptr, options.minimum_holding_time())))
{
  // Do nothing
}

//==============================================================================
void Negotiator::respond(
    std::shared_ptr<const schedule::Negotiation::Table> table,
    const Responder& responder,
    const bool* interrupt_flag) const
{
  const auto& profile =
      _pimpl->planner.get_configuration().vehicle_traits().profile();

  auto options = _pimpl->options;

  options.validator(
        rmf_utils::make_clone<NegotiatingRouteValidator>(*table, profile));

  options.interrupt_flag(interrupt_flag);

  const auto plan = _pimpl->planner.plan(_pimpl->start, _pimpl->goal, options);
  if (!plan)
  {
    // If we failed to produce a plan, then we should send a rejection
    responder.reject();
    return;
  }

  responder.submit(plan->get_itinerary());
}

//==============================================================================
class SimpleResponder::Implementation
{
public:

  std::shared_ptr<schedule::Negotiation> negotiation;
  schedule::ParticipantId for_participant;
  std::vector<schedule::ParticipantId> to_accommodate;

};

//==============================================================================
SimpleResponder::SimpleResponder(
    std::shared_ptr<schedule::Negotiation> negotiation,
    schedule::ParticipantId for_participant,
    std::vector<schedule::ParticipantId> to_accommodate)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(negotiation),
               for_participant,
               std::move(to_accommodate)
             }))
{
  // Do nothing
}

//==============================================================================
void SimpleResponder::submit(std::vector<Route> itinerary) const
{
  std::cout << "Getting submission from " << _pimpl->for_participant << std::endl;
  _pimpl->negotiation->submit(
        _pimpl->for_participant,
        _pimpl->to_accommodate,
        std::move(itinerary));
}

//==============================================================================
void SimpleResponder::reject() const
{
  std::cout << "Getting rejection from " << _pimpl->for_participant << std::endl;
  _pimpl->negotiation->reject(_pimpl->to_accommodate);
}

} // namespace agv
} // namespace rmf_traffic
