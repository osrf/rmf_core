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
#include <rmf_traffic/agv/Rollout.hpp>

#include <deque>

#include <iostream>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class SimpleNegotiator::Options::Implementation
{
public:

  Duration minimum_holding_time;

};

//==============================================================================
SimpleNegotiator::Options::Options(Duration min_hold_time)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{min_hold_time}))
{
  // Do nothing
}

//==============================================================================
auto SimpleNegotiator::Options::minimum_holding_time(Duration value) -> Options&
{
  _pimpl->minimum_holding_time = value;
  return *this;
}

//==============================================================================
Duration SimpleNegotiator::Options::minimum_holding_time() const
{
  return _pimpl->minimum_holding_time;
}

//==============================================================================
class SimpleNegotiator::Implementation
{
public:

  std::vector<Planner::Start> starts;
  Planner::Goal goal;
  Planner::Options options;
  Planner planner;

  Implementation(
    std::vector<Planner::Start> starts_,
    Planner::Goal goal_,
    Planner::Configuration configuration_,
    Planner::Options options_)
  : starts(std::move(starts_)),
    goal(std::move(goal_)),
    options(std::move(options_)),
    planner(std::move(configuration_), options)
  {
    // Do nothing
  }

};

//==============================================================================
SimpleNegotiator::SimpleNegotiator(
  Planner::Start start,
  Planner::Goal goal,
  Planner::Configuration planner_configuration,
  const Options& options)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation(
      {std::move(start)},
      std::move(goal),
      std::move(planner_configuration),
      Planner::Options(nullptr, options.minimum_holding_time()))))
{
  // Do nothing
}

//==============================================================================
SimpleNegotiator::SimpleNegotiator(
  std::vector<Planner::Start> starts,
  Planner::Goal goal,
  Planner::Configuration planner_configuration,
  const Options& options)
: _pimpl(rmf_utils::make_impl<Implementation>(
           std::move(starts),
           std::move(goal),
           std::move(planner_configuration),
           Planner::Options(nullptr, options.minimum_holding_time())))
{
  // Do nothing
}

namespace {
//==============================================================================
bool contains(
    const std::vector<schedule::ParticipantId>& ids,
    schedule::ParticipantId id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}
} // anonymous namespace

//==============================================================================
void SimpleNegotiator::respond(
  std::shared_ptr<const schedule::Negotiation::Table> table,
  const Responder& responder,
  const bool* interrupt_flag)
{
  const auto& profile =
    _pimpl->planner.get_configuration().vehicle_traits().profile();
  NegotiatingRouteValidator::Generator rv_generator(*table, profile);

  const auto& alternative_sets = rv_generator.alternative_sets();

  auto options = _pimpl->options;
  options.interrupt_flag(interrupt_flag);

  std::deque<rmf_utils::clone_ptr<NegotiatingRouteValidator>> validators;
  validators.push_back(
        rmf_utils::make_clone<NegotiatingRouteValidator>(rv_generator.begin()));

  rmf_utils::optional<schedule::Negotiation::Alternatives> alternatives;
  rmf_utils::optional<std::vector<schedule::ParticipantId>> best_blockers;

  while (!validators.empty() && !(interrupt_flag && *interrupt_flag))
  {
    const auto validator = std::move(validators.front());
    validators.pop_front();

    if (!validator)
    {
      std::cout << " ==== VALIDATOR ENDED" << std::endl;
      continue;
    }

    std::cout << " ==== Examining for rollouts: ";
    for (const auto& r : validator->rollouts())
      std::cout << "(" << r.participant << ": " << r.alternative << ") ";
    std::cout << std::endl;

    options.validator(validator);

    std::cout << " ==== About to start planning" << std::endl;

    if (validator->rollouts().size() > 0)
    {
      std::cout << "ARE WE ABOUT TO RETURN??" << std::endl;
    }

    const auto plan = _pimpl->planner.plan(
          _pimpl->starts, _pimpl->goal, options);
    std::cout << " ==== Finished planning" << std::endl;

    if (plan)
    {
      std::cout << " ==== Submitting:" << std::endl;

      const auto start_time = *plan->get_itinerary().front().trajectory().start_time();
      std::cout << "(start) --> ";
      for (const auto& r : plan->get_itinerary())
      {
        const auto rel_time = r.trajectory().front().time() - start_time;
        std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
                  << r.trajectory().front().position().transpose() << ") --> ";
      }
      const auto rel_time = plan->get_itinerary().back().trajectory().back().time() - start_time;
      std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
                << plan->get_itinerary().back().trajectory().back().position().transpose()
                << ") --> (end)\n" << std::endl;

      return responder.submit(plan->get_itinerary());
    }

    std::cout << " ==== Attempt failed" << std::endl;

    const auto& blockers = plan.blockers();
    if (!best_blockers)
      best_blockers = blockers;

    for (const auto r : alternative_sets)
    {
      if (contains(blockers, r))
      {
        std::cout << " ==== Pushing back validator for next [" << r << "] alternative" << std::endl;
        validators.push_back(
              rmf_utils::make_clone<NegotiatingRouteValidator>(
                validator->next(r)));
      }
    }

    const auto parent = table->parent();
    if (!parent)
      continue;

    const auto parent_id = parent->participant();
    if (!contains(blockers, parent_id))
    {
      // If the parent participant is not a blocker, then there is no point in
      // doing a rollout + rejection.
      continue;
    }

    if (alternatives)
    {
      // We already have a rollout, so there's no point in computing a new one.
      continue;
    }

    validator->mask(parent_id);
    options.interrupt_flag(nullptr);
    options.validator(validator);

    Rollout rollout(plan);
    // TODO(MXG): Make the 30 seconds configurable
    alternatives = rollout.expand(parent_id, std::chrono::seconds(15), options);
    if (alternatives->empty())
    {
      std::cout << " ====== Empty rollout" << std::endl;
      // If there weren't actually any alternatives produced, then reset it.
      alternatives = rmf_utils::nullopt;
    }
    else
    {
      std::cout << " ====== [" << alternatives->size() << "] rollouts" << std::endl;
      if (alternatives->size() > 10)
        alternatives->resize(10);

      for (const auto& itinerary : *alternatives)
      {
        const auto start_time = *itinerary.front()->trajectory().start_time();
        std::cout << "(start) --> ";
        for (const auto& r : itinerary)
        {
          const auto rel_time = r->trajectory().front().time() - start_time;
          std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
                    << r->trajectory().front().position().transpose() << ") --> ";
        }

        const auto rel_time = itinerary.back()->trajectory().back().time() - start_time;
        std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
                  << itinerary.back()->trajectory().back().position().transpose()
                  << ") --> (end)\n" << std::endl;
      }
    }

    options.interrupt_flag(interrupt_flag);
  }

  std::cout << " ==== No plan found" << std::endl;

  if (alternatives)
    return responder.reject(*alternatives);

  std::cout << " ==== No alternatives found" << std::endl;

  if (best_blockers)
    return responder.forfeit(*best_blockers);

  std::cout << " ==== No blockers found" << std::endl;

  // This would be suspicious. How could the planning fail without any blockers?
  responder.forfeit({});
}

} // namespace agv
} // namespace rmf_traffic
