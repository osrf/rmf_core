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
#include <rmf_traffic/agv/debug/debug_Negotiator.hpp>

#include <deque>

#include <iostream>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class SimpleNegotiator::Options::Implementation
{
public:

  ApprovalCallback approval_cb;
  Duration minimum_holding_time;

  static ApprovalCallback&& move_approval_cb(Options&& options)
  {
    return std::move(options._pimpl->approval_cb);
  }

};

//==============================================================================
SimpleNegotiator::Options::Options(
    ApprovalCallback approval_cb,
    Duration min_hold_time)
: _pimpl(rmf_utils::make_impl<Implementation>(
           Implementation{
             std::move(approval_cb),
             min_hold_time
           }))
{
  // Do nothing
}

//==============================================================================
auto SimpleNegotiator::Options::approval_callback(
    ApprovalCallback approval_cb) -> Options&
{
  _pimpl->approval_cb = std::move(approval_cb);
  return *this;
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
  Options::ApprovalCallback approval_cb;

  bool debug_print = false;

  Implementation(
    std::vector<Planner::Start> starts_,
    Planner::Goal goal_,
    Planner::Configuration configuration_,
    Planner::Options options_,
    Options::ApprovalCallback approval_cb_)
  : starts(std::move(starts_)),
    goal(std::move(goal_)),
    options(std::move(options_)),
    planner(std::move(configuration_), options),
    approval_cb(std::move(approval_cb_))
  {
    // Do nothing
  }

};

//==============================================================================
SimpleNegotiator::SimpleNegotiator(
  Planner::Start start,
  Planner::Goal goal,
  Planner::Configuration planner_configuration,
  Options options)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation(
      {std::move(start)},
      std::move(goal),
      std::move(planner_configuration),
      Planner::Options(nullptr, options.minimum_holding_time()),
      Options::Implementation::move_approval_cb(std::move(options)))))
{
  // Do nothing
}

//==============================================================================
SimpleNegotiator::SimpleNegotiator(
  std::vector<Planner::Start> starts,
  Planner::Goal goal,
  Planner::Configuration planner_configuration,
  Options options)
: _pimpl(rmf_utils::make_impl<Implementation>(
           std::move(starts),
           std::move(goal),
           std::move(planner_configuration),
           Planner::Options(nullptr, options.minimum_holding_time()),
           Options::Implementation::move_approval_cb(std::move(options))))
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

//==============================================================================
inline rmf_traffic::Time print_start(const rmf_traffic::Route& route)
{
  std::cout << "(start) --> ";
  std::cout << "(" << 0.0 << "; "
            << route.trajectory().front().position().transpose()
            << ") --> ";

  return *route.trajectory().start_time();
}

//==============================================================================
inline void print_route(
    const rmf_traffic::Route& route,
    const rmf_traffic::Time start_time)
{
  for (auto it = ++route.trajectory().begin(); it != route.trajectory().end(); ++it)
  {
    const auto& wp = *it;
    if (wp.velocity().norm() > 1e-3)
      continue;

    const auto rel_time = wp.time() - start_time;
    std::cout << "(" << rmf_traffic::time::to_seconds(rel_time) << "; "
              << wp.position().transpose() << ") --> ";
  }
}

//==============================================================================
inline void print_itinerary(
    const rmf_traffic::schedule::Itinerary& itinerary)
{
  auto start_time = print_start(*itinerary.front());
  for (const auto& r : itinerary)
    print_route(*r, start_time);

  std::cout << "(end)\n" << std::endl;
}

//==============================================================================
inline void print_itinerary(const std::vector<rmf_traffic::Route>& itinerary)
{
  auto start_time = print_start(itinerary.front());
  for (const auto& r : itinerary)
    print_route(r, start_time);

  std::cout << "(end)\n" << std::endl;
}


////==============================================================================
//struct

} // anonymous namespace

//==============================================================================
void SimpleNegotiator::respond(
  const schedule::Negotiation::Table::ViewerPtr& table_viewer,
  const Responder& responder,
  const bool* interrupt_flag)
{
  const auto& profile =
    _pimpl->planner.get_configuration().vehicle_traits().profile();
  NegotiatingRouteValidator::Generator rv_generator(table_viewer, profile);

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

    if (validator->end())
      continue;

    if (_pimpl->debug_print)
    {
      if (validator->rollouts().empty())
        std::cout << "Negotiating without rollouts" << std::endl;
      else
      {
        std::cout << "Negotiating with rollouts:";
        for (const auto& r : validator->rollouts())
        {
          std::cout << " [" << r.participant << ":" << r.alternative
                    << "|" << table_viewer->alternatives().at(r.participant)->size()
                    << "]";
        }
        std::cout << std::endl;
      }
    }

    options.validator(validator);
    const auto plan = _pimpl->planner.plan(
          _pimpl->starts, _pimpl->goal, options);

    if (plan)
    {
      if (_pimpl->debug_print)
      {
        std::cout << "Submitting:\n";
        print_itinerary(plan->get_itinerary());
      }

      Responder::ApprovalCallback responder_approval_cb;
      if (_pimpl->approval_cb)
      {
        responder_approval_cb = [
              approval_cb = _pimpl->approval_cb,
              approved_plan = *plan
            ]() -> Responder::UpdateVersion
        {
          return approval_cb(std::move(approved_plan));
        };
      }

      if (_pimpl->debug_print)
      {
        std::cout << " >>>>> Submitting" << std::endl;
      }
      return responder.submit(plan->get_itinerary(), responder_approval_cb);
    }

    if (_pimpl->debug_print)
    {
      std::cout << "Failed to find a plan. Blocked by:";
      for (const auto p : plan.blockers())
        std::cout << " " << p;
      std::cout << std::endl;
    }

    const auto& blockers = plan.blockers();
    if (!best_blockers)
      best_blockers = blockers;

    for (const auto r : alternative_sets)
    {
      if (contains(blockers, r))
      {
        validators.push_back(
              rmf_utils::make_clone<NegotiatingRouteValidator>(
                validator->next(r)));
      }
    }

    const auto has_parent = table_viewer->parent_id();
    if (!has_parent)
      continue;

    const auto parent_id = *has_parent;
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

    if (_pimpl->debug_print)
    {
      std::cout << "Negotiation parent ["
                << parent_id << "] is a blocker" << std::endl;
    }

    validator->mask(parent_id);
    options.interrupt_flag(nullptr);
    options.validator(validator);
    const auto old_holding_time = options.minimum_holding_time();
    options.minimum_holding_time(std::chrono::seconds(5));

    Rollout rollout(plan);
    // TODO(MXG): Make the span configurable
//    alternatives = rollout.expand(parent_id, std::chrono::seconds(30), options);
    alternatives = rollout.expand(parent_id, std::chrono::seconds(15), options);
    if (alternatives->empty())
    {
      alternatives = rmf_utils::nullopt;
      if (_pimpl->debug_print)
      {
        std::cout << "Could not roll out any alternatives" << std::endl;
      }
    }
    else
    {
      if (_pimpl->debug_print)
      {
        std::cout << "Rolled out [" << alternatives->size() << "] alternatives"
                  << std::endl;
      }

      if (alternatives->size() > 10)
        alternatives->resize(10);

      if (_pimpl->debug_print)
      {
        for (const auto& itinerary : *alternatives)
          print_itinerary(itinerary);
      }
    }

    options.interrupt_flag(interrupt_flag);
    options.minimum_holding_time(old_holding_time);
  }

  if (alternatives)
  {
    if (_pimpl->debug_print)
    {
      std::cout << " >>>>> Rejecting" << std::endl;
    }
    return responder.reject(*alternatives);
  }

  if (best_blockers)
  {
    if (_pimpl->debug_print)
    {
      std::cout << " >>>>> Forfeiting with blockers" << std::endl;
    }
    return responder.forfeit(*best_blockers);
  }

  if (_pimpl->debug_print)
  {
    std::cout << " >>>>> Forfeiting with NO BLOCKERS" << std::endl;
  }
  // This would be suspicious. How could the planning fail without any blockers?
  responder.forfeit({});
}

//==============================================================================
SimpleNegotiator& SimpleNegotiator::Debug::enable_debug_print(
    SimpleNegotiator& negotiator)
{
  negotiator._pimpl->debug_print = true;
  return negotiator;
}

} // namespace agv
} // namespace rmf_traffic
