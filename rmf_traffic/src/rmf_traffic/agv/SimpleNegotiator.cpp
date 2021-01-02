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

#include <rmf_traffic/agv/SimpleNegotiator.hpp>
#include <rmf_traffic/agv/Rollout.hpp>
#include <rmf_traffic/agv/debug/debug_Negotiator.hpp>

#include <deque>
#include <iostream>

namespace rmf_traffic {
namespace agv {

//==============================================================================
// This line tells the linker to take care of defining the value of this field
// inside of this translation unit.
const double SimpleNegotiator::Options::DefaultMaxCostLeeway;

//==============================================================================
class SimpleNegotiator::Options::Implementation
{
public:

  ApprovalCallback approval_cb;
  std::shared_ptr<const bool> interrupt_flag;
  std::optional<double> maximum_cost_leeway;
  std::optional<std::size_t> maximum_alts;
  Duration minimum_holding_time;
  std::optional<double> minimum_cost_threshold = DefaultMinCostThreshold;
  std::optional<double> maximum_cost_threshold = std::nullopt;

  static ApprovalCallback& get_approval_cb(Options& options)
  {
    return options._pimpl->approval_cb;
  }

};

//==============================================================================
SimpleNegotiator::Options::Options(ApprovalCallback approval_cb,
    std::shared_ptr<const bool> interrupt_flag,
    rmf_utils::optional<double> maximum_cost_leeway,
    rmf_utils::optional<std::size_t> maximum_alts,
    Duration min_hold_time)
: _pimpl(rmf_utils::make_impl<Implementation>(
           Implementation{
             std::move(approval_cb),
             std::move(interrupt_flag),
             maximum_cost_leeway,
             maximum_alts,
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
auto SimpleNegotiator::Options::interrupt_flag(
    std::shared_ptr<const bool> flag) -> Options&
{
  _pimpl->interrupt_flag = std::move(flag);
  return *this;
}

//==============================================================================
const std::shared_ptr<const bool>&
SimpleNegotiator::Options::interrupt_flag() const
{
  return _pimpl->interrupt_flag;
}

//==============================================================================
auto SimpleNegotiator::Options::maximum_cost_leeway(
    rmf_utils::optional<double> leeway) -> Options&
{
  _pimpl->maximum_cost_leeway = leeway;
  return *this;
}

//==============================================================================
rmf_utils::optional<double>
SimpleNegotiator::Options::maximum_cost_leeway() const
{
  return _pimpl->maximum_cost_leeway;
}

//==============================================================================
auto SimpleNegotiator::Options::minimum_cost_threshold(
    const std::optional<double> cost) -> Options&
{
  _pimpl->minimum_cost_threshold = cost;
  return *this;
}

//==============================================================================
std::optional<double> SimpleNegotiator::Options::minimum_cost_threshold() const
{
  return _pimpl->minimum_cost_threshold;
}

//==============================================================================
auto SimpleNegotiator::Options::maximum_cost_threshold(
    const std::optional<double> cost) -> Options&
{
  _pimpl->maximum_cost_threshold = cost;
  return *this;
}

//==============================================================================
std::optional<double> SimpleNegotiator::Options::maximum_cost_threshold() const
{
  return _pimpl->maximum_cost_threshold;
}

//==============================================================================
auto SimpleNegotiator::Options::maximum_alternatives(
    rmf_utils::optional<std::size_t> num) -> Options&
{
  _pimpl->maximum_alts = num;
  return *this;
}

//==============================================================================
rmf_utils::optional<std::size_t>
SimpleNegotiator::Options::maximum_alternatives() const
{
  return _pimpl->maximum_alts;
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
  Planner::Options planner_options;
  Planner planner;
  Options negotiator_options;

  bool debug_print = false;

  Implementation(
    std::vector<Planner::Start> starts_,
    Planner::Goal goal_,
    Planner::Configuration configuration_,
    Options options_)
  : starts(std::move(starts_)),
    goal(std::move(goal_)),
    planner_options(nullptr, options_.minimum_holding_time()),
    planner(std::move(configuration_), planner_options),
    negotiator_options(std::move(options_))
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
      std::move(options))))
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
           std::move(options)))
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
  assert(route.trajectory().size() > 0);
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
  assert(route.trajectory().size() > 0);
  for (auto it = ++route.trajectory().begin(); it
       != route.trajectory().end(); ++it)
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
  if (itinerary.empty())
  {
    std::cout << "No plan needed!" << std::endl;
  }
  else
  {
    auto start_time = print_start(*itinerary.front());
    for (const auto& r : itinerary)
      print_route(*r, start_time);

    std::cout << "(end)" << std::endl;
  }
}

//==============================================================================
inline void print_itinerary(const std::vector<rmf_traffic::Route>& itinerary)
{
  if (itinerary.empty())
  {
    std::cout << "No plan needed!" << std::endl;
  }
  else
  {
    auto start_time = print_start(itinerary.front());
    for (const auto& r : itinerary)
      print_route(r, start_time);

    std::cout << "(end)" << std::endl;
  }
}


////==============================================================================
class AlternativesTracker
{
public:

  AlternativesTracker(std::vector<schedule::ParticipantId> participants)
    : _ordered_participants(std::move(participants))
  {
    std::sort(_ordered_participants.begin(), _ordered_participants.end());
  }

  bool skip(const schedule::Negotiation::VersionedKeySequence& sequence)
  {
    if (_ordered_participants.empty())
      return false;

    bool new_sequence = true;

    const schedule::ParticipantId last = _ordered_participants.back();

    Element* n = &_root;
    for (const auto p : _ordered_participants)
    {
      for (const auto& s : sequence)
      {
        if (s.participant == p)
        {
          const auto insertion = n->next.insert({s.version, nullptr});
          const bool inserted = insertion.second;
          new_sequence = inserted;
          auto& next = insertion.first->second;
          if (inserted && p != last)
            next = std::make_unique<Element>();

          n = next.get();
          break;
        }
      }
    }

    return !new_sequence;
  }

private:

  struct Element
  {
    std::unordered_map<schedule::Version, std::unique_ptr<Element>> next;
  };

  std::vector<schedule::ParticipantId> _ordered_participants;
  Element _root;

};

} // anonymous namespace

//==============================================================================
void SimpleNegotiator::respond(
  const schedule::Negotiation::Table::ViewerPtr& table_viewer,
  const ResponderPtr& responder)
{
  const auto& profile =
    _pimpl->planner.get_configuration().vehicle_traits().profile();
  NegotiatingRouteValidator::Generator rv_generator(table_viewer, profile);

  const auto& alternative_sets = rv_generator.alternative_sets();

  auto options = _pimpl->planner_options;

  const auto maximum_cost_leeway =
      _pimpl->negotiator_options.maximum_cost_leeway();
  const auto max_alts = _pimpl->negotiator_options.maximum_alternatives();

  const auto minimum_cost_threshold =
      _pimpl->negotiator_options.minimum_cost_threshold();

  const auto maximum_cost_threshold =
      _pimpl->negotiator_options.maximum_cost_threshold();

  std::deque<rmf_utils::clone_ptr<NegotiatingRouteValidator>> validators;
  validators.push_back(
        rmf_utils::make_clone<NegotiatingRouteValidator>(rv_generator.begin()));

  rmf_utils::optional<schedule::Negotiation::Alternatives> alternatives;
  rmf_utils::optional<std::vector<schedule::ParticipantId>> best_blockers;

  if (_pimpl->debug_print)
  {
    std::cout << "Responding to [";
    for (const auto& p : table_viewer->sequence())
      std::cout << " " << p.participant << ":" << p.version;
    std::cout << " ]" << std::endl;
  }

  AlternativesTracker tracker(rv_generator.alternative_sets());

  const auto interrupt_flag = _pimpl->planner_options.interrupt_flag();
  while (!validators.empty() && !(interrupt_flag && *interrupt_flag))
  {
    const auto validator = std::move(validators.front());
    validators.pop_front();

    if (validator->end())
      continue;

    if (tracker.skip(validator->alternatives()))
      continue;

    if (_pimpl->debug_print)
    {
      if (validator->alternatives().empty())
      {
        std::cout << "Negotiating without rollouts" << std::endl;
      }
      else
      {
        std::cout << "Negotiating with rollouts:";
        for (const auto& r : validator->alternatives())
        {
          std::cout << " (" << r.participant << ":" << r.version
                    << "/" << table_viewer->alternatives().at(r.participant)->size()
                    << ")";
        }
        std::cout << std::endl;
      }
    }

    options.validator(validator);
    auto plan = _pimpl->planner.setup(_pimpl->starts, _pimpl->goal, options);
    const double initial_cost_estimate = *plan.cost_estimate();
    std::optional<double> cost_limit;
    if (maximum_cost_leeway.has_value())
    {
      cost_limit = maximum_cost_leeway.value() * initial_cost_estimate;
      if (minimum_cost_threshold.has_value())
        cost_limit = std::max(*minimum_cost_threshold, *cost_limit);
    }

    if (maximum_cost_threshold.has_value())
    {
      if (cost_limit.has_value())
        cost_limit = std::min(*cost_limit, *maximum_cost_threshold);
      else
        cost_limit = *maximum_cost_threshold;
    }

    plan.options().maximum_cost_estimate(cost_limit);

    plan.resume();

    if (plan)
    {
      if (_pimpl->debug_print)
      {
        const double cost = plan->get_cost();
        std::cout << "Maximum cost leeway factor: " << cost/initial_cost_estimate
                  << std::endl;

        std::cout << "Submitting:\n";
        print_itinerary(plan->get_itinerary());
      }

      Responder::ApprovalCallback responder_approval_cb;
      auto options_approval_callback =
          Options::Implementation::get_approval_cb(_pimpl->negotiator_options);
      if (options_approval_callback)
      {
        responder_approval_cb = [
              approval_cb = options_approval_callback,
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
      return responder->submit(plan->get_itinerary(), responder_approval_cb);
    }

    if (_pimpl->debug_print)
    {
      if (plan.cost_estimate())
      {
        std::cout << " ======= Failed ratio: "
                  << (*plan.cost_estimate())/initial_cost_estimate
                  << std::endl;
      }

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
    alternatives = rollout.expand(
          parent_id, std::chrono::seconds(15), options, max_alts);

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
        std::cout << "Rolled out [" << alternatives->size() << "] alternatives:"
                  << std::endl;

        std::size_t count = 0;
        for (const auto& itinerary : *alternatives)
        {
          if (++count > 5)
            break;

          print_itinerary(itinerary);
        }
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
    return responder->reject(*alternatives);
  }

  if (best_blockers)
  {
    if (_pimpl->debug_print)
    {
      std::cout << " >>>>> Forfeiting with blockers" << std::endl;
    }
    return responder->forfeit(*best_blockers);
  }

  if (_pimpl->debug_print)
  {
    std::cout << " >>>>> Forfeiting with NO BLOCKERS" << std::endl;
  }

  // This would be suspicious. How could the planning fail without any blockers?
  responder->forfeit({});
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
