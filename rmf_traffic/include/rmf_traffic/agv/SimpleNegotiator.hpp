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

#ifndef RMF_TRAFFIC__AGV__NEGOTIATOR_HPP
#define RMF_TRAFFIC__AGV__NEGOTIATOR_HPP

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Negotiator.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
/// A simple implementation of the schedule::Negotiator class. It uses an
/// agv::Planner to try to find a solution that fits on the negotiation table.
class SimpleNegotiator : public schedule::Negotiator
{
public:

  /// A class to specify user-defined options for the Negotiator.
  class Options
  {
  public:

    static constexpr double DefaultMaxCostLeeway = 1.5;
    static constexpr double DefaultMinCostThreshold = 30.0;

    using ApprovalCallback =
      std::function<Responder::UpdateVersion(rmf_traffic::agv::Plan)>;

    /// Constructor
    ///
    /// \param[in] approval_cb
    ///   The callback that will be triggered if the proposal is approved.
    ///
    /// \param[in] maximum_cost_leeway
    ///   The initial cost estimate for each planning attempt will be multiplied
    ///   by this factor to determine the maximum cost estimate that will be
    ///   allowed for a plan before giving up.
    ///
    /// \param[in] maximum_alts
    ///   The maximum number of alternatives to produce when rejecting a
    ///   proposal from another negotiator.
    ///
    /// \param[in] min_hold_time
    ///   The minimum amount of time that the planner should spend waiting at
    ///   holding points. See Planner::Options for more information.
    Options(
      ApprovalCallback approval_cb = nullptr,
      std::shared_ptr<const bool> interrupt_flag = nullptr,
      std::optional<double> maximum_cost_leeway = DefaultMaxCostLeeway,
      std::optional<std::size_t> maximum_alts = std::nullopt,
      Duration min_hold_time = Planner::Options::DefaultMinHoldingTime);

    /// Set the approval callback
    // TODO(MXG): The approval_callback option needs to be unit tested
    Options& approval_callback(ApprovalCallback cb);

    /// Set the interrupt flag
    Options& interrupt_flag(std::shared_ptr<const bool> flag);

    /// Get the interrupt flag
    const std::shared_ptr<const bool>& interrupt_flag() const;

    /// Set the maximum cost leeway
    Options& maximum_cost_leeway(std::optional<double> leeway);

    /// Get the maximum cost leeway
    std::optional<double> maximum_cost_leeway() const;

    /// Set the minimum cost threshold. When this and maximum_cost_leeway are
    /// both set, the maximum cost estimate will be chosen by
    /// std::max(
    ///   minimum_cost_threshold,
    ///   initial_cost_estimate * maximum_cost_leeway
    /// )
    ///
    /// By default, this is DefaultMinCostThreshold.
    Options& minimum_cost_threshold(std::optional<double> cost);

    /// Get the minimum cost threshold.
    std::optional<double> minimum_cost_threshold() const;

    /// Set the maximum cost threshold. When this is set, the cost will not be
    /// allowed to exceed it, even if the maximum cost leeway would allow it.
    /// By default, this is nullopt.
    Options& maximum_cost_threshold(std::optional<double> cost);

    /// Get the maximum cost threshold.
    std::optional<double> maximum_cost_threshold() const;

    Options& maximum_alternatives(rmf_utils::optional<std::size_t> num);

    std::optional<std::size_t> maximum_alternatives() const;

    /// Set the minimum amount of time to spend waiting at holding points
    Options& minimum_holding_time(Duration holding_time);

    /// Get the minimum amount of time to spend waiting at holding points
    Duration minimum_holding_time() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Constructor
  ///
  /// \param[in] start
  ///   The desired start for the plan.
  ///
  /// \param[in] goal
  ///   The desired goal for the plan.
  ///
  /// \param[in] planner_configuration
  ///   The configuration that will be used by the planner underlying this
  ///   Negotiator.
  ///
  /// \param[in] options
  ///   Additional options that will be used by the Negotiator.
  SimpleNegotiator(
    Planner::Start start,
    Planner::Goal goal,
    Planner::Configuration planner_configuration,
    Options options = Options());

  /// Constructor
  ///
  /// \param[in] start
  ///   A set of starts that can be used.
  ///
  /// \param[in] goal
  ///   The desired goal for the plan.
  ///
  /// \param[in] planner_configuration
  ///   The configuration that will be used by the planner underlying this
  ///   Negotiator.
  ///
  /// \param[in] options
  ///   Additional options that will be used by the Negotiator.
  SimpleNegotiator(
    std::vector<Planner::Start> starts,
    Planner::Goal goal,
    Planner::Configuration planner_configuration,
    Options options = Options());

  // TODO(MXG): Offer a constructor that accepts a Planner instance to benefit
  // from the cached heuristics.

  // Documentation inherited
  void respond(
    const schedule::Negotiation::Table::ViewerPtr& table_viewer,
    const ResponderPtr& responder) final;

  // TODO(MXG): How should we implement fallback behaviors when a different
  // negotiator rejects our proposal?

  class Implementation;
  class Debug;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__NEGOTIATOR_HPP
