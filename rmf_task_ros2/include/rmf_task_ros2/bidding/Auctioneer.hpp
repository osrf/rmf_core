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

// Skeleton for Auctioneer

#ifndef RMF_TASK_ROS2__AUCTIONEER__NODE_HPP
#define RMF_TASK_ROS2__AUCTIONEER__NODE_HPP

#include <queue>
#include <rclcpp/node.hpp>
#include <rmf_utils/optional.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/bidding/Bidding.hpp>

namespace rmf_task_ros2 {
namespace bidding {
//==============================================================================

class Auctioneer : public std::enable_shared_from_this<Auctioneer>
{
public:
  /// Create an instance of the Auctioneer. Handling all the bidding mechanism
  ///
  /// \param[in] node
  ///   ros2 node which will manage the bidding
  ///
  /// \param[in] sequential
  ///   bid notice is sent sequentially or in async, default is sequential
  ///
  /// \sa make()
  static std::shared_ptr<Auctioneer> make(
    const std::shared_ptr<rclcpp::Node>& node,
    const bool sequential = true);

  /// Start a bidding process with a input task. If sequenctial, this will be
  /// added in a queue, else, a bidding process will start instantly.
  ///
  /// \param[in] bid_notice
  ///   bidding task, task which will call for bid
  void start_bidding(const BidNotice& bid_notice);

  /// callback which will provide the winner when a bid is concluded
  ///
  /// \param[in] task_id
  ///   bidding task id
  ///
  /// \param[in] winner
  ///   single winner from all submissions. nullopt if non
  using BiddingResultCallback =
    std::function<void( const TaskID& task_id,
      const rmf_utils::optional<Submission> winner)>;

  /// Provide a callback fn which will be called when a bid is concluded
  ///
  /// \param[in] result_callback
  ///   This fn will be called when a bidding result is derived
  void receive_bidding_result(BiddingResultCallback result_callback);

  /// A pure abstract interface class for the auctioneer to choose the best
  /// choosing the best submissions.
  class Evaluator
  {
  public:

    /// Given a list of submissions, choose the one that is the "best". It is up to
    /// the implementation of the Evaluator to decide how to rank.
    virtual std::size_t choose(const Submissions& submissions) const = 0;

    virtual ~Evaluator() = default;
  };

  /// Provide a custom evaluator which will be used to choose the best bid
  /// If no selection is given, Default is: LeastFleetDiffCostEvaluator
  ///
  /// \param[in] evaluator
  void select_evaluator(std::shared_ptr<Evaluator> evaluator);

  /// Get the best winner from all submissions
  ///
  /// \param[in] submissions
  ///
  /// \return Winner, nullopt if no winner
  rmf_utils::optional<Submission> evaluate(const Submissions& submissions);

  class Implementation;

private:
  Auctioneer();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class LeastFleetDiffCostEvaluator : public Auctioneer::Evaluator
{
  std::size_t choose(const Submissions& submissions) const final;
};

//==============================================================================
class LeastFleetCostEvaluator : public Auctioneer::Evaluator
{
  std::size_t choose(const Submissions& submissions) const final;
};

//==============================================================================
class QuickestFinishEvaluator : public Auctioneer::Evaluator
{
  std::size_t choose(const Submissions& submissions) const final;
};

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__AUCTIONEER__NODE_HPP
