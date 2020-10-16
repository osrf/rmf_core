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

#include <rclcpp/node.hpp>
#include <rmf_utils/optional.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/bidding/Bidding.hpp>
#include <rmf_task_ros2/bidding/Nomination.hpp>

namespace rmf_task_ros2 {
namespace bidding {
//==============================================================================

using BiddingTaskPtr = std::shared_ptr<BiddingTask>; // toberemoved

class Auctioneer : public std::enable_shared_from_this<Auctioneer>
{
public:
  /// Create an instance of the Auctioneer. Handling all the bidding mechanism
  ///
  /// \param[in] node
  ///   ros2 node which will manage the bidding
  ///
  /// \sa make()
  static std::shared_ptr<Auctioneer> make(
    const std::shared_ptr<rclcpp::Node>& node);

  /// Start a bidding process
  ///
  /// \param[in] bidding_task
  ///   The task to bid
  void start_bidding(const BiddingTask& bidding_task);

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

  // /// Provide a custom evaluator which will be used to choose the best bid
  // void select_evaluator(const Evaluator& evaluator);

  class Implementation;

private:
  Auctioneer();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__AUCTIONEER__NODE_HPP
