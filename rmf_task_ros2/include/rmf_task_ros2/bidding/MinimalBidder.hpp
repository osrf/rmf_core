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

#ifndef SRC__RMF_TASK_ROS2__BIDDER_HPP
#define SRC__RMF_TASK_ROS2__BIDDER_HPP

#include <set>

#include <rclcpp/node.hpp>
#include <rmf_utils/impl_ptr.hpp>

#include <rmf_task_ros2/StandardNames.hpp>
#include <rmf_task_ros2/TaskProfile.hpp>
#include <rmf_task_ros2/bidding/Bidding.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
// Skeleton for a TaskBidder

class MinimalBidder
{
public: 
  struct Profile
  {
    std::string bidder_name;
    std::set<TaskType> valid_tasks;
  };

  /// Create a bidder to bid for incoming task requests from Task Dispatcher
  ///
  /// \param[in] bidder profile
  /// \param[in] ros2 node
  static std::shared_ptr<MinimalBidder> make(
      std::shared_ptr<rclcpp::Node> node,
      const Profile& profile);
  
  /// Callback function which user provide a bid submission after receiving 
  /// a bid notice from the autioneer
  ///
  /// \param[in] bid notice msg
  /// \return submission of bid proposal
  using ParseSubmissionCallback =
      std::function<Submission(const BidNotice& notice)>;
  
  /// Create a call for bid callback instance
  ///
  /// \param[in] callback function to provide bid proposal
  void call_for_bid(ParseSubmissionCallback submission_cb);

  class Implementation;

private: 
  MinimalBidder();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace bidder
} // namespace rmf_task_ros2

#endif // SRC__RMF_TASK_ROS2__BIDDER_HPPHAHA
