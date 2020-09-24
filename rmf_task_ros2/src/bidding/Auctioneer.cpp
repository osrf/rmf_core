
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

#include <rmf_task_ros2/bidding/Auctioneer.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
 
Auctioneer::Auctioneer(std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();
  double timeout_sec = 2.0; 
  _bidding_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(timeout_sec));


  _bid_notice_pub = _node->create_publisher<BidNotice>(
    rmf_task_ros2::BidNoticeTopicName, dispatch_qos);

  _bid_proposal_sub = _node->create_subscription<BidProposal>(
    rmf_task_ros2::BidProposalTopicName, dispatch_qos,
    [&](const BidProposal::UniquePtr msg)
    {
      this->receive_proposal(*msg);
    });
  
  _timer = _node->create_wall_timer(std::chrono::milliseconds(500), [&]()
    {
      this->check_bidding_process();
    });
};

/// Start a bidding process
void Auctioneer::start_bidding(const BiddingTask& bidding_task)
{
  _queue_bidding_tasks.push_back(bidding_task);  
  std::cout << "\n Start Task Bidding for task_id: " 
            << bidding_task.task_id << std::endl;

  // todo: identify potential bidders

  // Populate notice msg with queue_task here
  BidNotice notice_msg;
  notice_msg.task_id = bidding_task.task_id;
  notice_msg.itinerary = bidding_task.itinerary;
  notice_msg.submission_time = _node->now();
  notice_msg.fleet_names = bidding_task.bidders;
  _bid_notice_pub->publish(notice_msg);
};

/// callback when a bid is completed
void bidding_result_callback(
    std::function<void(const Submission& winner)> result_callback)
{
  // init
};

//==============================================================================
// private zone

void Auctioneer::receive_proposal(const BidProposal& msg)
{
  std::cout << " Receive Bidding proposal for task_id: " 
            << msg.task_id << std::endl;
  
  auto task_it = std::find_if(
    _queue_bidding_tasks.begin(), _queue_bidding_tasks.end(),
    [&](const BiddingTask& task){ return task.task_id == msg.task_id;});

  // check if bidding task is "mine"
  if (task_it == _queue_bidding_tasks.end())
    return;

  // add proposal to nominees list. //todo remove
  Nomination::Nominee nominee;

  task_it->nominees->push_back(nominee);
  // assert(task_it->bidders.size() < task_it->nominees->size());

  // check if all bidders' proposals are received
  if ( task_it->bidders.size() != task_it->nominees->size())
    return;
}

void Auctioneer::check_bidding_process()
{
  // check if bidding task has reached timeout... sad
  auto task_it = std::find_if(
    _queue_bidding_tasks.begin(), _queue_bidding_tasks.end(),
    [&](const BiddingTask& task)
    { 
      auto duration = std::chrono::steady_clock::now() - task.start_time;
      return duration >= *_bidding_timeout; 
    });
  
  if (task_it == _queue_bidding_tasks.end())
    return;
  
  // todo: if nominees are not empty, trigger Nomination

  std::cout << " Timeout reached! ready to remove task_id: " 
            << task_it->task_id << std::endl;
  _queue_bidding_tasks.erase(task_it);
}

} // namespace bidding
} // namespace rmf_task_ros2
