
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

std::shared_ptr<Auctioneer> Auctioneer::make(std::shared_ptr<rclcpp::Node> node)
{
  return std::shared_ptr<Auctioneer>(new Auctioneer(node));
}

Auctioneer::Auctioneer(std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
  const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

  _bid_notice_pub = _node->create_publisher<BidNotice>(
    rmf_task_ros2::BidNoticeTopicName, dispatch_qos);

  _bid_proposal_sub = _node->create_subscription<BidProposal>(
    rmf_task_ros2::BidProposalTopicName, dispatch_qos,
    [&](const BidProposal::UniquePtr msg)
    {
      this->receive_proposal(*msg);
    });
  
  _timer = _node->create_wall_timer(std::chrono::milliseconds(1000), [&]()
    {
      this->check_bidding_process();
    });
}

/// Start a bidding process
void Auctioneer::start_bidding(const BiddingTask& bidding_task)
{
  std::cout << "\n Add Bidding Task for task_id: " 
            << bidding_task.task_id << " to queue"<< std::endl;
  _queue_bidding_tasks[bidding_task.task_id] = 
    std::make_shared<BiddingTask>(bidding_task);

  // todo: think if this reallly sequencial
  BidNotice notice_msg = convert(bidding_task);
  notice_msg.submission_time = _node->now();
  _bid_notice_pub->publish(notice_msg);
}

void Auctioneer::receive_bidding_result(
    std::function<void(const Submission& winner)> result_callback)
{
  _bidding_result_callback = std::move(result_callback);
}

//==============================================================================
// private zone

void Auctioneer::receive_proposal(const BidProposal& msg)
{
  std::cout << " Receive Bidding proposal for task_id: " 
            << msg.task_id << std::endl;
  
  // check if bidding task is "mine"
  auto task_it = _queue_bidding_tasks.find(msg.task_id);
  if (task_it == _queue_bidding_tasks.end())
    return; // not found

  // add submited proposal to the current bidding task
  auto submission = convert(msg);
  _queue_bidding_tasks[msg.task_id]->submissions.push_back(submission);
}


void Auctioneer::check_bidding_process()
{
  // check if timeout is reached
  for( auto const& [id, bid_tsk] : _queue_bidding_tasks )
  {
    auto duration = std::chrono::steady_clock::now() - bid_tsk->submission_time;
    if (duration > bid_tsk->time_window )
      this->determine_winner(bid_tsk); 
    
    if (!bid_tsk->announce_all)
    {
      if ( bid_tsk->submissions.size() == bid_tsk->bidders.size())
        this->determine_winner(bid_tsk);
    }
  }
  std::cout << "Remaining: " << _queue_bidding_tasks.size() 
            << " bidding tasks" << std::endl;
}

void Auctioneer::determine_winner(BiddingTaskPtr bidding_task)
{ 
  // Nominate and Evaluate Here! TODO
  // Nomination task_nomination(task_it->nominees);
  // Nomination::Nominee chosen_estimate = 
  //   task_nomination.evaluate(QuickestFinishEvaluator());
  
  //todo if winner is nullopt
  Submission winner;
  std::cout << "Found winning Fleet Adapter: " 
            << winner.fleet_name << std::endl;
  
  // remove completed task from queue
  _queue_bidding_tasks.erase(bidding_task->task_id);
  
  // check if _bidding_result_callback fn is initailized
  if (!_bidding_result_callback)
    return;
  
  this->_bidding_result_callback(winner);
}

} // namespace bidding
} // namespace rmf_task_ros2
