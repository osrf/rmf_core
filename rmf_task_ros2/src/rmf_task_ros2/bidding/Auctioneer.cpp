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

#include <rmf_task_ros2/bidding/Bidding.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
class Auctioneer::Implementation
{
public:

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::TimerBase::SharedPtr timer;
  BiddingResultCallback bidding_result_callback;
  // Nomination::Evaluator _evaluator;
  // std::map<TaskID, BiddingTaskPtr> queue_bidding_tasks;
  
  struct BiddingTaskSubmissions
  {
    BiddingTask bidding_task;
    std::vector<bidding::Submission> submissions;
  };
  std::map<TaskID, BiddingTaskSubmissions> queue_bidding_tasks;

  using BidNoticePub = rclcpp::Publisher<BidNotice>;
  BidNoticePub::SharedPtr bid_notice_pub;

  using BidProposalSub = rclcpp::Subscription<BidProposal>;
  BidProposalSub::SharedPtr bid_proposal_sub;

  Implementation(const std::shared_ptr<rclcpp::Node>& node_)
  : node(node_)
  {
    const auto dispatch_qos = rclcpp::ServicesQoS().reliable();

    bid_notice_pub = node->create_publisher<BidNotice>(
      rmf_task_ros2::BidNoticeTopicName, dispatch_qos);

    bid_proposal_sub = node->create_subscription<BidProposal>(
      rmf_task_ros2::BidProposalTopicName, dispatch_qos,
      [&](const BidProposal::UniquePtr msg)
      {
        this->receive_proposal(*msg);
      });
    
    timer = node->create_wall_timer(std::chrono::milliseconds(1000), [&]()
      {
        this->check_bidding_process();
      });
  }

  /// Start a bidding process
  void start_bidding(const BiddingTask& bidding_task)
  {
    std::cout << "\n Add Bidding Task for task_id: " 
              << bidding_task.task_profile.task_id << " to queue"<< std::endl;
    queue_bidding_tasks[bidding_task.task_profile.task_id] = {bidding_task};

    // todo: think if this reallly sequencial
    BidNotice notice_msg = convert(bidding_task);
    notice_msg.task_profile.submission_time = node->now();
    bid_notice_pub->publish(notice_msg);
  }

  // Receive proposal and evaluate // todo think
  void receive_proposal(const BidProposal& msg)
  {
    std::cout << "[Auctioneer] Receive proposal for task_id: " 
              << msg.task_profile.task_id << std::endl;

    // check if bidding task is "mine"
    auto task_it = queue_bidding_tasks.find(msg.task_profile.task_id);
    if (task_it == queue_bidding_tasks.end())
      return; // not found

    // add submited proposal to the current bidding task
    auto submission = convert(msg);
    queue_bidding_tasks[msg.task_profile.task_id].submissions.push_back(submission);
  }

  // determine the winner within a bidding task instance
  void check_bidding_process()
  {
    // check if timeout is reached
    for( auto const& [id, tsk] : queue_bidding_tasks )
    {
      auto duration = std::chrono::steady_clock::now() 
        - tsk.bidding_task.task_profile.submission_time;
      if (duration > tsk.bidding_task.time_window )
      {
        std::cout << " - Deadline reached"<< std::endl;
        this->determine_winner(id, tsk.submissions);
      }
    }
    // std::cout << "Remaining: " << queue_bidding_tasks.size() 
    //           << " bidding tasks" << std::endl;
  }

  void determine_winner(
      const TaskID& task_id, 
      const std::vector<Submission>& submissions)
  { 
    rmf_utils::optional<Submission> winner = rmf_utils::nullopt;
    
    if(submissions.size() == 0)
    {
      std::cerr << " Bidding task has not received any bids"<< std::endl;
    }
    else
    {
      // Nominate and Evaluate Here
      Nomination task_nomination(submissions);
      auto _evaluator = LeastFleetDiffCostEvaluator();
      winner = task_nomination.evaluate(_evaluator);
      std::cout << "Found winning Fleet Adapter: " 
                << winner->bidder_name << std::endl;
    }

    // remove completed task from queue
    queue_bidding_tasks.erase(task_id);
    
    // check if bidding_result_callback fn is initailized
    if (bidding_result_callback)
      this->bidding_result_callback(task_id, winner);
  }
};

//==============================================================================
std::shared_ptr<Auctioneer> Auctioneer::make(
    const std::shared_ptr<rclcpp::Node>& node)
{
  auto pimpl = rmf_utils::make_unique_impl<Implementation>(node);

  if (pimpl)
  {
    auto auctioneer = std::shared_ptr<Auctioneer>(new Auctioneer());
    auctioneer->_pimpl = std::move(pimpl);
    return auctioneer;
  }
  return nullptr;
}

//==============================================================================
void Auctioneer::start_bidding(const BiddingTask& bidding_task)
{
  _pimpl->start_bidding(bidding_task);
}

//==============================================================================
void Auctioneer::receive_bidding_result(BiddingResultCallback result_callback)
{
  _pimpl->bidding_result_callback = std::move(result_callback);
}

//==============================================================================
// void Auctioneer::select_evaluator(const Nomination::Evaluator& evaluator)
// {
//   _evaluator = evaluator;
// }

//==============================================================================
Auctioneer::Auctioneer()
{
  // do nothing
}

} // namespace bidding
} // namespace rmf_task_ros2
