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
  std::shared_ptr<Evaluator> winner_evaluator;

  struct BiddingTask
  {
    BidNotice bid_notice;
    builtin_interfaces::msg::Time start_time;
    std::vector<bidding::Submission> submissions;
  };

  // non-sequential tasks
  std::map<TaskID, BiddingTask> ongoing_bidding_tasks;

  // sequential bidding tasks
  bool sequential;
  bool bidding_in_proccess = false;
  std::queue<BiddingTask> queue_bidding_tasks;

  using BidNoticePub = rclcpp::Publisher<BidNotice>;
  BidNoticePub::SharedPtr bid_notice_pub;

  using BidProposalSub = rclcpp::Subscription<BidProposal>;
  BidProposalSub::SharedPtr bid_proposal_sub;

  Implementation(const std::shared_ptr<rclcpp::Node>& node_,
    const bool sequential)
  : node(node_), sequential(sequential)
  {
    // default evaluator
    winner_evaluator = std::shared_ptr<LeastFleetDiffCostEvaluator>(
      new LeastFleetDiffCostEvaluator());

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
  void start_bidding(const BidNotice& bid_notice)
  {
    std::cout << "\n[Auctioneer] Add Bidding task_id: "
              << bid_notice.task_profile.task_id << " to queue"<< std::endl;

    BiddingTask bidding_task;
    bidding_task.bid_notice = bid_notice;
    bidding_task.start_time = node->now();

    if (sequential)
    {
      queue_bidding_tasks.push(bidding_task);
    }
    else
    {
      ongoing_bidding_tasks[bid_notice.task_profile.task_id] = bidding_task;
      bid_notice_pub->publish(bid_notice);
    }
  }

  // Receive proposal and evaluate
  void receive_proposal(const BidProposal& msg)
  {
    auto id_ = msg.task_profile.task_id;
    std::cout << "[Auctioneer] Receive proposal for task_id: "
              << id_ << std::endl;

    // check if bidding task is "mine", if found
    // add submited proposal to the current bidding tasks list
    if (sequential)
    {
      if (queue_bidding_tasks.front().bid_notice.task_profile.task_id == id_)
        queue_bidding_tasks.front().submissions.push_back(convert(msg));
    }
    else
    {
      if (ongoing_bidding_tasks.count(id_))
        ongoing_bidding_tasks[id_].submissions.push_back(convert(msg));
    }
  }

  // determine the winner within a bidding task instance
  void check_bidding_process()
  {
    // ugly!!!! to be clean up
    if (sequential)
    {
      if (queue_bidding_tasks.size() == 0)
        return;

      // Executing the task in front queue
      auto front_task = queue_bidding_tasks.front();

      if (bidding_in_proccess)
      {
        if (determine_winner(front_task))
        {
          queue_bidding_tasks.pop();
          bidding_in_proccess = false;
        }
      }
      else
      {
        std::cout << " - Start new bidding task: "
                  << front_task.bid_notice.task_profile.task_id << std::endl;
        front_task.start_time = node->now();
        bid_notice_pub->publish(front_task.bid_notice);
        bidding_in_proccess = true;
      }
    }
    else
    {
      // check if timeout is reached
      for (auto it = ongoing_bidding_tasks.begin();
        it != ongoing_bidding_tasks.end(); )
      {
        // bidding task
        if (determine_winner(it->second))
          it = ongoing_bidding_tasks.erase(it);
        else
          ++it;
      }
    }
    std::cout << "." <<std::flush;
  }

  bool determine_winner(const BiddingTask& bidding_task)
  {
    auto duration = node->now() - bidding_task.start_time;

    if (duration > bidding_task.bid_notice.time_window)
    {
      auto id = bidding_task.bid_notice.task_profile.task_id;
      std::cout << " - Deadline reached: "<< id << std::endl;
      rmf_utils::optional<Submission> winner = rmf_utils::nullopt;

      if (bidding_task.submissions.size() == 0)
      {
        std::cerr << " Bidding task has not received any bids"<< std::endl;
      }
      else
      {
        winner = evaluate(bidding_task.submissions);
        std::cout << "Found winning Fleet Adapter: "
                  << winner->fleet_name << std::endl;
      }

      // Call the user defined callback function
      if (bidding_result_callback)
        bidding_result_callback(id, winner);

      return true;
    }
    return false;
  }

  rmf_utils::optional<Submission> evaluate(const Submissions& submissions)
  {
    if (submissions.size() == 0)
      return rmf_utils::nullopt;

    const std::size_t choice = winner_evaluator->choose(submissions);

    if (choice > submissions.size())
      return rmf_utils::nullopt;

    return (submissions)[choice];
  }
};

//==============================================================================
std::shared_ptr<Auctioneer> Auctioneer::make(
  const std::shared_ptr<rclcpp::Node>& node,
  const bool sequential)
{
  auto pimpl = rmf_utils::make_unique_impl<Implementation>(node, sequential);

  if (pimpl)
  {
    auto auctioneer = std::shared_ptr<Auctioneer>(new Auctioneer());
    auctioneer->_pimpl = std::move(pimpl);
    return auctioneer;
  }
  return nullptr;
}

//==============================================================================
void Auctioneer::start_bidding(const BidNotice& bid_notice)
{
  _pimpl->start_bidding(bid_notice);
}

//==============================================================================
void Auctioneer::receive_bidding_result(BiddingResultCallback result_callback)
{
  _pimpl->bidding_result_callback = std::move(result_callback);
}

//==============================================================================
void Auctioneer::select_evaluator(
  std::shared_ptr<Auctioneer::Evaluator> evaluator)
{
  _pimpl->winner_evaluator = std::move(evaluator);
}

//==============================================================================
rmf_utils::optional<Submission> Auctioneer::evaluate(
  const Submissions& submissions)
{
  return _pimpl->evaluate(submissions);
}

//==============================================================================
Auctioneer::Auctioneer()
{
  // do nothing
}

//==============================================================================
std::size_t LeastFleetDiffCostEvaluator::choose(
  const Submissions& submissions) const
{
  auto winner_it = submissions.begin();
  float winner_cost_diff = winner_it->new_cost - winner_it->prev_cost;
  for (auto nominee_it = submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    float nominee_cost_diff = nominee_it->new_cost - nominee_it->prev_cost;
    if (nominee_cost_diff < winner_cost_diff)
    {
      winner_it = nominee_it;
      winner_cost_diff = nominee_cost_diff;
    }
  }
  return std::distance(submissions.begin(), winner_it);
}

//==============================================================================
std::size_t LeastFleetCostEvaluator::choose(
  const Submissions& submissions) const
{
  auto winner_it = submissions.begin();
  for (auto nominee_it = submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    if (nominee_it->new_cost < winner_it->new_cost)
      winner_it = nominee_it;
  }
  return std::distance(submissions.begin(), winner_it);
}

//==============================================================================
std::size_t QuickestFinishEvaluator::choose(
  const Submissions& submissions) const
{
  auto winner_it = submissions.begin();
  for (auto nominee_it = submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    if (nominee_it->finish_time < winner_it->finish_time)
      winner_it = nominee_it;
  }
  return std::distance(submissions.begin(), winner_it);
}

} // namespace bidding
} // namespace rmf_task_ros2
