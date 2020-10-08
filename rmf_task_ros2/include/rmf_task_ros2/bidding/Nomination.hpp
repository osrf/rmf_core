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

// Skeleton for Task Nomination
// This is a class to select the best Nominee via an evaluator

#ifndef RMF_TASK_ROS2__NOMINATION_HPP
#define RMF_TASK_ROS2__NOMINATION_HPP

#include <iostream>
#include <rmf_traffic/Time.hpp>
#include <rmf_utils/optional.hpp>
#include <rmf_task_ros2/bidding/Bidding.hpp>

namespace rmf_task_ros2 {
namespace bidding {
//==============================================================================

using Submissions = std::vector<Submission>;

// This is a client class for evaluator to select the best Task nominees 
// from multiple submitted task estimations (nominees)
class Nomination
{
public:
  /// Constructor
  ///
  /// \param[in] input submissions of all potential nominees
  Nomination( const Submissions& submissions)
    : _submissions(submissions)
  {
    std::cout << " Submitted submissions for evaluation! size: " 
              << _submissions.size()<<  std::endl;
  }

  /// A pure abstract interface class for choosing the best nominee.
  class Evaluator
  {
  public:

    /// Given a list of nominee, choose the one that is the "best". It is up to
    /// the implementation of the Evaluator to decide how to rank.
    virtual std::size_t choose(const Submissions& submissions) const = 0;

    virtual ~Evaluator() = default;
  };

  /// Get the best winner from nominees
  ///
  /// \param[in] Evaluator 
  /// \return Winner
  rmf_utils::optional<Submission> evaluate(const Evaluator& evaluator)
  {
    if(_submissions.size()==0)
      return rmf_utils::nullopt;
    
    const std::size_t choice = evaluator.choose(_submissions);
    
    if(choice > _submissions.size())
      return rmf_utils::nullopt;
    
    return (_submissions)[choice];
  }

private:

  Submissions _submissions;
};

//==============================================================================
class LeastFleetCostEvaluator : public Nomination::Evaluator
{
public:
  std::size_t choose(const Submissions& submissions) const final
  {
    auto winner_it = submissions.begin();
    for ( auto nominee_it = submissions.begin(); 
          nominee_it != submissions.end(); ++nominee_it)
    {
      if (nominee_it->new_cost < winner_it->new_cost)
        winner_it = nominee_it;
    }
    return std::distance( submissions.begin(), winner_it );    
  };
};

class LeastFleetDiffCostEvaluator : public Nomination::Evaluator
{
public:
  std::size_t choose(const Submissions& submissions) const final
  {
    auto winner_it = submissions.begin();
    float winner_cost_diff = winner_it->new_cost - winner_it->prev_cost;
    for ( auto nominee_it = submissions.begin(); 
          nominee_it != submissions.end(); ++nominee_it)
    {
      float nominee_cost_diff = nominee_it->new_cost - nominee_it->prev_cost;
      if (nominee_cost_diff < winner_cost_diff)
      {
        winner_it = nominee_it;
        winner_cost_diff = nominee_cost_diff;
      }
    }
    return std::distance( submissions.begin(), winner_it );    
  };
};

class QuickestFinishEvaluator : public Nomination::Evaluator
{
public:
  // Documentation inherited
  std::size_t choose(const Submissions& submissions) const final
  {
    auto winner_it = submissions.begin();
    for ( auto nominee_it = submissions.begin(); 
          nominee_it != submissions.end(); ++nominee_it)
    {
      // TODO implementation Here!!! choose the least finish time 
      if (nominee_it->finish_time < winner_it->finish_time)
        winner_it = nominee_it;
    }
    return std::distance( submissions.begin(), winner_it );    
  };
};

} // namespace bidding
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__NOMINATION_HPP
